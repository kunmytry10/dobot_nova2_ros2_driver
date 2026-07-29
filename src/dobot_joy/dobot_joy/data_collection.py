import json
import queue
import subprocess
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

import cv2
import message_filters
import rclpy
from cv_bridge import CvBridge
from dobot_interfaces.msg import DobotState, GripperStatus, TeleopAction
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, JointState, Joy
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger


CAMERA_NAMES = ("wrist", "global")


def _stamp_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _camera_info_dict(msg: CameraInfo) -> dict:
    return {
        "frame_id": str(msg.header.frame_id),
        "width": int(msg.width),
        "height": int(msg.height),
        "distortion_model": str(msg.distortion_model),
        "d": [float(value) for value in msg.d],
        "k": [float(value) for value in msg.k],
        "r": [float(value) for value in msg.r],
        "p": [float(value) for value in msg.p],
    }


class DataCollectionNode(Node):
    """Record synchronized wrist/global images, observations, and actions."""

    def __init__(self):
        super().__init__("dobot_data_collection")
        self.dataset_root = Path(
            str(self.declare_parameter("dataset_root", "data_collection").value)
        )
        self.wrist_image_topic = str(
            self.declare_parameter(
                "wrist_image_topic", "/camera/color/image_raw"
            ).value
        )
        self.wrist_camera_info_topic = str(
            self.declare_parameter(
                "wrist_camera_info_topic", "/camera/color/camera_info"
            ).value
        )
        self.global_image_topic = str(
            self.declare_parameter(
                "global_image_topic", "/global_camera/color/image_raw"
            ).value
        )
        self.global_camera_info_topic = str(
            self.declare_parameter(
                "global_camera_info_topic",
                "/global_camera/color/camera_info",
            ).value
        )
        self.sample_rate_hz = float(
            self.declare_parameter("sample_rate_hz", 10.0).value
        )
        self.max_image_skew_sec = float(
            self.declare_parameter("max_image_skew_sec", 0.05).value
        )
        self.sync_queue_size = int(
            self.declare_parameter("sync_queue_size", 20).value
        )
        self.write_queue_size = int(
            self.declare_parameter("write_queue_size", 32).value
        )
        self.jpeg_quality = int(self.declare_parameter("jpeg_quality", 95).value)
        self.state_timeout_sec = float(
            self.declare_parameter("state_timeout_sec", 1.0).value
        )
        self.task_instruction = str(
            self.declare_parameter("task_instruction", "").value
        )
        task_file_value = str(self.declare_parameter("task_file", "").value)
        self.task_file = Path(task_file_value) if task_file_value else None
        self.lerobot_enabled = bool(
            self.declare_parameter("lerobot_enabled", True).value
        )
        self.lerobot_python = Path(
            str(self.declare_parameter("lerobot_python", "python3").value)
        )
        lerobot_root_value = str(
            self.declare_parameter("lerobot_dataset_root", "").value
        )
        self.lerobot_dataset_root = (
            Path(lerobot_root_value)
            if lerobot_root_value
            else self.dataset_root / "lerobot_pi05"
        )
        self.lerobot_repo_id = str(
            self.declare_parameter(
                "lerobot_repo_id", "local/dobot_nova2_pi05"
            ).value
        )
        self.lerobot_export_timeout_sec = float(
            self.declare_parameter("lerobot_export_timeout_sec", 900.0).value
        )
        self._lerobot_export_script = Path(__file__).with_name("lerobot_export.py")

        self.bridge = CvBridge()
        self._lock = threading.RLock()
        self._recording = False
        self._finishing = False
        self._session_dir = None
        self._started_wall_sec = 0.0
        self._started_monotonic = 0.0
        self._last_sample_monotonic = 0.0
        self._next_sample_monotonic = 0.0
        self._sample_clock_initialized = False
        self._next_frame_slot = 0
        self._missed_sample_slots = 0
        self._max_sample_timing_error_sec = 0.0
        self._sample_count = 0
        self._enqueued_count = 0
        self._dropped_pairs = 0
        self._write_errors = 0
        self._last_pair_skew_sec = None
        self._max_pair_skew_sec = 0.0
        self._observations_file = None
        self._events_file = None
        self._latest = {}
        self._received = {}
        self._camera_info = {name: None for name in CAMERA_NAMES}
        self._lerobot_export = None
        self._lerobot_environment = self._probe_lerobot_environment()
        self._pending_session_dir = self._find_pending_session()

        self._write_queue = queue.Queue(maxsize=max(1, self.write_queue_size))
        self._writer_thread = threading.Thread(
            target=self._writer_loop,
            name="dobot-data-writer",
            daemon=True,
        )
        self._writer_thread.start()

        self.create_subscription(DobotState, "/dobot_state", self._on_robot, 10)
        self.create_subscription(JointState, "/joint_states", self._on_joints, 10)
        self.create_subscription(Float64MultiArray, "/tcp_pose", self._on_tcp, 10)
        self.create_subscription(GripperStatus, "/gripper_state", self._on_gripper, 10)
        self.create_subscription(Joy, "/joy", self._on_joy, 10)
        self.create_subscription(
            TeleopAction, "/joy/teleop_action", self._on_action, 10
        )
        self.create_subscription(
            CameraInfo,
            self.wrist_camera_info_topic,
            lambda msg: self._on_camera_info("wrist", msg),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            self.global_camera_info_topic,
            lambda msg: self._on_camera_info("global", msg),
            qos_profile_sensor_data,
        )

        self._wrist_image_sub = message_filters.Subscriber(
            self,
            Image,
            self.wrist_image_topic,
            qos_profile=qos_profile_sensor_data,
        )
        self._global_image_sub = message_filters.Subscriber(
            self,
            Image,
            self.global_image_topic,
            qos_profile=qos_profile_sensor_data,
        )
        self._wrist_image_sub.registerCallback(
            lambda msg: self._mark_image_received("wrist", msg)
        )
        self._global_image_sub.registerCallback(
            lambda msg: self._mark_image_received("global", msg)
        )
        self._image_sync = message_filters.ApproximateTimeSynchronizer(
            [self._wrist_image_sub, self._global_image_sub],
            queue_size=max(2, self.sync_queue_size),
            slop=max(0.0, self.max_image_skew_sec),
        )
        self._image_sync.registerCallback(self._on_image_pair)

        self.create_service(Trigger, "/data_collection/start", self._start)
        self.create_service(Trigger, "/data_collection/stop", self._stop)
        self.create_service(Trigger, "/data_collection/accept", self._accept)
        self.create_service(Trigger, "/data_collection/reject", self._reject)
        self.create_service(Trigger, "/data_collection/status", self._status)
        self.create_timer(0.25, self._check_recording_sources)

    def destroy_node(self):
        with self._lock:
            recording = self._recording
        if recording:
            self._finish_session("node_shutdown", complete=False)
        self._write_queue.put(None)
        self._writer_thread.join(timeout=5.0)
        super().destroy_node()

    def _remember(self, name: str, value) -> None:
        with self._lock:
            self._latest[name] = value
            self._received[name] = time.monotonic()

    def _on_robot(self, msg: DobotState):
        state = {
            "connected": bool(msg.connected),
            "feedback_valid": bool(msg.feedback_valid),
            "robot_mode": int(msg.robot_mode),
            "robot_mode_text": str(msg.robot_mode_text),
            "speed_scaling": float(msg.speed_scaling),
            "enable_status": int(msg.enable_status),
            "running_status": int(msg.running_status),
            "error_status": int(msg.error_status),
            "drag_status": int(msg.drag_status),
            "stamp_sec": float(msg.stamp_sec),
            "q_target_deg": [float(value) for value in msg.q_target],
            "tcp_target_mm_deg": [float(value) for value in msg.tcp_target],
        }
        self._remember("robot", state)
        invalid = (
            not state["connected"]
            or not state["feedback_valid"]
            or state["error_status"]
            or state["robot_mode"] == 9
        )
        with self._lock:
            should_stop = self._recording and invalid
        if should_stop:
            path, _ = self._finish_session("robot_not_ready", complete=False)
            self.get_logger().error(f"data collection stopped incomplete: {path}")

    def _on_joints(self, msg: JointState):
        self._remember(
            "joints",
            {
                "names": list(msg.name),
                "position_rad": [float(value) for value in msg.position],
                "velocity_rad_s": [float(value) for value in msg.velocity],
                "effort": [float(value) for value in msg.effort],
                "stamp_sec": _stamp_sec(msg.header.stamp),
            },
        )

    def _on_tcp(self, msg: Float64MultiArray):
        self._remember("tcp_pose_mm_deg", [float(value) for value in msg.data[:6]])

    def _on_gripper(self, msg: GripperStatus):
        self._remember(
            "gripper",
            {
                "connected": bool(msg.connected),
                "success": bool(msg.success),
                "init_state": int(msg.init_state),
                "grip_state": int(msg.grip_state),
                "position_permille": int(msg.position_permille),
                "opening_mm": float(msg.opening_mm),
                "force_percent": int(msg.force_percent),
                "moving": bool(msg.moving),
                "object_detected": bool(msg.object_detected),
                "object_dropped": bool(msg.object_dropped),
            },
        )

    def _on_joy(self, msg: Joy):
        self._remember(
            "joy",
            {
                "axes": [float(value) for value in msg.axes],
                "buttons": [int(value) for value in msg.buttons],
                "stamp_sec": _stamp_sec(msg.header.stamp),
            },
        )

    def _on_action(self, msg: TeleopAction):
        self._remember(
            "action",
            {
                "stamp_sec": _stamp_sec(msg.stamp),
                "axis_id": str(msg.axis_id),
                "cartesian_jog_normalized": [
                    float(value) for value in msg.cartesian_jog
                ],
                "motion_active": bool(msg.motion_active),
                "deadman": bool(msg.deadman),
                "coord_type": int(msg.coord_type),
                "user": int(msg.user),
                "tool": int(msg.tool),
                "gripper_action": str(msg.gripper_action),
                "gripper_target_mm": float(msg.gripper_target_mm),
                "gripper_target_normalized": float(
                    msg.gripper_target_normalized
                ),
            },
        )

    def _on_camera_info(self, camera: str, msg: CameraInfo):
        with self._lock:
            self._camera_info[camera] = _camera_info_dict(msg)
            self._received[f"{camera}_camera_info"] = time.monotonic()

    def _mark_image_received(self, camera: str, msg: Image):
        del msg
        with self._lock:
            self._received[f"{camera}_image"] = time.monotonic()

    def _on_image_pair(self, wrist_msg: Image, global_msg: Image):
        now = time.monotonic()
        wrist_stamp = _stamp_sec(wrist_msg.header.stamp)
        global_stamp = _stamp_sec(global_msg.header.stamp)
        skew_sec = abs(wrist_stamp - global_stamp)
        with self._lock:
            self._received["image_pair"] = now
            self._last_pair_skew_sec = skew_sec
            self._max_pair_skew_sec = max(self._max_pair_skew_sec, skew_sec)
            if not self._recording or self._finishing:
                return
            period = 1.0 / self.sample_rate_hz if self.sample_rate_hz > 0.0 else 0.0
            if not self._sample_clock_initialized:
                self._next_sample_monotonic = now
                self._sample_clock_initialized = True
            early_tolerance = period * 0.5
            if period and now < self._next_sample_monotonic - early_tolerance:
                return
            while period and now > self._next_sample_monotonic + early_tolerance:
                self._missed_sample_slots += 1
                self._next_frame_slot += 1
                self._next_sample_monotonic += period
            frame_slot = self._next_frame_slot
            scheduled_monotonic = self._next_sample_monotonic
            timing_error_sec = now - scheduled_monotonic
            sample_id = self._enqueued_count + 1
            item = {
                "sample_id": sample_id,
                "frame_slot": frame_slot,
                "t": frame_slot * period,
                "capture_monotonic": now,
                "timing_error_sec": timing_error_sec,
                "session_dir": self._session_dir,
                "wrist_msg": wrist_msg,
                "global_msg": global_msg,
                "skew_sec": skew_sec,
                "snapshot": json.loads(json.dumps(self._latest)),
            }
            try:
                self._write_queue.put_nowait(item)
            except queue.Full:
                self._dropped_pairs += 1
                self.get_logger().error("data write queue full; synchronized pair dropped")
                return
            self._last_sample_monotonic = now
            self._max_sample_timing_error_sec = max(
                self._max_sample_timing_error_sec,
                abs(timing_error_sec),
            )
            self._next_frame_slot += 1
            self._next_sample_monotonic += period
            self._enqueued_count = sample_id

    def _writer_loop(self):
        while True:
            item = self._write_queue.get()
            try:
                if item is None:
                    return
                self._write_sample(item)
            except Exception as exc:
                with self._lock:
                    self._write_errors += 1
                self.get_logger().error(f"data sample write failed: {exc}")
            finally:
                self._write_queue.task_done()

    def _write_sample(self, item: dict):
        sample_id = item["sample_id"]
        session_dir = item["session_dir"]
        image_paths = {}
        image_messages = {
            "wrist": item["wrist_msg"],
            "global": item["global_msg"],
        }
        for camera, message in image_messages.items():
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            relative_path = (
                Path("images") / camera / f"frame_{sample_id:06d}.jpg"
            )
            ok = cv2.imwrite(
                str(session_dir / relative_path),
                image,
                [cv2.IMWRITE_JPEG_QUALITY, max(1, min(100, self.jpeg_quality))],
            )
            if not ok:
                raise RuntimeError(f"cv2.imwrite returned false for {camera}")
            image_paths[camera] = str(relative_path)

        wrist_msg = image_messages["wrist"]
        global_msg = image_messages["global"]
        observation = {
            "sample_id": sample_id,
            "frame_slot": int(item["frame_slot"]),
            "t": float(item["t"]),
            "sample_timing_error_sec": float(item["timing_error_sec"]),
            "images": {
                "wrist": {
                    "path": image_paths["wrist"],
                    "stamp_sec": _stamp_sec(wrist_msg.header.stamp),
                    "frame_id": str(wrist_msg.header.frame_id),
                },
                "global": {
                    "path": image_paths["global"],
                    "stamp_sec": _stamp_sec(global_msg.header.stamp),
                    "frame_id": str(global_msg.header.frame_id),
                },
            },
            "image_pair_skew_sec": float(item["skew_sec"]),
            **item["snapshot"],
        }
        with self._lock:
            if session_dir != self._session_dir or self._observations_file is None:
                return
            self._observations_file.write(
                json.dumps(observation, separators=(",", ":")) + "\n"
            )
            self._observations_file.flush()
            self._sample_count += 1

    def _start(self, request, response):
        del request
        with self._lock:
            if self._recording or self._finishing:
                response.success = False
                response.message = f"data collection already active: {self._session_dir}"
                return response
            if self._pending_session_dir is not None:
                response.success = False
                response.message = (
                    "data collection pending review; short Back to accept or "
                    f"hold Back to reject: {self._pending_session_dir}"
                )
                return response
            try:
                self._refresh_task_instruction()
            except OSError as exc:
                response.success = False
                response.message = f"failed to read data collection task: {exc}"
                return response
            ready, message = self._ready_to_start()
            if not ready:
                response.success = False
                response.message = message
                return response
            try:
                self._begin_session()
            except Exception as exc:
                response.success = False
                response.message = f"failed to start data collection: {exc}"
                return response
            response.success = True
            response.message = f"data collection started: {self._session_dir}"
            self.get_logger().info(response.message)
            return response

    def _stop(self, request, response):
        del request
        with self._lock:
            recording = self._recording
            pending = self._pending_session_dir
            if self._finishing:
                response.success = False
                response.message = "data collection is busy"
                return response
        if pending is not None and not recording:
            return self._accept(None, response)
        if not recording:
            response.success = False
            response.message = "data collection is not active"
            return response
        path, complete = self._finish_session("user_stop", complete=None)
        response.success = complete
        if complete and self.lerobot_enabled:
            response.message = (
                f"data collection pending review: {path}; "
                "short Back to accept or hold Back to reject"
            )
        elif complete:
            response.message = f"data collection saved: {path}"
        else:
            response.message = (
                f"data collection incomplete: {path}; samples={self._sample_count}, "
                f"dropped_pairs={self._dropped_pairs}, "
                f"write_errors={self._write_errors}, "
                f"missed_sample_slots={self._missed_sample_slots}"
            )
        self.get_logger().info(response.message)
        return response

    def _accept(self, request, response):
        del request
        with self._lock:
            path = self._pending_session_dir
            if path is None:
                response.success = False
                response.message = "data collection has no pending episode"
                return response
            if self._recording or self._finishing:
                response.success = False
                response.message = "data collection is busy"
                return response
            self._finishing = True
        export = self._export_to_lerobot(path) if self.lerobot_enabled else {
            "exported": True,
            "disabled": True,
        }
        accepted = bool(export.get("exported"))
        with self._lock:
            self._lerobot_export = export
            if accepted:
                self._update_episode_curation(
                    path,
                    status="accepted",
                    episode_success=True,
                    lerobot_export=export,
                )
                self._append_episode_event(
                    path,
                    "accept",
                    {"lerobot_episode_index": export.get("episode_index")},
                )
                self._pending_session_dir = None
            else:
                self._update_episode_curation(
                    path,
                    status="pending",
                    episode_success=None,
                    lerobot_export=export,
                )
                self._append_episode_event(
                    path,
                    "accept_failed",
                    {"error": export.get("error", "LeRobot export failed")},
                )
            self._finishing = False
        response.success = accepted
        response.message = (
            f"data collection saved: {path}"
            if accepted
            else (
                f"data collection accept failed; episode remains pending: {path}; "
                f"lerobot_error={export.get('error', 'LeRobot export failed')}"
            )
        )
        (self.get_logger().info if accepted else self.get_logger().error)(
            response.message
        )
        return response

    def _reject(self, request, response):
        del request
        with self._lock:
            recording = self._recording
            pending = self._pending_session_dir
            if self._finishing:
                response.success = False
                response.message = "data collection is busy"
                return response
        if recording:
            pending, complete = self._finish_session("user_reject", complete=None)
            if not complete:
                response.success = True
                response.message = f"data collection discarded incomplete: {pending}"
                self.get_logger().info(response.message)
                return response
        if pending is None:
            response.success = False
            response.message = "data collection has no pending episode"
            return response
        with self._lock:
            self._update_episode_curation(
                pending,
                status="rejected",
                episode_success=False,
                lerobot_export=None,
            )
            self._append_episode_event(pending, "reject")
            self._pending_session_dir = None
        response.success = True
        response.message = f"data collection rejected and kept as raw data: {pending}"
        self.get_logger().info(response.message)
        return response

    def _status(self, request, response):
        del request
        now = time.monotonic()
        with self._lock:
            source_ages = {
                name: round(now - self._received[name], 3)
                for name in (
                    "wrist_image",
                    "global_image",
                    "image_pair",
                )
                if name in self._received
            }
            response.success = True
            response.message = json.dumps(
                {
                    "recording": self._recording,
                    "finishing": self._finishing,
                    "phase": (
                        "recording"
                        if self._recording
                        else "pending"
                        if self._pending_session_dir is not None
                        else "idle"
                    ),
                    "session_dir": str(self._session_dir or ""),
                    "pending_session_dir": str(self._pending_session_dir or ""),
                    "sample_count": self._sample_count,
                    "queued_sample_count": self._write_queue.qsize(),
                    "dropped_pairs": self._dropped_pairs,
                    "write_errors": self._write_errors,
                    "missed_sample_slots": self._missed_sample_slots,
                    "max_sample_timing_error_ms": round(
                        self._max_sample_timing_error_sec * 1000.0, 3
                    ),
                    "last_pair_skew_ms": (
                        None
                        if self._last_pair_skew_sec is None
                        else round(self._last_pair_skew_sec * 1000.0, 3)
                    ),
                    "max_pair_skew_ms": round(
                        self._max_pair_skew_sec * 1000.0, 3
                    ),
                    "source_age_sec": source_ages,
                    "task_instruction": self.task_instruction,
                    "task_file": str(self.task_file or ""),
                    "lerobot_enabled": self.lerobot_enabled,
                    "lerobot_dataset_root": str(self.lerobot_dataset_root),
                    "lerobot_repo_id": self.lerobot_repo_id,
                    "lerobot_environment": self._lerobot_environment,
                    "last_lerobot_export": self._lerobot_export,
                }
            )
            return response

    def _refresh_task_instruction(self):
        if self.task_file is None:
            return
        if self.task_file.is_file():
            self.task_instruction = self.task_file.read_text(encoding="utf-8").strip()

    def _required_sources(self):
        return [
            "robot",
            "joints",
            "tcp_pose_mm_deg",
            "gripper",
            "joy",
            "action",
            "wrist_image",
            "global_image",
            "image_pair",
            "wrist_camera_info",
            "global_camera_info",
        ]

    def _ready_to_start(self):
        if self.lerobot_enabled and not self._lerobot_environment.get("available"):
            return (
                False,
                "data collection rejected: LeRobot v3 environment is not ready: "
                + self._lerobot_environment.get("error", "unknown error"),
            )
        if self.lerobot_enabled and not self.task_instruction.strip():
            return (
                False,
                "data collection rejected: set a LeRobot task with make data-task "
                'TASK:="..."',
            )
        if self.sample_rate_hz <= 0.0 or not self.sample_rate_hz.is_integer():
            return False, "data collection rejected: LeRobot fps must be a positive integer"
        now = time.monotonic()
        required = self._required_sources()
        missing = [name for name in required if name not in self._received]
        if missing:
            return False, "data collection not ready; missing: " + ", ".join(missing)
        dynamic_sources = [
            name for name in required if not name.endswith("camera_info")
        ]
        stale = [
            name
            for name in dynamic_sources
            if now - self._received.get(name, 0.0) > self.state_timeout_sec
        ]
        if stale:
            return False, "data collection not ready; stale: " + ", ".join(stale)
        robot = self._latest.get("robot", {})
        if not robot.get("connected") or not robot.get("feedback_valid"):
            return False, "data collection rejected: robot feedback is not ready"
        if robot.get("error_status") or robot.get("robot_mode") == 9:
            return False, "data collection rejected: robot is in error state"
        if robot.get("enable_status") != 1:
            return False, "data collection rejected: robot is not enabled"
        return True, "ready"

    def _check_recording_sources(self):
        now = time.monotonic()
        with self._lock:
            if not self._recording or self._finishing:
                return
            dynamic_sources = [
                name
                for name in self._required_sources()
                if not name.endswith("camera_info")
            ]
            stale = [
                name
                for name in dynamic_sources
                if now - self._received.get(name, 0.0) > self.state_timeout_sec
            ]
        if stale:
            path, _ = self._finish_session("source_timeout", complete=False)
            self.get_logger().error(
                "data collection stopped incomplete; stale: "
                + ", ".join(stale)
                + f"; {path}"
            )

    def _begin_session(self):
        self.dataset_root.mkdir(parents=True, exist_ok=True)
        base_name = time.strftime("episode_%Y%m%d_%H%M%S")
        session_dir = self.dataset_root / base_name
        suffix = 1
        while session_dir.exists():
            session_dir = self.dataset_root / f"{base_name}_{suffix:02d}"
            suffix += 1
        for camera in CAMERA_NAMES:
            (session_dir / "images" / camera).mkdir(parents=True)
        (session_dir / "camera_info").mkdir()
        self._session_dir = session_dir
        self._observations_file = (session_dir / "steps.jsonl").open(
            "w", encoding="utf-8"
        )
        self._events_file = (session_dir / "events.jsonl").open("w", encoding="utf-8")
        self._started_wall_sec = time.time()
        self._started_monotonic = time.monotonic()
        self._last_sample_monotonic = 0.0
        self._next_sample_monotonic = self._started_monotonic
        self._sample_clock_initialized = False
        self._next_frame_slot = 0
        self._missed_sample_slots = 0
        self._max_sample_timing_error_sec = 0.0
        self._sample_count = 0
        self._enqueued_count = 0
        self._dropped_pairs = 0
        self._write_errors = 0
        self._max_pair_skew_sec = 0.0
        self._lerobot_export = None
        self._recording = True
        self._finishing = False
        self._write_event("start")
        for camera, info in self._camera_info.items():
            if info is not None:
                (session_dir / "camera_info" / f"{camera}.json").write_text(
                    json.dumps(info, indent=2), encoding="utf-8"
                )
        self._write_metadata(
            complete=False,
            stop_reason="",
            curation_status="recording",
        )

    def _finish_session(self, stop_reason: str, complete=None):
        with self._lock:
            if self._session_dir is None:
                return None, False
            self._recording = False
            self._finishing = True
            path = self._session_dir
        self._write_queue.join()
        with self._lock:
            actual_complete = (
                self._sample_count > 0
                and self._dropped_pairs == 0
                and self._write_errors == 0
                and self._missed_sample_slots == 0
            )
            if complete is not None:
                actual_complete = bool(complete) and actual_complete
            self._write_event("stop", {"reason": stop_reason})
            if self._observations_file is not None:
                self._observations_file.close()
                self._observations_file = None
            if self._events_file is not None:
                self._events_file.close()
                self._events_file = None
            self._write_metadata(
                complete=actual_complete,
                stop_reason=stop_reason,
                curation_status=(
                    "pending"
                    if actual_complete and self.lerobot_enabled
                    else "accepted"
                    if actual_complete
                    else "incomplete"
                ),
            )
        if actual_complete and self.lerobot_enabled:
            with self._lock:
                self._pending_session_dir = path
        elif self.lerobot_enabled:
            self._lerobot_export = {
                "exported": False,
                "error": "raw episode is incomplete; LeRobot export skipped",
            }
            with self._lock:
                self._write_metadata(
                    complete=False,
                    stop_reason=stop_reason,
                    curation_status="incomplete",
                )
        with self._lock:
            self._session_dir = None
            self._finishing = False
            return path, actual_complete

    def _find_pending_session(self):
        if not self.dataset_root.is_dir():
            return None
        pending = []
        for path in self.dataset_root.glob("episode_*"):
            metadata_path = path / "metadata.json"
            try:
                metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
            except (OSError, json.JSONDecodeError):
                continue
            if metadata.get("curation_status") == "pending":
                pending.append(path)
        if not pending:
            return None
        pending.sort(key=lambda path: path.stat().st_mtime)
        if len(pending) > 1:
            self.get_logger().warning(
                f"found {len(pending)} pending episodes; using latest: {pending[-1]}"
            )
        else:
            self.get_logger().info(f"recovered pending episode: {pending[-1]}")
        return pending[-1]

    def _update_episode_curation(
        self,
        episode_dir: Path,
        status: str,
        episode_success,
        lerobot_export,
    ):
        target = episode_dir / "metadata.json"
        metadata = json.loads(target.read_text(encoding="utf-8"))
        metadata["updated_utc"] = _utc_now()
        metadata["curation_status"] = status
        metadata["episode_success"] = episode_success
        metadata.setdefault("lerobot", {})["export"] = lerobot_export
        temporary = episode_dir / "metadata.json.tmp"
        temporary.write_text(json.dumps(metadata, indent=2), encoding="utf-8")
        temporary.replace(target)

    def _append_episode_event(self, episode_dir: Path, event: str, details=None):
        payload = {"event": event, "stamp_sec": time.time(), "utc": _utc_now()}
        if details:
            payload.update(details)
        with (episode_dir / "events.jsonl").open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(payload, separators=(",", ":")) + "\n")

    def _probe_lerobot_environment(self):
        if not self.lerobot_enabled:
            return {"available": False, "disabled": True}
        result = self._run_lerobot(["check"], timeout_sec=60.0)
        if result.get("returncode") == 0:
            result.pop("returncode", None)
            result["available"] = True
            return result
        return {
            "available": False,
            "error": result.get("error", "LeRobot environment check failed"),
        }

    def _export_to_lerobot(self, raw_episode: Path):
        result = self._run_lerobot(
            [
                "export",
                str(raw_episode),
                str(self.lerobot_dataset_root),
                self.lerobot_repo_id,
                str(int(round(self.sample_rate_hz))),
            ],
            timeout_sec=self.lerobot_export_timeout_sec,
        )
        result.pop("returncode", None)
        if not result.get("exported"):
            result.setdefault("error", "LeRobot export failed")
        return result

    def _run_lerobot(self, arguments, timeout_sec):
        command = [
            str(self.lerobot_python),
            str(self._lerobot_export_script),
            *arguments,
        ]
        try:
            completed = subprocess.run(
                command,
                capture_output=True,
                check=False,
                text=True,
                timeout=max(1.0, timeout_sec),
            )
        except (OSError, subprocess.TimeoutExpired) as exc:
            return {"returncode": -1, "error": str(exc)}
        if completed.returncode:
            detail = completed.stderr.strip() or completed.stdout.strip()
            return {"returncode": completed.returncode, "error": detail[-2000:]}
        try:
            result = json.loads(completed.stdout)
        except json.JSONDecodeError as exc:
            return {
                "returncode": -1,
                "error": f"invalid LeRobot exporter response: {exc}",
            }
        result["returncode"] = 0
        return result

    def _write_event(self, event: str, details=None):
        if self._events_file is None:
            return
        payload = {"event": event, "stamp_sec": time.time(), "utc": _utc_now()}
        if details:
            payload.update(details)
        self._events_file.write(json.dumps(payload, separators=(",", ":")) + "\n")
        self._events_file.flush()

    def _write_metadata(
        self,
        complete: bool,
        stop_reason: str,
        curation_status: str,
    ):
        acquisition_complete = (
            self._sample_count > 0
            and self._dropped_pairs == 0
            and self._write_errors == 0
            and self._missed_sample_slots == 0
        )
        payload = {
            "format_version": 2,
            "dataset_type": "dobot_teleoperation_episode",
            "complete": bool(complete),
            "acquisition_complete": acquisition_complete,
            "created_utc": datetime.fromtimestamp(
                self._started_wall_sec, timezone.utc
            ).isoformat(),
            "updated_utc": _utc_now(),
            "duration_sec": max(0.0, time.monotonic() - self._started_monotonic),
            "sample_count": int(self._sample_count),
            "sample_rate_hz": float(self.sample_rate_hz),
            "missed_sample_slots": int(self._missed_sample_slots),
            "max_sample_timing_error_sec": float(
                self._max_sample_timing_error_sec
            ),
            "dropped_pairs": int(self._dropped_pairs),
            "write_errors": int(self._write_errors),
            "max_pair_skew_sec": float(self._max_pair_skew_sec),
            "configured_max_image_skew_sec": float(self.max_image_skew_sec),
            "stop_reason": stop_reason,
            "task_instruction": self.task_instruction,
            "curation_status": curation_status,
            "episode_success": (
                True
                if curation_status == "accepted"
                else False
                if curation_status == "rejected"
                else None
            ),
            "lerobot": {
                "enabled": self.lerobot_enabled,
                "dataset_format": "LeRobotDataset v3.0",
                "dataset_root": str(self.lerobot_dataset_root),
                "repo_id": self.lerobot_repo_id,
                "export": self._lerobot_export,
            },
            "units": {
                "joint_position": "rad",
                "joint_velocity": "rad_s",
                "tcp_pose": "mm_deg",
                "gripper_opening": "mm",
                "action_cartesian": "normalized_fixed_rate_direction",
                "action_gripper": "normalized_opening",
                "image_pair_skew": "sec",
            },
            "topics": {
                "wrist_image": self.wrist_image_topic,
                "wrist_camera_info": self.wrist_camera_info_topic,
                "global_image": self.global_image_topic,
                "global_camera_info": self.global_camera_info_topic,
                "joint_states": "/joint_states",
                "tcp_pose": "/tcp_pose",
                "robot_state": "/dobot_state",
                "gripper_state": "/gripper_state",
                "joy": "/joy",
                "action": "/joy/teleop_action",
            },
            "training_schema": {
                "step_key": "sample_id",
                "observation": [
                    "images.wrist",
                    "images.global",
                    "joints.position_rad",
                    "joints.velocity_rad_s",
                    "tcp_pose_mm_deg",
                    "gripper.opening_mm",
                    "robot",
                    "robot.q_target_deg",
                    "robot.tcp_target_mm_deg",
                ],
                "action": {
                    "vector_order": ["X", "Y", "Z", "Rx", "Ry", "Rz"],
                    "cartesian_units": "normalized_fixed_rate_direction",
                    "gripper": "target_opening_normalized",
                },
            },
            "files": {
                "steps": "steps.jsonl",
                "events": "events.jsonl",
                "camera_info": {
                    "wrist": "camera_info/wrist.json",
                    "global": "camera_info/global.json",
                },
                "color_images": {
                    "wrist": "images/wrist",
                    "global": "images/global",
                },
            },
        }
        target = self._session_dir / "metadata.json"
        temporary = self._session_dir / "metadata.json.tmp"
        temporary.write_text(json.dumps(payload, indent=2), encoding="utf-8")
        temporary.replace(target)


def main(args=None):
    rclpy.init(args=args)
    node = DataCollectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
