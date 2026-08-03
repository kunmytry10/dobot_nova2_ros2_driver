import json
import os
import queue
import threading
import time
from collections import deque
from datetime import datetime, timezone
from pathlib import Path

import message_filters
import numpy as np
import rclpy
from cv_bridge import CvBridge
from dobot_interfaces.msg import CartesianServoCommand, DobotState, GripperStatus
from dobot_interfaces.srv import GripperCommand, JogCommand
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger

from dobot_policy.policy_logic import (
    build_state,
    decode_move_jog,
    normalized_gripper_target,
    servo_velocity,
    validate_action_chunk,
)


def _now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


class DobotPolicyNode(Node):
    """Run an OpenPI action policy as a guarded, receding-horizon ROS loop."""

    def __init__(self):
        super().__init__("dobot_policy_node")
        self._declare_parameters()
        self._load_parameters()

        self.bridge = CvBridge()
        self._lock = threading.RLock()
        self._callback_group = ReentrantCallbackGroup()
        self._latest = {}
        self._received = {}
        self._active = False
        self._generation = 0
        self._episode_started = 0.0
        self._actions = deque()
        self._next_actions = None
        self._current_axis = None
        self._desired_axis = None
        self._jog_stop_pending = False
        self._jog_start_pending = False
        self._last_action_time = 0.0
        self._last_servo_velocity = np.zeros(6, dtype=np.float64)
        self._motion_started_at = None
        self._last_gripper_command_time = 0.0
        self._last_gripper_target_mm = None
        self._inference_inflight = False
        self._inference_started = 0.0
        self._inference_request = None
        self._inference_sequence = 0
        self._inference_event = threading.Event()
        self._inference_results = queue.Queue()
        self._shutdown = threading.Event()
        self._policy_client = None
        self._policy_connected = False
        self._trigger_pending = set()
        self._last_setup_request = 0.0
        self._gripper_init_attempted = False

        self._open_log()
        self.diagnostics_pub = self.create_publisher(
            String, "/dobot_policy/diagnostics", 10
        )
        self.servo_pub = self.create_publisher(
            CartesianServoCommand,
            self.servo_command_topic,
            qos_profile_sensor_data,
        )
        self.jog_client = self.create_client(
            JogCommand, "/move_jog", callback_group=self._callback_group
        )
        self.gripper_client = self.create_client(
            GripperCommand, "/gripper_move", callback_group=self._callback_group
        )
        self.enable_client = self.create_client(
            Trigger, "/enable_robot", callback_group=self._callback_group
        )
        self.gripper_init_client = self.create_client(
            Trigger, "/gripper_init", callback_group=self._callback_group
        )

        self.create_subscription(DobotState, "/dobot_state", self._on_robot, 10)
        self.create_subscription(JointState, "/joint_states", self._on_joints, 10)
        self.create_subscription(
            Float64MultiArray, "/tcp_pose", self._on_tcp_pose, 10
        )
        self.create_subscription(
            GripperStatus, "/gripper_state", self._on_gripper, 10
        )
        self._wrist_sub = message_filters.Subscriber(
            self,
            Image,
            self.wrist_image_topic,
            qos_profile=qos_profile_sensor_data,
        )
        self._global_sub = message_filters.Subscriber(
            self,
            Image,
            self.global_image_topic,
            qos_profile=qos_profile_sensor_data,
        )
        self._image_sync = message_filters.ApproximateTimeSynchronizer(
            [self._wrist_sub, self._global_sub],
            queue_size=max(2, self.image_sync_queue_size),
            slop=max(0.0, self.max_image_skew_sec),
        )
        self._image_sync.registerCallback(self._on_images)

        self.create_service(Trigger, "/dobot_policy/start", self._start_service)
        self.create_service(Trigger, "/dobot_policy/stop", self._stop_service)
        self.create_service(Trigger, "/dobot_policy/status", self._status_service)
        self.create_timer(
            1.0 / max(1.0, self.control_rate_hz), self._control_tick
        )
        self.create_timer(0.05, self._drain_inference_results)

        self._inference_thread = threading.Thread(
            target=self._inference_loop,
            name="dobot-policy-inference",
            daemon=True,
        )
        self._inference_thread.start()
        self._event(
            "node_ready",
            armed=self.armed,
            auto_start=self.auto_start,
            policy=f"{self.policy_host}:{self.policy_port}",
            control_mode=self.control_mode,
        )
        if self.auto_start:
            self._activate("auto_start")

    def _declare_parameters(self):
        defaults = {
            "policy_host": "127.0.0.1",
            "policy_port": 8000,
            "prompt": "pick up the tape roll",
            "control_mode": "move_jog",
            "servo_command_topic": "/cartesian_servo/command",
            "disable_policy_proxy": True,
            "wrist_image_topic": "/camera/color/image_raw",
            "global_image_topic": "/global_camera/color/image_raw",
            "image_sync_queue_size": 20,
            "max_image_skew_sec": 0.05,
            "armed": False,
            "auto_start": False,
            "auto_enable_robot": False,
            "auto_init_gripper": False,
            "gripper_enabled": True,
            "motion_test_duration_sec": 3.0,
            "control_rate_hz": 10.0,
            "action_horizon": 16,
            "steps_per_inference": 4,
            "inference_lead_steps": 2,
            "inference_timeout_sec": 15.0,
            "source_timeout_sec": 1.0,
            "startup_timeout_sec": 20.0,
            "max_episode_sec": 45.0,
            "move_threshold": 0.5,
            "minimum_axis_margin": 0.15,
            "allow_rotation": False,
            "coord_type": 0,
            "user": 0,
            "tool": 0,
            "servo_deadband": 0.03,
            "servo_axis_scales": [1.0, 1.0, 1.0, 0.0, 0.0, 1.0],
            "gripper_min_opening_mm": 0.0,
            "gripper_max_opening_mm": 95.0,
            "gripper_open_threshold": 0.5,
            "gripper_force_percent": 50,
            "gripper_command_delta_mm": 5.0,
            "gripper_command_period_sec": 0.5,
            "gripper_timeout_sec": 3.0,
            "log_root": "logs/policy",
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)

    def _load_parameters(self):
        for name in (
            "policy_host",
            "prompt",
            "control_mode",
            "servo_command_topic",
            "wrist_image_topic",
            "global_image_topic",
            "log_root",
        ):
            setattr(self, name, str(self.get_parameter(name).value))
        for name in (
            "policy_port",
            "image_sync_queue_size",
            "action_horizon",
            "steps_per_inference",
            "inference_lead_steps",
            "coord_type",
            "user",
            "tool",
            "gripper_force_percent",
        ):
            setattr(self, name, int(self.get_parameter(name).value))
        for name in (
            "armed",
            "auto_start",
            "auto_enable_robot",
            "auto_init_gripper",
            "gripper_enabled",
            "allow_rotation",
            "disable_policy_proxy",
        ):
            setattr(self, name, bool(self.get_parameter(name).value))
        for name in (
            "max_image_skew_sec",
            "control_rate_hz",
            "inference_timeout_sec",
            "source_timeout_sec",
            "startup_timeout_sec",
            "max_episode_sec",
            "motion_test_duration_sec",
            "move_threshold",
            "minimum_axis_margin",
            "servo_deadband",
            "gripper_min_opening_mm",
            "gripper_max_opening_mm",
            "gripper_open_threshold",
            "gripper_command_delta_mm",
            "gripper_command_period_sec",
            "gripper_timeout_sec",
        ):
            setattr(self, name, float(self.get_parameter(name).value))
        self.servo_axis_scales = list(
            self.get_parameter("servo_axis_scales").value
        )
        if self.control_mode not in {"move_jog", "servo_p"}:
            raise ValueError("control_mode must be move_jog or servo_p")
        if self.control_mode == "servo_p" and self.coord_type != 0:
            raise ValueError("servo_p control requires coord_type=0")
        if len(self.servo_axis_scales) != 6:
            raise ValueError("servo_axis_scales must contain six values")
        self.steps_per_inference = max(
            1, min(self.action_horizon, self.steps_per_inference)
        )
        self.inference_lead_steps = max(
            0, min(self.steps_per_inference - 1, self.inference_lead_steps)
        )

    def _open_log(self):
        root = Path(self.log_root).expanduser().resolve()
        root.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self._log_path = root / f"pi05_tape_grasp_{stamp}.jsonl"
        self._human_log_path = root / f"pi05_tape_grasp_{stamp}.log"
        self._artifact_dir = root / f"pi05_tape_grasp_{stamp}_artifacts"
        self._artifact_dir.mkdir(parents=True, exist_ok=True)
        self._log_file = self._log_path.open("a", encoding="utf-8")
        self._human_log_file = self._human_log_path.open("a", encoding="utf-8")
        self._log_lock = threading.Lock()

    @staticmethod
    def _format_human_event(event, fields):
        """Keep a compact operator log beside the lossless JSONL record."""
        stamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        level = "ERROR" if event.endswith(("failed", "rejected")) else "INFO"
        if event == "episode_finished" and not fields.get("success", False):
            level = "ERROR"
        important = []
        for key in (
            "reason", "error", "message", "request_id", "generation", "elapsed_ms",
            "queue_len", "target_mm", "axis", "success",
        ):
            if key in fields:
                value = fields[key]
                if isinstance(value, float):
                    value = f"{value:.3f}"
                important.append(f"{key}={value}")
        suffix = " " + " ".join(important) if important else ""
        return f"{stamp} {level:<5} {event:<28}{suffix}"

    def _remember(self, name, value):
        with self._lock:
            self._latest[name] = value
            self._received[name] = time.monotonic()

    def _on_robot(self, message):
        self._remember("robot", message)

    def _on_joints(self, message):
        self._remember("joints", list(message.position[:6]))

    def _on_tcp_pose(self, message):
        self._remember("tcp", list(message.data[:6]))

    def _on_gripper(self, message):
        self._remember("gripper", message)

    def _on_images(self, wrist_message, global_message):
        try:
            wrist = np.ascontiguousarray(
                self.bridge.imgmsg_to_cv2(wrist_message, desired_encoding="rgb8")
            )
            global_image = np.ascontiguousarray(
                self.bridge.imgmsg_to_cv2(global_message, desired_encoding="rgb8")
            )
        except Exception as exc:
            self._event("image_conversion_failed", error=str(exc))
            return
        self._remember("images", (wrist, global_image))
        with self._lock:
            should_request = self._active and not self._actions
        if should_request:
            self._queue_inference()

    def _start_service(self, request, response):
        del request
        response.success, response.message = self._activate("service")
        return response

    def _stop_service(self, request, response):
        del request
        self._finish(False, "stopped by service")
        response.success = True
        response.message = "policy stopped and motion stop requested"
        return response

    def _status_service(self, request, response):
        del request
        with self._lock:
            status = {
                "active": self._active,
                "armed": self.armed,
                "control_mode": self.control_mode,
                "gripper_enabled": self.gripper_enabled,
                "policy_connected": self._policy_connected,
                "queued_actions": len(self._actions),
                "next_chunk_ready": self._next_actions is not None,
                "current_axis": self._current_axis,
                "servo_velocity": self._last_servo_velocity.round(6).tolist(),
            "inference_inflight": self._inference_inflight,
            "log": str(self._log_path),
            "human_log": str(self._human_log_path),
            "artifacts": str(self._artifact_dir),
            }
        response.success = True
        response.message = json.dumps(status, ensure_ascii=True)
        return response

    def _activate(self, source):
        with self._lock:
            if self._active:
                return False, "policy is already active"
            self._generation += 1
            self._active = True
            self._episode_started = time.monotonic()
            self._actions.clear()
            self._next_actions = None
            self._motion_started_at = None
            self._last_gripper_target_mm = None
            self._gripper_init_attempted = False
        if self.armed and self.auto_enable_robot:
            self._call_trigger(self.enable_client, "enable_robot")
        if self.armed and self.gripper_enabled and self.auto_init_gripper:
            self._call_trigger(self.gripper_init_client, "gripper_init")
        self._event(
            "episode_started",
            source=source,
            armed=self.armed,
            control_mode=self.control_mode,
        )
        self._queue_inference()
        mode = "ARMED" if self.armed else "DRY-RUN"
        return True, f"policy started in {mode} mode"

    def _call_trigger(self, client, name):
        if name in self._trigger_pending or not client.service_is_ready():
            return False
        self._trigger_pending.add(name)
        future = client.call_async(Trigger.Request())
        future.add_done_callback(
            lambda result, service=name: self._trigger_done(result, service)
        )
        return True

    def _trigger_done(self, future, service):
        self._trigger_pending.discard(service)
        try:
            result = future.result()
            self._event(
                "trigger_result",
                service=service,
                success=bool(result.success),
                message=result.message,
            )
        except Exception as exc:
            self._event("trigger_failed", service=service, error=str(exc))

    def _observation_snapshot(self):
        with self._lock:
            required = ("joints", "tcp", "gripper", "images")
            if any(name not in self._latest for name in required):
                return None
            wrist, global_image = self._latest["images"]
            state = build_state(
                self._latest["joints"],
                self._latest["tcp"],
                self._latest["gripper"].opening_mm,
            )
            return {
                "state": state,
                "wrist_image": wrist.copy(),
                "global_image": global_image.copy(),
                "prompt": self.prompt,
            }

    def _queue_inference(self):
        with self._lock:
            if (
                not self._active
                or self._inference_inflight
                or self._next_actions is not None
            ):
                return False
            observation = self._observation_snapshot()
            if observation is None:
                return False
            self._inference_inflight = True
            self._inference_started = time.monotonic()
            self._inference_sequence += 1
            request_id = self._inference_sequence
            self._inference_request = (
                self._generation,
                request_id,
                observation,
            )
            self._event(
                "inference_requested",
                request_id=request_id,
                state=observation["state"].round(6).tolist(),
                global_image_shape=list(observation["global_image"].shape),
                wrist_image_shape=list(observation["wrist_image"].shape),
                source_age_ms={
                    name: round(
                        (self._inference_started - stamp) * 1000.0, 3
                    )
                    for name, stamp in self._received.items()
                    if name in {"robot", "joints", "tcp", "gripper", "images"}
                },
            )
            self._inference_event.set()
            return True

    def _inference_loop(self):
        while not self._shutdown.is_set():
            self._inference_event.wait(timeout=0.2)
            if self._shutdown.is_set():
                return
            with self._lock:
                request = self._inference_request
                self._inference_request = None
                self._inference_event.clear()
            if request is None:
                continue
            generation, request_id, observation = request
            started = time.monotonic()
            artifact_error = None
            try:
                self._save_observation_artifacts(request_id, observation)
            except Exception as exc:
                artifact_error = str(exc)
            try:
                if self._policy_client is None:
                    if self.disable_policy_proxy:
                        for name in (
                            "HTTP_PROXY",
                            "HTTPS_PROXY",
                            "ALL_PROXY",
                            "http_proxy",
                            "https_proxy",
                            "all_proxy",
                        ):
                            os.environ.pop(name, None)
                        os.environ["NO_PROXY"] = "localhost,127.0.0.1,::1"
                        os.environ["no_proxy"] = "localhost,127.0.0.1,::1"
                    from openpi_client import websocket_client_policy

                    self._policy_client = (
                        websocket_client_policy.WebsocketClientPolicy(
                            self.policy_host, self.policy_port
                        )
                    )
                    with self._lock:
                        self._policy_connected = True
                result = self._policy_client.infer(observation)
                actions = validate_action_chunk(
                    result["actions"], self.action_horizon
                )
                actions_path = self._artifact_dir / f"request_{request_id:04d}_actions.npy"
                np.save(actions_path, actions)
                self._inference_results.put(
                    (
                        generation,
                        request_id,
                        actions,
                        time.monotonic() - started,
                        artifact_error,
                        None,
                    )
                )
            except Exception as exc:
                self._policy_client = None
                with self._lock:
                    self._policy_connected = False
                self._inference_results.put(
                    (
                        generation,
                        request_id,
                        None,
                        time.monotonic() - started,
                        artifact_error,
                        str(exc),
                    )
                )

    def _save_observation_artifacts(self, request_id, observation):
        from PIL import Image as PilImage

        prefix = self._artifact_dir / f"request_{request_id:04d}"
        PilImage.fromarray(observation["global_image"]).save(
            f"{prefix}_global.jpg", quality=90
        )
        PilImage.fromarray(observation["wrist_image"]).save(
            f"{prefix}_wrist.jpg", quality=90
        )
        np.save(f"{prefix}_state.npy", observation["state"])

    def _drain_inference_results(self):
        while True:
            try:
                generation, request_id, actions, elapsed, artifact_error, error = (
                    self._inference_results.get_nowait()
                )
            except queue.Empty:
                return
            with self._lock:
                if generation != self._generation or not self._active:
                    continue
                self._inference_inflight = False
                if error is not None:
                    self._event(
                        "inference_failed",
                        request_id=request_id,
                        elapsed_ms=round(elapsed * 1000.0, 3),
                        artifact_error=artifact_error,
                        error=error,
                    )
                    self._fail(f"policy inference failed: {error}")
                    continue
                selected = [
                    row.copy() for row in actions[: self.steps_per_inference]
                ]
                if self._actions:
                    self._next_actions = selected
                else:
                    self._actions.extend(selected)
                self._event(
                    "inference_complete",
                    request_id=request_id,
                    elapsed_ms=round(elapsed * 1000.0, 3),
                    action_min=float(actions.min()),
                    action_max=float(actions.max()),
                    actions_file=str(
                        self._artifact_dir / f"request_{request_id:04d}_actions.npy"
                    ),
                    artifact_error=artifact_error,
                )

    def _control_tick(self):
        self._drain_inference_results()
        with self._lock:
            if not self._active:
                return
            now = time.monotonic()
            if now - self._episode_started > self.max_episode_sec:
                self._fail("episode timeout")
                return
            if (
                self._inference_inflight
                and now - self._inference_started > self.inference_timeout_sec
            ):
                self._fail("policy inference timeout")
                return
            self._prepare_hardware(now)
            reason = self._safety_reason(now)
            if reason:
                if now - self._episode_started < self.startup_timeout_sec:
                    self._stop_motion()
                    return
                self._fail(reason)
                return
            if (
                not self.gripper_enabled
                and self._motion_started_at is not None
                and now - self._motion_started_at >= self.motion_test_duration_sec
            ):
                self._finish(True, "100% ServoP motion-only validation completed")
                return
            if not self._actions and self._next_actions is not None:
                self._actions.extend(self._next_actions)
                self._next_actions = None
            if not self._actions:
                self._stop_motion()
                self._queue_inference()
                return
            action = self._actions.popleft()
            if self.control_mode == "move_jog":
                decision = decode_move_jog(
                    action,
                    self.move_threshold,
                    self.minimum_axis_margin,
                    self.allow_rotation,
                )
                velocity = np.zeros(6, dtype=np.float64)
            else:
                decision = None
                velocity = servo_velocity(
                    action,
                    self.servo_deadband,
                    self.servo_axis_scales,
                )
            gripper_target = normalized_gripper_target(
                action,
                self.gripper_min_opening_mm,
                self.gripper_max_opening_mm,
                self.gripper_open_threshold,
            )
            self._last_action_time = now
            self._event(
                "action_step",
                raw=np.asarray(action).round(6).tolist(),
                control_mode=self.control_mode,
                axis=decision.axis_id if decision is not None else None,
                decision=decision.reason if decision is not None else "servo_p",
                magnitude=round(
                    decision.magnitude
                    if decision is not None
                    else float(np.max(np.abs(velocity))),
                    6,
                ),
                servo_velocity=velocity.round(6).tolist(),
                gripper_target_mm=round(gripper_target, 3),
                gripper_command=(
                    "open"
                    if gripper_target >= self.gripper_max_opening_mm - 1e-6
                    else "close"
                ),
                queued=len(self._actions),
                joints_rad=[round(value, 6) for value in self._latest["joints"]],
                tcp_pose_mm_deg=[
                    round(value, 6) for value in self._latest["tcp"]
                ],
                gripper_opening_mm=round(
                    float(self._latest["gripper"].opening_mm), 3
                ),
                robot_mode=int(self._latest["robot"].robot_mode),
                robot_speed_scaling=round(
                    float(self._latest["robot"].speed_scaling), 3
                ),
            )
            if self.armed:
                if self.control_mode == "move_jog":
                    self._set_jog_axis(decision.axis_id)
                else:
                    self._set_servo_velocity(velocity)
                if self.gripper_enabled:
                    self._set_gripper_target(gripper_target, now)
            if len(self._actions) <= self.inference_lead_steps:
                self._queue_inference()

    def _prepare_hardware(self, now):
        if not self.armed or now - self._last_setup_request < 2.0:
            return
        robot = self._latest.get("robot")
        gripper = self._latest.get("gripper")
        requested = False
        if (
            self.auto_enable_robot
            and (robot is None or robot.enable_status != 1)
        ):
            requested = self._call_trigger(self.enable_client, "enable_robot")
        if (
            self.gripper_enabled
            and self.auto_init_gripper
            and not self._gripper_init_attempted
            and (gripper is None or not gripper.initialized)
        ):
            requested_init = self._call_trigger(
                self.gripper_init_client, "gripper_init"
            )
            self._gripper_init_attempted = requested_init
            requested = requested_init or requested
        if requested:
            self._last_setup_request = now

    def _safety_reason(self, now):
        required = ("robot", "joints", "tcp", "gripper", "images")
        missing = [name for name in required if name not in self._latest]
        if missing:
            return "waiting for sources: " + ", ".join(missing)
        stale = [
            name
            for name in required
            if now - self._received.get(name, 0.0) > self.source_timeout_sec
        ]
        if stale:
            return "stale sources: " + ", ".join(stale)
        robot = self._latest["robot"]
        if not robot.connected or not robot.feedback_valid:
            return "robot feedback is unavailable"
        if robot.error_status or robot.robot_mode == 9:
            return "robot is in error state"
        if self.armed and robot.enable_status != 1:
            return "robot is not enabled"
        gripper = self._latest["gripper"]
        if self.armed and self.gripper_enabled and (
            not gripper.success
            or not gripper.connected
            or not gripper.initialized
        ):
            return "gripper state is unavailable or not initialized"
        return ""

    def _stop_motion(self):
        if not self.armed:
            return
        if self.control_mode == "servo_p":
            self._set_servo_velocity(np.zeros(6), active=False)
        else:
            self._set_jog_axis(None)

    def _set_servo_velocity(self, velocity, active=True):
        if not self.armed:
            return
        values = np.asarray(velocity, dtype=np.float64)
        if values.shape != (6,) or not np.isfinite(values).all():
            self._fail("invalid ServoP velocity")
            return
        values = np.clip(values, -1.0, 1.0)
        message = CartesianServoCommand()
        message.stamp = self.get_clock().now().to_msg()
        message.normalized_velocity = values.tolist()
        message.active = bool(active)
        message.deadman = bool(active)
        message.coord_type = self.coord_type
        message.user = self.user
        message.tool = self.tool
        message.status = "policy" if active else "policy stop"
        try:
            self.servo_pub.publish(message)
            self._last_servo_velocity = values if active else np.zeros(6)
            if (
                active
                and self._motion_started_at is None
                and np.any(np.abs(values) > 1e-6)
            ):
                self._motion_started_at = time.monotonic()
                self._event(
                    "motion_test_started",
                    duration_sec=self.motion_test_duration_sec,
                    servo_velocity=values.round(6).tolist(),
                )
        except Exception as exc:
            if active:
                self._fail(f"ServoP publish failed: {exc}")
            else:
                self.get_logger().warning(f"ServoP stop publish failed: {exc}")

    def _set_jog_axis(self, axis_id):
        if not self.armed:
            return
        self._desired_axis = axis_id
        if self._jog_stop_pending:
            return
        if self._current_axis == axis_id:
            if axis_id is not None:
                self._send_jog_start(axis_id)
            return
        if self._current_axis is not None:
            self._send_jog_stop()
            return
        if axis_id is not None:
            self._send_jog_start(axis_id)

    def _send_jog_start(self, axis_id):
        if self._jog_start_pending:
            return
        if not self.jog_client.service_is_ready():
            self._fail("/move_jog service is unavailable")
            return
        request = JogCommand.Request()
        request.axis_id = str(axis_id)
        request.stop = False
        request.coord_type = self.coord_type
        request.user = self.user
        request.tool = self.tool
        self._current_axis = axis_id
        self._jog_start_pending = True
        future = self.jog_client.call_async(request)
        future.add_done_callback(
            lambda result, axis=axis_id: self._jog_start_done(result, axis)
        )

    def _jog_start_done(self, future, axis_id):
        self._jog_start_pending = False
        try:
            result = future.result()
            self._event(
                "jog_start_result",
                axis=axis_id,
                success=bool(result.success),
                error_id=int(result.error_id),
                message=result.message,
                raw_reply=result.raw_reply,
            )
            if not result.success:
                self._fail(f"move_jog {axis_id} rejected: {result.message}")
        except Exception as exc:
            self._fail(f"move_jog {axis_id} failed: {exc}")

    def _send_jog_stop(self):
        # Shutdown may invalidate the ROS context before destroy_node runs.
        # A watchdog in the driver also stops MoveJog when heartbeats cease.
        if not rclpy.ok(context=self.context):
            self._current_axis = None
            self._jog_stop_pending = False
            return
        try:
            service_ready = self.jog_client.service_is_ready()
        except Exception:
            self._current_axis = None
            self._jog_stop_pending = False
            return
        if not service_ready:
            self._current_axis = None
            self._jog_stop_pending = False
            return
        request = JogCommand.Request()
        request.stop = True
        self._current_axis = None
        self._jog_stop_pending = True
        try:
            future = self.jog_client.call_async(request)
        except Exception:
            self._jog_stop_pending = False
            return
        future.add_done_callback(self._jog_stop_done)

    def _jog_stop_done(self, future):
        try:
            result = future.result()
            self._event(
                "jog_stop_result",
                success=bool(result.success),
                error_id=int(result.error_id),
                message=result.message,
                raw_reply=result.raw_reply,
            )
            if not result.success:
                self._event("jog_stop_rejected", message=result.message)
        except Exception as exc:
            self._event("jog_stop_failed", error=str(exc))
        with self._lock:
            self._jog_stop_pending = False
            desired = self._desired_axis if self._active else None
            if desired is not None:
                self._send_jog_start(desired)

    def _set_gripper_target(self, target_mm, now):
        if (
            self._last_gripper_target_mm is not None
            and abs(target_mm - self._last_gripper_target_mm)
            < self.gripper_command_delta_mm
        ):
            return
        if now - self._last_gripper_command_time < self.gripper_command_period_sec:
            return
        if not self.gripper_client.service_is_ready():
            self._fail("/gripper_move service is unavailable")
            return
        request = GripperCommand.Request()
        request.opening_mm = float(target_mm)
        request.position_permille = -1
        request.force_percent = self.gripper_force_percent
        request.force_n = -1.0
        request.wait = False
        request.timeout_sec = self.gripper_timeout_sec
        self._last_gripper_target_mm = float(target_mm)
        self._last_gripper_command_time = now
        future = self.gripper_client.call_async(request)
        future.add_done_callback(self._gripper_done)

    def _gripper_done(self, future):
        try:
            result = future.result()
            self._event(
                "gripper_result",
                success=bool(result.success),
                message=result.message,
                opening_mm=round(float(result.opening_mm), 3),
                position_permille=int(result.position_permille),
                object_detected=bool(result.object_detected),
            )
            if not result.success:
                self._fail(f"gripper command rejected: {result.message}")
        except Exception as exc:
            self._fail(f"gripper command failed: {exc}")

    def _fail(self, reason):
        self._finish(False, reason)

    def _finish(self, success, reason):
        with self._lock:
            was_active = self._active
            self._active = False
            self._generation += 1
            self._actions.clear()
            self._next_actions = None
            self._inference_inflight = False
            self._desired_axis = None
        if self.armed and was_active:
            self._stop_motion()
        if was_active:
            self._event("episode_finished", success=bool(success), reason=reason)
            if success:
                label = (
                    "POLICY GRASP SUCCEEDED"
                    if self.gripper_enabled
                    else "POLICY MOTION VALIDATION SUCCEEDED"
                )
                self.get_logger().info(f"{label}: {reason}")
            else:
                self.get_logger().error(f"policy stopped: {reason}")

    def _event(self, event, **fields):
        record = {"time": _now_iso(), "event": event, **fields}
        line = json.dumps(record, ensure_ascii=True, separators=(",", ":"))
        human_line = self._format_human_event(event, fields)
        with self._log_lock:
            self._log_file.write(line + "\n")
            self._log_file.flush()
            self._human_log_file.write(human_line + "\n")
            self._human_log_file.flush()
        message = String()
        message.data = line
        if hasattr(self, "diagnostics_pub"):
            self.diagnostics_pub.publish(message)
        operator_events = {
            "node_ready",
            "episode_started",
            "episode_finished",
            "inference_failed",
            "gripper_state_unavailable",
            "source_timeout",
            "watchdog_stop",
        }
        if event in operator_events or event.endswith(("failed", "rejected")):
            if event == "episode_finished" and not fields.get("success", False):
                self.get_logger().error(human_line)
            elif event.endswith(("failed", "rejected")):
                self.get_logger().warning(human_line)
            else:
                self.get_logger().info(human_line)

    def destroy_node(self):
        self._finish(False, "node shutdown")
        self._shutdown.set()
        self._inference_event.set()
        self._inference_thread.join(timeout=1.0)
        with self._log_lock:
            self._log_file.close()
            self._human_log_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DobotPolicyNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception:
        # ros2 launch may invalidate the context before executor.spin returns.
        # Treat that normal shutdown path like Ctrl-C, but preserve real errors.
        if rclpy.ok():
            raise
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
