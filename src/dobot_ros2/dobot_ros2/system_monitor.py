import json
import os
import platform
import re
import socket
import subprocess
import time
from collections import deque
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from dobot_interfaces.msg import DobotState
from rcl_interfaces.msg import Log
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Image, Joy
from std_msgs.msg import String
from std_srvs.srv import Trigger


def _utc_now():
    return datetime.now(timezone.utc).isoformat()


class SystemMonitor(Node):
    """Report system readiness and persist one structured log per launch."""

    def __init__(self):
        super().__init__("dobot_system_monitor")
        self.log_dir = Path(str(self.declare_parameter("log_dir", "").value))
        self.robot_mode = str(
            self.declare_parameter("robot_mode", "bringup").value
        )
        self.start_camera = bool(
            self.declare_parameter("start_camera", True).value
        )
        self.start_joy = bool(self.declare_parameter("start_joy", True).value)
        self.start_data_collection = bool(
            self.declare_parameter("start_data_collection", True).value
        )
        self.init_gripper = bool(
            self.declare_parameter("init_gripper", True).value
        )
        self.task_file = Path(
            str(self.declare_parameter("task_file", "").value)
        )
        self.dataset_root = str(
            self.declare_parameter("dataset_root", "").value
        )
        self.lerobot_dataset_root = str(
            self.declare_parameter("lerobot_dataset_root", "").value
        )
        self.lerobot_repo_id = str(
            self.declare_parameter("lerobot_repo_id", "").value
        )
        self.data_reject_hold_sec = float(
            self.declare_parameter("data_reject_hold_sec", 2.0).value
        )
        self.wrist_image_topic = str(
            self.declare_parameter(
                "wrist_image_topic", "/camera/color/image_raw"
            ).value
        )
        self.global_image_topic = str(
            self.declare_parameter(
                "global_image_topic", "/global_camera/color/image_raw"
            ).value
        )
        self.diagnostics_topic = str(
            self.declare_parameter(
                "diagnostics_topic", "/joy/teleop_diagnostics"
            ).value
        )
        self.health_period_sec = max(
            0.2,
            float(self.declare_parameter("health_period_sec", 1.0).value),
        )

        self.log_dir.mkdir(parents=True, exist_ok=True)
        self._node_log_dir = self.log_dir / "nodes"
        self._node_log_dir.mkdir(exist_ok=True)
        self._node_log_streams = {}
        self._rosout_file = (self.log_dir / "nodes.jsonl").open(
            "a", encoding="utf-8"
        )
        self._events_file = (self.log_dir / "events.jsonl").open(
            "a", encoding="utf-8"
        )
        self._health_file = (self.log_dir / "health.jsonl").open(
            "a", encoding="utf-8"
        )
        self._started_monotonic = time.monotonic()
        self._last_received = {}
        self._camera_times = {
            "wrist": deque(maxlen=60),
            "global": deque(maxlen=60),
        }
        self._robot = None
        self._ready = False
        self._gripper_init_started = False
        self._gripper_init_complete = not self.init_gripper
        self._gripper_init_error = ""

        self.gripper_init_client = self.create_client(Trigger, "/gripper_init")
        self.data_status_client = self.create_client(
            Trigger, "/data_collection/status"
        )
        self.create_subscription(DobotState, "/dobot_state", self._on_robot, 10)
        self.create_subscription(Joy, "/joy", self._on_joy, qos_profile_sensor_data)
        self.create_subscription(
            String, self.diagnostics_topic, self._on_teleop_diagnostic, 50
        )
        rosout_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1000,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(Log, "/rosout", self._on_rosout, rosout_qos)
        self.create_subscription(
            Image,
            self.wrist_image_topic,
            lambda msg: self._on_image("wrist", msg),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            self.global_image_topic,
            lambda msg: self._on_image("global", msg),
            qos_profile_sensor_data,
        )
        self.create_timer(self.health_period_sec, self._check_system)
        self._write_manifest()
        self._event("system_start", {"robot_mode": self.robot_mode})
        self.get_logger().info(f"system run log: {self.log_dir}")

    def destroy_node(self):
        self._event("system_stop", {"ready": self._ready})
        self._rosout_file.close()
        for stream in self._node_log_streams.values():
            stream.close()
        self._events_file.close()
        self._health_file.close()
        super().destroy_node()

    def _write_manifest(self):
        task = ""
        if self.task_file.is_file():
            try:
                task = self.task_file.read_text(encoding="utf-8").strip()
            except OSError:
                pass
        git_commit = ""
        git_dirty = None
        try:
            git_commit = subprocess.run(
                ["git", "rev-parse", "HEAD"],
                capture_output=True,
                check=False,
                text=True,
                timeout=2.0,
            ).stdout.strip()
            git_dirty = bool(
                subprocess.run(
                    ["git", "status", "--porcelain"],
                    capture_output=True,
                    check=False,
                    text=True,
                    timeout=2.0,
                ).stdout.strip()
            )
        except (OSError, subprocess.TimeoutExpired):
            pass
        payload = {
            "started_utc": _utc_now(),
            "hostname": socket.gethostname(),
            "platform": platform.platform(),
            "pid": os.getpid(),
            "ros_distro": os.environ.get("ROS_DISTRO", ""),
            "git_commit": git_commit,
            "git_dirty": git_dirty,
            "robot_mode": self.robot_mode,
            "start_camera": self.start_camera,
            "start_joy": self.start_joy,
            "start_data_collection": self.start_data_collection,
            "init_gripper": self.init_gripper,
            "task_file": str(self.task_file),
            "task_instruction": task,
            "dataset_root": self.dataset_root,
            "lerobot_dataset_root": self.lerobot_dataset_root,
            "lerobot_repo_id": self.lerobot_repo_id,
            "data_reject_hold_sec": self.data_reject_hold_sec,
            "topics": {
                "wrist_image": self.wrist_image_topic,
                "global_image": self.global_image_topic,
                "joy": "/joy",
                "teleop_diagnostics": self.diagnostics_topic,
                "robot_state": "/dobot_state",
            },
        }
        (self.log_dir / "manifest.json").write_text(
            json.dumps(payload, indent=2), encoding="utf-8"
        )

    def _event(self, name, details=None):
        payload = {"utc": _utc_now(), "event": name}
        if details:
            payload.update(details)
        self._events_file.write(json.dumps(payload, separators=(",", ":")) + "\n")
        self._events_file.flush()

    def _on_robot(self, msg):
        now = time.monotonic()
        previous = self._robot
        self._last_received["robot"] = now
        self._robot = {
            "connected": bool(msg.connected),
            "feedback_valid": bool(msg.feedback_valid),
            "enable_status": int(msg.enable_status),
            "error_status": int(msg.error_status),
            "robot_mode": int(msg.robot_mode),
            "robot_mode_text": str(msg.robot_mode_text),
        }
        if previous != self._robot:
            self._event("robot_state_changed", self._robot)

    def _on_joy(self, msg):
        del msg
        if "joy" not in self._last_received:
            self._event("joy_connected")
        self._last_received["joy"] = time.monotonic()

    def _on_image(self, camera, msg):
        del msg
        now = time.monotonic()
        if camera not in self._last_received:
            self._event("camera_connected", {"camera": camera})
        self._last_received[camera] = now
        self._camera_times[camera].append(now)

    def _on_teleop_diagnostic(self, msg):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            payload = {"message": msg.data}
        self._event("teleop_diagnostic", {"payload": payload})

    def _on_rosout(self, msg):
        stamp_sec = float(msg.stamp.sec) + float(msg.stamp.nanosec) * 1e-9
        utc = datetime.fromtimestamp(stamp_sec, timezone.utc).isoformat()
        level = {
            10: "DEBUG",
            20: "INFO",
            30: "WARN",
            40: "ERROR",
            50: "FATAL",
        }.get(int(msg.level), str(int(msg.level)))
        name = str(msg.name or "unknown")
        payload = {
            "utc": utc,
            "level": level,
            "node": name,
            "message": str(msg.msg),
            "source": {
                "file": str(msg.file),
                "function": str(msg.function),
                "line": int(msg.line),
            },
        }
        self._rosout_file.write(json.dumps(payload, separators=(",", ":")) + "\n")
        self._rosout_file.flush()
        safe_name = re.sub(r"[^A-Za-z0-9_.-]+", "_", name).strip("_") or "unknown"
        stream = self._node_log_streams.get(safe_name)
        if stream is None:
            stream = (self._node_log_dir / f"{safe_name}.log").open(
                "a", encoding="utf-8"
            )
            self._node_log_streams[safe_name] = stream
        source = f"{msg.file}:{msg.line} {msg.function}" if msg.file else ""
        stream.write(
            f"{utc} [{level}] [{name}]"
            f"{f' [{source}]' if source else ''} {msg.msg}\n"
        )
        stream.flush()

    def _rate(self, camera):
        stamps = self._camera_times[camera]
        if len(stamps) < 2 or stamps[-1] <= stamps[0]:
            return 0.0
        return (len(stamps) - 1) / (stamps[-1] - stamps[0])

    def _age(self, name, now):
        stamp = self._last_received.get(name)
        return None if stamp is None else round(now - stamp, 3)

    def _start_gripper_init(self):
        if self._gripper_init_started or self._gripper_init_complete:
            return
        if self._robot is None or not self._robot["connected"]:
            return
        if not self.gripper_init_client.service_is_ready():
            return
        self._gripper_init_started = True
        self._event("gripper_init_requested")
        future = self.gripper_init_client.call_async(Trigger.Request())
        future.add_done_callback(self._on_gripper_init)

    def _on_gripper_init(self, future):
        try:
            response = future.result()
            self._gripper_init_complete = bool(response.success)
            self._gripper_init_error = "" if response.success else response.message
            self._event(
                "gripper_init_finished",
                {"success": bool(response.success), "message": response.message},
            )
        except Exception as exc:  # pragma: no cover - ROS callback safety
            self._gripper_init_error = str(exc)
            self._event("gripper_init_failed", {"error": str(exc)})

    def _check_system(self):
        now = time.monotonic()
        self._start_gripper_init()
        robot_ready = bool(
            self._robot
            and self._robot["connected"]
            and self._robot["feedback_valid"]
            and self._age("robot", now) < 1.0
        )
        cameras_ready = not self.start_camera or all(
            self._age(name, now) is not None and self._age(name, now) < 1.0
            for name in ("wrist", "global")
        )
        joy_ready = not self.start_joy or (
            self._age("joy", now) is not None and self._age("joy", now) < 1.0
        )
        data_ready = (
            not self.start_data_collection or self.data_status_client.service_is_ready()
        )
        ready = all(
            (robot_ready, cameras_ready, joy_ready, data_ready, self._gripper_init_complete)
        )
        payload = {
            "utc": _utc_now(),
            "uptime_sec": round(now - self._started_monotonic, 3),
            "ready": ready,
            "robot_ready": robot_ready,
            "cameras_ready": cameras_ready,
            "joy_ready": joy_ready,
            "data_collection_ready": data_ready,
            "gripper_init_complete": self._gripper_init_complete,
            "gripper_init_error": self._gripper_init_error,
            "source_age_sec": {
                name: self._age(name, now)
                for name in ("robot", "wrist", "global", "joy")
            },
            "camera_rate_hz": {
                name: round(self._rate(name), 3) for name in ("wrist", "global")
            },
            "robot": self._robot,
            "nodes": sorted(
                f"{namespace.rstrip('/')}/{name}".replace("//", "/")
                for name, namespace in self.get_node_names_and_namespaces()
            ),
        }
        self._health_file.write(json.dumps(payload, separators=(",", ":")) + "\n")
        self._health_file.flush()
        if ready and not self._ready:
            self._ready = True
            self._event("system_ready", payload)
            self.get_logger().info("SYSTEM READY: joystick control and collection available")
        elif not ready and self._ready:
            self._ready = False
            self._event("system_not_ready", payload)
            self.get_logger().error("SYSTEM NOT READY: inspect health.jsonl")


def main(args=None):
    rclpy.init(args=args)
    node = SystemMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except KeyboardInterrupt:
                pass


if __name__ == "__main__":
    main()
