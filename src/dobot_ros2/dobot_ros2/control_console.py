import json
import mimetypes
import threading
from array import array
from functools import partial
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from posixpath import normpath
from typing import Any, Dict, Optional
from urllib.parse import unquote, urlparse

import rclpy
from dobot_interfaces.msg import DobotState, GripperStatus
from dobot_interfaces.srv import (
    GetJointState,
    GetRobotState,
    GetTcpPose,
    GripperCommand,
    GripperState,
    MoveCommand,
    TrajectoryCommand,
    TrajectoryList,
)
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger


class DobotControlConsole(Node):
    """HTTP control console backed by the existing Dobot ROS services."""

    def __init__(self):
        super().__init__("dobot_control_console")
        self.host = str(self.declare_parameter("console_host", "0.0.0.0").value)
        self.port = int(self.declare_parameter("console_port", 8080).value)
        self.service_timeout_sec = float(
            self.declare_parameter("console_service_timeout_sec", 300.0).value
        )
        self.static_dir = self._resolve_static_dir(
            str(self.declare_parameter("console_static_dir", "").value)
        )

        self._latest_lock = threading.Lock()
        self._latest_joint_state: Dict[str, Any] = {}
        self._latest_tcp_pose = []
        self._latest_dobot_state: Dict[str, Any] = {}
        self._latest_gripper_state: Dict[str, Any] = {}

        self._trigger_clients = {
            "clear_error": self.create_client(Trigger, "clear_error"),
            "enable_robot": self.create_client(Trigger, "enable_robot"),
            "disable_robot": self.create_client(Trigger, "disable_robot"),
            "emergency_stop": self.create_client(Trigger, "emergency_stop"),
            "get_error_id": self.create_client(Trigger, "get_error_id"),
            "teach_status": self.create_client(Trigger, "teach_status"),
            "gripper_init": self.create_client(Trigger, "gripper_init"),
        }
        self._move_clients = {
            "movej": self.create_client(MoveCommand, "movej"),
            "movejp": self.create_client(MoveCommand, "movejp"),
            "movel": self.create_client(MoveCommand, "movel"),
            "movep": self.create_client(MoveCommand, "movep"),
        }
        self._teach_clients = {
            "teach_start": self.create_client(TrajectoryCommand, "teach_start"),
            "teach_stop": self.create_client(TrajectoryCommand, "teach_stop"),
            "teach_replay": self.create_client(TrajectoryCommand, "teach_replay"),
            "teach_delete": self.create_client(TrajectoryCommand, "teach_delete"),
        }
        self._teach_list_client = self.create_client(TrajectoryList, "teach_list")
        self._gripper_move_client = self.create_client(GripperCommand, "gripper_move")
        self._gripper_state_client = self.create_client(GripperState, "get_gripper_state")
        self._robot_state_client = self.create_client(GetRobotState, "get_robot_state")
        self._joint_state_client = self.create_client(GetJointState, "get_joint_state")
        self._tcp_pose_client = self.create_client(GetTcpPose, "get_tcp_pose")

        self.create_subscription(JointState, "joint_states", self._on_joint_state, 10)
        self.create_subscription(Float64MultiArray, "tcp_pose", self._on_tcp_pose, 10)
        self.create_subscription(DobotState, "dobot_state", self._on_dobot_state, 10)
        self.create_subscription(GripperStatus, "gripper_state", self._on_gripper_state, 10)

        handler = partial(ConsoleRequestHandler, console=self)
        self._server = ThreadingHTTPServer((self.host, self.port), handler)
        self._server_thread = threading.Thread(
            target=self._server.serve_forever,
            daemon=True,
        )
        self._server_thread.start()
        self.get_logger().info(f"Dobot control console: http://{self.host}:{self.port}")

    def destroy_node(self):
        self._server.shutdown()
        self._server.server_close()
        super().destroy_node()

    def api_state(self) -> Dict[str, Any]:
        data = {
            "ok": True,
            "cached": self._cached_state(),
            "services": self._service_availability(),
        }

        robot_state = self._call_service(
            self._robot_state_client,
            GetRobotState.Request(),
        )
        joint_state = self._call_service(
            self._joint_state_client,
            GetJointState.Request(),
        )
        tcp_pose = self._call_service(
            self._tcp_pose_client,
            GetTcpPose.Request(),
        )
        gripper_state = self._call_service(
            self._gripper_state_client,
            GripperState.Request(),
        )
        teach_status = self.api_trigger("teach_status")

        data["robot_state"] = robot_state
        data["joint_state"] = joint_state
        data["tcp_pose"] = tcp_pose
        data["gripper_state"] = gripper_state
        data["teach_status"] = teach_status
        return data

    def api_trajectories(self) -> Dict[str, Any]:
        result = self._call_service(self._teach_list_client, TrajectoryList.Request())
        return result

    def api_trigger(self, name: str) -> Dict[str, Any]:
        client = self._trigger_clients.get(name)
        if client is None:
            return {"ok": False, "message": f"unknown trigger service: {name}"}
        return self._call_service(client, Trigger.Request())

    def api_move(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        kind = str(payload.get("kind", "")).strip()
        client = self._move_clients.get(kind)
        if client is None:
            return {"ok": False, "message": f"unknown move service: {kind}"}

        request = MoveCommand.Request()
        request.target = self._six_float_values(payload.get("target", []))
        request.user = int(payload.get("user", 0))
        request.tool = int(payload.get("tool", 0))
        request.speed = int(payload.get("speed", 0))
        request.acceleration = int(payload.get("acceleration", 0))
        request.wait = bool(payload.get("wait", True))
        request.timeout_sec = float(payload.get("timeout_sec", 20.0))
        return self._call_service(client, request)

    def api_gripper_move(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        request = GripperCommand.Request()
        request.opening_mm = float(payload.get("opening_mm", -1.0))
        request.position_permille = int(payload.get("position_permille", 1000))
        request.force_percent = int(payload.get("force_percent", 50))
        request.force_n = float(payload.get("force_n", -1.0))
        request.wait = bool(payload.get("wait", True))
        request.timeout_sec = float(payload.get("timeout_sec", 20.0))
        return self._call_service(self._gripper_move_client, request)

    def api_teach(self, action: str, payload: Dict[str, Any]) -> Dict[str, Any]:
        if action == "list":
            return self.api_trajectories()

        service_name = f"teach_{action}"
        client = self._teach_clients.get(service_name)
        if client is None:
            return {"ok": False, "message": f"unknown teach action: {action}"}

        request = TrajectoryCommand.Request()
        request.name = str(payload.get("name", ""))
        request.overwrite = bool(payload.get("overwrite", False))
        request.speed = int(payload.get("speed", 0))
        request.acceleration = int(payload.get("acceleration", 0))
        request.replay_mode = str(payload.get("replay_mode", ""))
        request.override_wait = bool(payload.get("override_wait", False))
        request.wait = bool(payload.get("wait", True))
        request.timeout_sec = float(payload.get("timeout_sec", 20.0))
        return self._call_service(client, request)

    def _call_service(self, client, request) -> Dict[str, Any]:
        if not client.wait_for_service(timeout_sec=0.2):
            return {
                "ok": False,
                "available": False,
                "message": f"service not available: {client.srv_name}",
            }

        future = client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout=self.service_timeout_sec):
            return {
                "ok": False,
                "available": True,
                "message": f"service timeout: {client.srv_name}",
            }
        try:
            response = future.result()
        except Exception as exc:
            return {
                "ok": False,
                "available": True,
                "message": f"service failed: {exc}",
            }
        return self._response_to_dict(response)

    def _cached_state(self) -> Dict[str, Any]:
        with self._latest_lock:
            return {
                "joint_state": dict(self._latest_joint_state),
                "tcp_pose": list(self._latest_tcp_pose),
                "dobot_state": dict(self._latest_dobot_state),
                "gripper_state": dict(self._latest_gripper_state),
            }

    def _service_availability(self) -> Dict[str, bool]:
        clients = {}
        clients.update(self._trigger_clients)
        clients.update(self._move_clients)
        clients.update(self._teach_clients)
        clients["teach_list"] = self._teach_list_client
        clients["gripper_move"] = self._gripper_move_client
        clients["get_gripper_state"] = self._gripper_state_client
        clients["get_robot_state"] = self._robot_state_client
        clients["get_joint_state"] = self._joint_state_client
        clients["get_tcp_pose"] = self._tcp_pose_client
        return {
            name: client.service_is_ready()
            for name, client in sorted(clients.items())
        }

    def _on_joint_state(self, message: JointState) -> None:
        with self._latest_lock:
            self._latest_joint_state = {
                "name": list(message.name),
                "position": [float(value) for value in message.position],
            }

    def _on_tcp_pose(self, message: Float64MultiArray) -> None:
        with self._latest_lock:
            self._latest_tcp_pose = [float(value) for value in message.data]

    def _on_dobot_state(self, message: DobotState) -> None:
        with self._latest_lock:
            self._latest_dobot_state = self._response_to_dict(message)

    def _on_gripper_state(self, message: GripperStatus) -> None:
        with self._latest_lock:
            self._latest_gripper_state = self._response_to_dict(message)

    def _resolve_static_dir(self, requested: str) -> Path:
        if requested:
            return Path(requested).expanduser().resolve()
        try:
            from ament_index_python.packages import get_package_share_directory

            return Path(get_package_share_directory("dobot_ros2")) / "web"
        except Exception:
            return Path(__file__).resolve().parents[1] / "web"

    def _six_float_values(self, values) -> list:
        result = [float(value) for value in list(values)[:6]]
        while len(result) < 6:
            result.append(0.0)
        return result

    def _response_to_dict(self, response) -> Dict[str, Any]:
        result = {"ok": bool(getattr(response, "success", True))}
        fields = getattr(response, "get_fields_and_field_types", lambda: {})()
        for field in fields:
            result[field] = self._json_value(getattr(response, field, None))
        return result

    def _json_value(self, value):
        if hasattr(value, "tolist"):
            value = value.tolist()
        if isinstance(value, (str, int, float, bool)) or value is None:
            return value
        if isinstance(value, (array, list, tuple)):
            return [self._json_value(item) for item in value]
        return str(value)


class ConsoleRequestHandler(BaseHTTPRequestHandler):
    """Small HTTP API and static file server for the control console."""

    def __init__(self, *args, console: DobotControlConsole, **kwargs):
        self.console = console
        super().__init__(*args, **kwargs)

    def do_GET(self):
        parsed = urlparse(self.path)
        if parsed.path == "/api/state":
            self._send_json(self.console.api_state())
            return
        if parsed.path == "/api/trajectories":
            self._send_json(self.console.api_trajectories())
            return
        self._serve_static(parsed.path)

    def do_POST(self):
        parsed = urlparse(self.path)
        payload = self._read_json()
        if parsed.path == "/api/trigger":
            self._send_json(self.console.api_trigger(str(payload.get("service", ""))))
            return
        if parsed.path == "/api/move":
            self._send_json(self.console.api_move(payload))
            return
        if parsed.path == "/api/gripper/move":
            self._send_json(self.console.api_gripper_move(payload))
            return
        if parsed.path.startswith("/api/teach/"):
            action = parsed.path.rsplit("/", 1)[-1]
            self._send_json(self.console.api_teach(action, payload))
            return
        self._send_json({"ok": False, "message": "not found"}, status=404)

    def log_message(self, fmt, *args):
        self.console.get_logger().debug(fmt % args)

    def _read_json(self) -> Dict[str, Any]:
        length = int(self.headers.get("Content-Length", "0") or 0)
        if length <= 0:
            return {}
        raw = self.rfile.read(length).decode("utf-8")
        try:
            return json.loads(raw)
        except json.JSONDecodeError:
            return {}

    def _send_json(self, payload: Dict[str, Any], status: int = 200) -> None:
        data = json.dumps(payload, separators=(",", ":")).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def _serve_static(self, request_path: str) -> None:
        static_root = self.console.static_dir.resolve()
        relative = normpath(unquote(request_path)).lstrip("/")
        if relative in ("", "."):
            relative = "index.html"
        path = static_root / relative
        try:
            path.relative_to(static_root)
        except ValueError:
            self._send_json({"ok": False, "message": "invalid path"}, status=403)
            return
        if path.is_dir():
            path = path / "index.html"
        if not path.exists() or not path.is_file():
            self._send_json({"ok": False, "message": "not found"}, status=404)
            return

        content = path.read_bytes()
        content_type = mimetypes.guess_type(str(path))[0] or "application/octet-stream"
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(content)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(content)


def main(args=None):
    rclpy.init(args=args)
    node = DobotControlConsole()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
