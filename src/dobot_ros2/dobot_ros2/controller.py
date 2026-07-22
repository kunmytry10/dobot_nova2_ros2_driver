import json
import re
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Callable, List, Optional, Sequence, Tuple

from dobot_api import DobotApiDashboard, DobotApiFeedBack, DobotApiMove


FEEDBACK_MAGIC = 0x123456789ABCDEF
ENABLED_ROBOT_MODES = {5, 7, 10, 11}
DISABLED_ROBOT_MODES = {3, 4}
ERROR_ROBOT_MODE = 9
ROBOT_MODE_TEXT = {
    1: "INIT",
    2: "BRAKE_OPEN",
    3: "POWER_DISABLED",
    4: "NOT_ENABLE",
    5: "ENABLE",
    6: "BACKDRIVE",
    7: "RUNNING",
    8: "RECORDING",
    9: "ERROR",
    10: "PAUSE",
    11: "JOG",
}
_ALARM_INDEX = None
TCP_ERROR_TEXT = {
    -1: "没有获取成功/命令接收失败或执行失败",
    -10000: "命令错误：下发的命令不存在",
    -20000: "参数数量错误：下发命令中的参数数量错误",
}


def _as_text(reply) -> str:
    if isinstance(reply, bytes):
        return reply.decode("utf-8", errors="replace")
    return str(reply or "")


def _reply_error_id(reply: str) -> Optional[int]:
    text = _as_text(reply)
    match = re.search(r"^\s*(-?\d+)", text)
    if match is None:
        match = re.search(r"-?\d+", text)
    return int(match.group(1 if match.lastindex else 0)) if match else None


def _parse_braced_values(reply: str, expected_count: int = 6) -> List[float]:
    text = _as_text(reply)
    match = re.search(r"\{([^{}]*)\}", text)
    if match is None:
        return []
    values = [
        float(value)
        for value in re.findall(
            r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?",
            match.group(1),
        )
    ]
    if expected_count and len(values) < expected_count:
        return []
    if expected_count <= 0:
        return values
    return values[:expected_count]


def _positive_ints(reply: str) -> List[int]:
    text = _as_text(reply)
    match = re.search(r"\{([^{}]*)\}", text)
    source = match.group(1) if match is not None else text
    return [int(value) for value in re.findall(r"-?\d+", source) if int(value) > 0]


def _format_values(values: Sequence[float]) -> str:
    return ",".join(f"{float(value):.6f}" for value in values)


def _format_joint_near(values: Sequence[float]) -> str:
    return "{" + _format_values(values) + "}"


def _alarm_index() -> dict:
    global _ALARM_INDEX
    if _ALARM_INDEX is not None:
        return _ALARM_INDEX

    bases = [Path(__file__).resolve().parents[1] / "files"]
    try:
        from ament_index_python.packages import get_package_share_directory

        bases.append(Path(get_package_share_directory("dobot_ros2")) / "files")
    except Exception:
        pass

    index = {}
    for base in bases:
        for source, filename in (
            ("controller", "alarm_controller.json"),
            ("servo", "alarm_servo.json"),
        ):
            path = base / filename
            if not path.exists():
                continue
            try:
                with path.open(encoding="utf-8") as file:
                    for item in json.load(file):
                        alarm = dict(item)
                        alarm["source"] = source
                        index.setdefault(int(alarm["id"]), []).append(alarm)
            except Exception:
                continue
    _ALARM_INDEX = index
    return _ALARM_INDEX


def _format_error_details(error_ids: Sequence[int]) -> str:
    if not error_ids:
        return "no active error ids"

    index = _alarm_index()
    details = []
    for error_id in error_ids[:6]:
        error_id = int(error_id)
        if error_id < 0:
            if error_id in TCP_ERROR_TEXT:
                details.append(f"{error_id}: {TCP_ERROR_TEXT[error_id]}")
                continue
            if -30099 <= error_id <= -30000:
                index_value = abs(error_id) - 30000
                details.append(f"{error_id}: 参数类型错误，第{index_value}个参数类型错误")
                continue
            if -40099 <= error_id <= -40000:
                index_value = abs(error_id) - 40000
                details.append(f"{error_id}: 参数范围错误，第{index_value}个参数范围错误")
                continue
        alarms = index.get(int(error_id), [])
        if not alarms:
            details.append(f"{error_id}: unknown error")
            continue
        alarm = sorted(
            alarms,
            key=lambda item: 0 if item.get("source") == "controller" else 1,
        )[0]
        zh = alarm.get("zh_CN", {})
        en = alarm.get("en", {})
        description = zh.get("description") or en.get("description") or "unknown error"
        solution = zh.get("solution") or en.get("solution") or ""
        text = f"{error_id}: {description}"
        if solution:
            text += f"; {solution}"
        details.append(text)
    if len(error_ids) > 6:
        details.append(f"... {len(error_ids) - 6} more")
    return "error_ids=" + ",".join(str(value) for value in error_ids) + "; " + " | ".join(details)


def _max_abs_delta(actual: Sequence[float], target: Sequence[float]) -> float:
    if not actual or not target:
        return float("inf")
    return max(abs(float(a) - float(b)) for a, b in zip(actual, target))


@dataclass
class ControllerConfig:
    """Runtime settings for Dobot TCP/IP control.

    Dobot command units are kept unchanged here: Cartesian positions are mm,
    Cartesian rotations are deg, and joint targets are deg.
    """

    robot_ip: str = "192.168.5.1"
    dashboard_port: int = 29999
    move_port: int = 30003
    feedback_port: int = 30004
    default_user: int = 0
    default_tool: int = 0
    default_speed_j: int = 0
    default_acc_j: int = 0
    default_speed_l: int = 0
    default_acc_l: int = 0
    robot_model: str = "Nova 2"
    rated_payload_kg: float = 2.0
    workspace_radius_mm: float = 625.0
    max_tcp_speed_mps: float = 1.6
    repeatability_mm: float = 0.05
    joint_zero_deg: List[float] = field(default_factory=lambda: [0.0] * 6)
    joint_lower_limits_deg: List[float] = field(
        default_factory=lambda: [-360.0, -180.0, -156.0, -360.0, -360.0, -360.0]
    )
    joint_upper_limits_deg: List[float] = field(
        default_factory=lambda: [360.0, 180.0, 156.0, 360.0, 360.0, 360.0]
    )
    max_joint_speed_deg_s: List[float] = field(default_factory=lambda: [135.0] * 6)
    joint_limit_check: bool = True
    joint_limit_margin_deg: float = 0.0
    command_timeout_sec: float = 3.0
    motion_timeout_sec: float = 30.0
    wait_for_motion: bool = False
    motion_status_check: bool = True
    post_motion_check: bool = True
    post_motion_check_timeout_sec: float = 2.0
    joint_arrival_tolerance_deg: float = 0.5
    tcp_position_tolerance_mm: float = 1.0
    tcp_rotation_tolerance_deg: float = 1.0
    ik_check: bool = True
    ik_use_joint_near: bool = True
    enable_on_start: bool = False
    teach_trajectory_dir: str = "/home/ros/ws/trajectories"
    teach_sample_rate_hz: float = 5.0
    teach_min_joint_delta_deg: float = 0.5
    teach_min_tcp_delta_mm: float = 1.0
    teach_replay_speed: int = 10
    teach_replay_acc: int = 10
    teach_replay_wait: bool = True
    teach_replay_timeout_sec: float = 20.0
    teach_replay_mode: str = "movej"
    teach_servoj_rate_hz: float = 33.0
    teach_servoj_t: float = 0.1
    teach_servoj_lookahead_time: float = 50.0
    teach_servoj_gain: float = 500.0


@dataclass
class FeedbackState:
    """Latest controller feedback frame in Dobot TCP/IP units."""

    joints: List[float] = field(default_factory=list)
    q_target: List[float] = field(default_factory=list)
    tcp_pose: List[float] = field(default_factory=list)
    tcp_target: List[float] = field(default_factory=list)
    robot_mode: int = 0
    speed_scaling: float = 0.0
    enable_status: int = 0
    running_status: int = 0
    error_status: int = 0
    drag_status: int = 0
    record_button_signal: int = 0
    stamp: float = 0.0


@dataclass
class MotionResult:
    """Normalized result returned to ROS service callbacks."""

    success: bool
    error_id: int = -1
    message: str = ""
    raw_reply: str = ""
    ik_reply: str = ""
    ik_joints: List[float] = field(default_factory=lambda: [0.0] * 6)


@dataclass
class DashboardResult:
    """Normalized result returned by dashboard service wrappers."""

    success: bool
    error_id: int = -1
    message: str = ""
    raw_reply: str = ""
    value: int = 0
    values: List[int] = field(default_factory=list)


@dataclass
class TeachResult:
    """Normalized result for teach record/replay operations."""

    success: bool
    error_id: int = -1
    message: str = ""
    trajectory_name: str = ""
    path: str = ""
    point_count: int = 0
    raw_reply: str = ""


class DobotController:
    """Owns the Dobot TCP clients and serializes motion command traffic."""

    def __init__(
        self,
        config: ControllerConfig,
        feedback_callback: Optional[Callable[[FeedbackState], None]] = None,
        log_callback: Optional[Callable[[str], None]] = None,
    ):
        self.config = config
        self.feedback_callback = feedback_callback
        self.log_callback = log_callback
        self.dashboard: Optional[DobotApiDashboard] = None
        self.move_client: Optional[DobotApiMove] = None
        self._feedback_client: Optional[DobotApiFeedBack] = None
        self._feedback_thread: Optional[threading.Thread] = None
        self._feedback_running = False
        self._state = FeedbackState()
        self._state_lock = threading.Lock()
        self._client_lock = threading.RLock()
        self._command_lock = threading.Lock()
        self._teach_lock = threading.RLock()
        self._teach_thread: Optional[threading.Thread] = None
        self._teach_recording = False
        self._teach_name = ""
        self._teach_started_at = 0.0
        self._teach_points: List[dict] = []

    def connect(self) -> None:
        with self._client_lock:
            if self.dashboard is not None and self.move_client is not None:
                self._start_feedback()
                return
            dashboard = None
            move_client = None
            try:
                dashboard = DobotApiDashboard(self.config.robot_ip, self.config.dashboard_port)
                move_client = DobotApiMove(self.config.robot_ip, self.config.move_port)
            except Exception:
                for client in (dashboard, move_client):
                    if client is not None:
                        try:
                            client.close()
                        except Exception:
                            pass
                raise

            self.dashboard = dashboard
            self.move_client = move_client
            self._start_feedback()
            if self.config.enable_on_start:
                self._send_dashboard("EnableRobot()", self.config.command_timeout_sec)

    def disconnect(self) -> None:
        with self._client_lock:
            self._feedback_running = False
            if self._feedback_client is not None:
                try:
                    self._feedback_client.close()
                except Exception:
                    pass
            if self._feedback_thread is not None:
                self._feedback_thread.join(timeout=1.5)
                self._feedback_thread = None
            for client in (self.dashboard, self.move_client):
                if client is not None:
                    try:
                        client.close()
                    except Exception:
                        pass
            self.dashboard = None
            self.move_client = None
            self._feedback_client = None

    def is_connected(self) -> bool:
        return self.dashboard is not None and self.move_client is not None

    def latest_state(self) -> FeedbackState:
        with self._state_lock:
            return FeedbackState(**self._state.__dict__)

    def clear_error(self) -> DashboardResult:
        return self._dashboard_command("ClearError()", "clear_error")

    def enable_robot(self) -> DashboardResult:
        return self._dashboard_command("EnableRobot()", "enable_robot")

    def disable_robot(self) -> DashboardResult:
        return self._dashboard_command("DisableRobot()", "disable_robot")

    def emergency_stop(self) -> DashboardResult:
        return self._dashboard_command("EmergencyStop()", "emergency_stop")

    def dashboard_command(self, command: str, label: str) -> DashboardResult:
        return self._dashboard_command(command, label)

    def teach_start(self, name: str = "", overwrite: bool = False) -> TeachResult:
        with self._teach_lock:
            if self._teach_recording:
                return TeachResult(
                    False,
                    message=f"teach recording already active: {self._teach_name}",
                    trajectory_name=self._teach_name,
                    point_count=len(self._teach_points),
                )

            trajectory_name = self._normalize_trajectory_name(name)
            path = self._trajectory_path(trajectory_name)
            if path.exists() and not overwrite:
                return TeachResult(
                    False,
                    message=f"trajectory already exists: {trajectory_name}",
                    trajectory_name=trajectory_name,
                    path=str(path),
                )

            ready = self._check_ready_for_motion()
            if not ready.success:
                return TeachResult(
                    False,
                    ready.error_id,
                    ready.message,
                    trajectory_name,
                    str(path),
                )

            drag = self._dashboard_command("StartDrag()", "teach_start")
            if not drag.success:
                return TeachResult(
                    False,
                    drag.error_id,
                    drag.message,
                    trajectory_name,
                    str(path),
                    raw_reply=drag.raw_reply,
                )

            self._teach_recording = True
            self._teach_name = trajectory_name
            self._teach_started_at = time.time()
            self._teach_points = []
            self._teach_thread = threading.Thread(target=self._teach_record_loop, daemon=True)
            self._teach_thread.start()
            return TeachResult(
                True,
                0,
                f"teach recording started: {trajectory_name}",
                trajectory_name,
                str(path),
                raw_reply=drag.raw_reply,
            )

    def teach_stop(self, name: str = "") -> TeachResult:
        with self._teach_lock:
            if not self._teach_recording:
                return TeachResult(False, message="teach recording is not active")
            trajectory_name = self._normalize_trajectory_name(name or self._teach_name)
            self._teach_recording = False

        if self._teach_thread is not None:
            self._teach_thread.join(timeout=2.0)
            self._teach_thread = None

        drag = self._dashboard_command("StopDrag()", "teach_stop")
        with self._teach_lock:
            points = list(self._teach_points)
            self._teach_name = ""
            self._teach_started_at = 0.0
            self._teach_points = []

        path = self._trajectory_path(trajectory_name)
        if not points:
            return TeachResult(
                False,
                drag.error_id if drag.error_id != 0 else -1,
                "teach recording stopped but no feedback points were captured",
                trajectory_name,
                str(path),
                raw_reply=drag.raw_reply,
            )

        try:
            self._save_trajectory(trajectory_name, points, path)
        except Exception as exc:
            return TeachResult(
                False,
                -1,
                f"failed to save trajectory: {exc}",
                trajectory_name,
                str(path),
                len(points),
                drag.raw_reply,
            )

        if not drag.success:
            message = f"trajectory saved, but StopDrag failed: {drag.message}"
            error_id = drag.error_id
        else:
            message = f"teach recording saved: {trajectory_name}"
            error_id = 0
        return TeachResult(
            drag.success,
            error_id,
            message,
            trajectory_name,
            str(path),
            len(points),
            drag.raw_reply,
        )

    def teach_replay(
        self,
        name: str,
        speed: int = 0,
        acceleration: int = 0,
        replay_mode: str = "",
        wait: Optional[bool] = None,
        timeout_sec: float = 0.0,
    ) -> TeachResult:
        trajectory_name = self._normalize_trajectory_name(name, generate=False)
        if not trajectory_name:
            return TeachResult(False, message="trajectory name is required")

        path = self._trajectory_path(trajectory_name)
        try:
            trajectory = self._load_trajectory(path)
        except Exception as exc:
            return TeachResult(
                False,
                message=f"failed to load trajectory: {exc}",
                path=str(path),
            )

        points = trajectory.get("points", [])
        if not points:
            return TeachResult(
                False,
                message="trajectory contains no points",
                trajectory_name=trajectory_name,
                path=str(path),
            )

        replay_speed = int(speed) if int(speed) > 0 else int(self.config.teach_replay_speed)
        replay_acc = (
            int(acceleration)
            if int(acceleration) > 0
            else int(self.config.teach_replay_acc)
        )
        replay_wait = self.config.teach_replay_wait if wait is None else bool(wait)
        timeout = float(timeout_sec or self.config.teach_replay_timeout_sec)
        mode = self._teach_replay_mode(replay_mode)

        if mode == "servoj":
            return self._teach_replay_servoj(
                trajectory_name,
                path,
                points,
                replay_speed,
                replay_acc,
                timeout,
            )
        if mode != "movej":
            return TeachResult(
                False,
                message=f"unsupported teach replay mode: {mode}",
                trajectory_name=trajectory_name,
                path=str(path),
                point_count=len(points),
            )

        first_joints = points[0]["joints_deg"]
        result = self.move(
            "movej",
            first_joints,
            speed=replay_speed,
            acceleration=replay_acc,
            wait=True,
            timeout_sec=timeout,
        )
        if not result.success:
            return TeachResult(
                False,
                result.error_id,
                f"failed to move to trajectory start: {result.message}",
                trajectory_name,
                str(path),
                len(points),
                result.raw_reply,
            )

        raw_replies = [result.raw_reply]
        for point in points[1:]:
            result = self.move(
                "movej",
                point["joints_deg"],
                speed=replay_speed,
                acceleration=replay_acc,
                wait=replay_wait,
                timeout_sec=timeout,
            )
            raw_replies.append(result.raw_reply)
            if not result.success:
                return TeachResult(
                    False,
                    result.error_id,
                    f"trajectory replay stopped: {result.message}",
                    trajectory_name,
                    str(path),
                    len(points),
                    " | ".join(raw_replies[-3:]),
                )

        return TeachResult(
            True,
            0,
            f"trajectory replay accepted: {trajectory_name}",
            trajectory_name,
            str(path),
            len(points),
            " | ".join(raw_replies[-3:]),
        )

    def _teach_replay_servoj(
        self,
        trajectory_name: str,
        path: Path,
        points: Sequence[dict],
        speed: int,
        acceleration: int,
        timeout_sec: float,
    ) -> TeachResult:
        for point in points:
            limit_result = self._check_joint_limits(
                "servoj",
                point["joints_deg"],
                point["joints_deg"],
            )
            if not limit_result.success:
                return TeachResult(
                    False,
                    limit_result.error_id,
                    limit_result.message,
                    trajectory_name,
                    str(path),
                    len(points),
                )

        first_joints = points[0]["joints_deg"]
        result = self.move(
            "movej",
            first_joints,
            speed=speed,
            acceleration=acceleration,
            wait=True,
            timeout_sec=timeout_sec,
        )
        if not result.success:
            return TeachResult(
                False,
                result.error_id,
                f"failed to move to trajectory start: {result.message}",
                trajectory_name,
                str(path),
                len(points),
                result.raw_reply,
            )

        try:
            samples = self._resample_teach_points(points, self.config.teach_servoj_rate_hz)
            sent_count = self._send_servoj_samples(samples)
        except Exception as exc:
            return TeachResult(
                False,
                -1,
                f"servoj replay failed: {exc}",
                trajectory_name,
                str(path),
                len(points),
                result.raw_reply,
            )

        state = self.latest_state()
        if state.error_status or state.robot_mode == ERROR_ROBOT_MODE:
            error_detail = self.get_error_id()
            return TeachResult(
                False,
                error_detail.value if error_detail.value else -1,
                (
                    "servoj replay sent but robot entered error status; "
                    f"{error_detail.message}"
                ),
                trajectory_name,
                str(path),
                len(points),
                error_detail.raw_reply,
            )

        return TeachResult(
            True,
            0,
            (
                f"trajectory servoj replay sent: {trajectory_name}; "
                f"samples={sent_count}"
            ),
            trajectory_name,
            str(path),
            len(points),
            result.raw_reply,
        )

    def teach_delete(self, name: str) -> TeachResult:
        trajectory_name = self._normalize_trajectory_name(name, generate=False)
        if not trajectory_name:
            return TeachResult(False, message="trajectory name is required")
        path = self._trajectory_path(trajectory_name)
        if not path.exists():
            return TeachResult(
                False,
                message=f"trajectory not found: {trajectory_name}",
                trajectory_name=trajectory_name,
                path=str(path),
            )
        try:
            path.unlink()
        except Exception as exc:
            return TeachResult(
                False,
                message=f"failed to delete trajectory: {exc}",
                trajectory_name=trajectory_name,
                path=str(path),
            )
        return TeachResult(
            True,
            0,
            f"trajectory deleted: {trajectory_name}",
            trajectory_name,
            str(path),
        )

    def teach_list(self) -> List[TeachResult]:
        directory = self._trajectory_dir()
        results = []
        if not directory.exists():
            return results
        for path in sorted(directory.glob("*.json")):
            try:
                data = self._load_trajectory(path)
                name = str(data.get("name") or path.stem)
                points = data.get("points", [])
                results.append(
                    TeachResult(True, 0, "trajectory", name, str(path), len(points))
                )
            except Exception:
                results.append(
                    TeachResult(False, -1, "invalid trajectory", path.stem, str(path))
                )
        return results

    def teach_status(self) -> TeachResult:
        with self._teach_lock:
            return TeachResult(
                True,
                0,
                "teach recording active" if self._teach_recording else "teach recording idle",
                self._teach_name,
                str(self._trajectory_path(self._teach_name)) if self._teach_name else "",
                len(self._teach_points),
            )

    def robot_mode(self) -> DashboardResult:
        result = self._dashboard_command("RobotMode()", "robot_mode")
        mode = self._first_braced_int(result.raw_reply)
        if result.success and mode is not None:
            result.value = mode
            result.values = [mode]
            result.message = f"robot_mode={mode} {ROBOT_MODE_TEXT.get(mode, '')}".strip()
        elif result.success:
            result.message = f"robot_mode reply did not contain a mode: {result.raw_reply}"
        return result

    def get_error_id(self) -> DashboardResult:
        result = self._dashboard_command("GetErrorID()", "get_error_id")
        if result.success:
            error_ids = _positive_ints(result.raw_reply)
            result.values = error_ids
            result.value = error_ids[0] if error_ids else 0
            result.message = _format_error_details(error_ids)
        return result

    def move(
        self,
        kind: str,
        target: Sequence[float],
        user: int = 0,
        tool: int = 0,
        speed: int = 0,
        acceleration: int = 0,
        wait: bool = False,
        timeout_sec: float = 0.0,
    ) -> MotionResult:
        """Run one motion command after validating reachability with IK."""

        if len(target) != 6:
            return MotionResult(False, message="target must contain exactly 6 values")
        try:
            self.connect()
            effective_user = self.config.default_user if user < 0 else int(user)
            effective_tool = self.config.default_tool if tool < 0 else int(tool)
            timeout = float(timeout_sec or self.config.motion_timeout_sec)
            speed_value, acc_value = self._effective_motion_options(kind, speed, acceleration)

            status_result = self._check_ready_for_motion()
            if not status_result.success:
                return status_result

            ik_result = self._check_ik(kind, target, effective_user, effective_tool)
            if not ik_result.success:
                return ik_result

            limit_result = self._check_joint_limits(kind, target, ik_result.ik_joints)
            if not limit_result.success:
                return limit_result

            command_started_at = time.time()
            command_target = ik_result.ik_joints if kind == "movep" else target
            command = self._build_motion_command(
                kind,
                command_target,
                effective_user,
                effective_tool,
                speed_value,
                acc_value,
            )
            raw_reply = self._send_move_with_reconnect(command, self.config.command_timeout_sec)
            error_id = _reply_error_id(raw_reply)
            if error_id != 0:
                self._reconnect_move_after_failure(kind, raw_reply)
                return MotionResult(
                    False,
                    error_id=error_id if error_id is not None else -1,
                    message=self._motion_failure_message(kind, error_id, raw_reply),
                    raw_reply=raw_reply,
                    ik_reply=ik_result.ik_reply,
                    ik_joints=ik_result.ik_joints,
                )

            wait_reply = ""
            if wait or self.config.wait_for_motion:
                wait_reply = self._send_move_with_reconnect("Sync()", timeout)
                sync_error = _reply_error_id(wait_reply)
                post_result = self._check_post_motion_state(
                    kind,
                    target,
                    command_started_at,
                    raw_reply,
                    wait_reply,
                    ik_result,
                )
                if post_result is not None:
                    if sync_error != 0:
                        post_result.message = f"Sync failed; {post_result.message}"
                        if post_result.error_id == -1:
                            post_result.error_id = sync_error if sync_error is not None else -1
                        self._reconnect_move_after_failure("Sync", wait_reply)
                    return post_result
                if sync_error != 0:
                    self._reconnect_move_after_failure("Sync", wait_reply)
                    return MotionResult(
                        True,
                        error_id=0,
                        message=(
                            "target reached and feedback is healthy, but Sync returned "
                            f"{sync_error if sync_error is not None else 'no error code'}"
                        ),
                        raw_reply=f"{raw_reply} | Sync -> {wait_reply}",
                        ik_reply=ik_result.ik_reply,
                        ik_joints=ik_result.ik_joints,
                    )

            return MotionResult(
                True,
                error_id=0,
                message=self._motion_success_message(kind),
                raw_reply=f"{raw_reply} | Sync -> {wait_reply}" if wait_reply else raw_reply,
                ik_reply=ik_result.ik_reply,
                ik_joints=ik_result.ik_joints,
            )
        except Exception as exc:
            return MotionResult(False, message=str(exc))

    def _dashboard_command(self, command: str, label: str) -> DashboardResult:
        try:
            self.connect()
            raw_reply = self._send_dashboard_with_reconnect(
                command,
                self.config.command_timeout_sec,
            )
            error_id = _reply_error_id(raw_reply)
            success = error_id == 0
            return DashboardResult(
                success=success,
                error_id=error_id if error_id is not None else -1,
                message=f"{label} {'accepted' if success else 'rejected'}",
                raw_reply=raw_reply,
            )
        except Exception as exc:
            return DashboardResult(False, message=f"{label} failed: {exc}")

    def _start_feedback(self) -> None:
        if self._feedback_thread is not None and self._feedback_thread.is_alive():
            return
        self._feedback_running = True
        self._feedback_thread = threading.Thread(target=self._feedback_loop, daemon=True)
        self._feedback_thread.start()

    def _feedback_loop(self) -> None:
        try:
            self._feedback_client = DobotApiFeedBack(
                self.config.robot_ip,
                self.config.feedback_port,
            )
            while self._feedback_running:
                packet = self._feedback_client.feedBackData()
                if packet is None or len(packet) == 0:
                    continue
                if int(packet["test_value"][0]) != FEEDBACK_MAGIC:
                    continue
                state = FeedbackState(
                    joints=[float(value) for value in packet["q_actual"][0]],
                    q_target=[float(value) for value in packet["q_target"][0]],
                    tcp_pose=[float(value) for value in packet["tool_vector_actual"][0]],
                    tcp_target=[float(value) for value in packet["Tool_vector_target"][0]],
                    robot_mode=int(packet["robot_mode"][0]),
                    speed_scaling=float(packet["speed_scaling"][0]),
                    enable_status=int(packet["enable_status"][0]),
                    running_status=int(packet["running_status"][0]),
                    error_status=int(packet["error_status"][0]),
                    drag_status=int(packet["drag_status"][0]),
                    record_button_signal=int(packet["record_button_signal"][0]),
                    stamp=time.time(),
                )
                with self._state_lock:
                    self._state = state
                if self.feedback_callback is not None:
                    self.feedback_callback(state)
        except Exception as exc:
            self._log(f"feedback stopped: {exc}")
        finally:
            if self._feedback_client is not None:
                try:
                    self._feedback_client.close()
                except Exception:
                    pass
                self._feedback_client = None

    def _check_ik(
        self,
        kind: str,
        target: Sequence[float],
        user: int,
        tool: int,
    ) -> MotionResult:
        if kind == "movej":
            return MotionResult(True, error_id=0, message="joint target does not need IK")
        if not self.config.ik_check:
            return MotionResult(True, error_id=0, message="inverse kinematics check disabled")
        return self._check_cartesian_ik(target, user, tool, joint_near=None)

    def _check_cartesian_ik(
        self,
        pose: Sequence[float],
        user: int,
        tool: int,
        joint_near: Optional[Sequence[float]] = None,
    ) -> MotionResult:
        if joint_near is None and self.config.ik_use_joint_near:
            state = self.latest_state()
            if len(state.joints) == 6:
                joint_near = state.joints

        # Dobot can return multiple IK branches for the same TCP pose. Passing
        # the current joints as JointNear keeps the selected branch predictable.
        suffix = ""
        if self.config.ik_use_joint_near and joint_near is not None:
            suffix = f",1,{_format_joint_near(joint_near)}"
        command = f"InverseSolution({_format_values(pose)},{user:d},{tool:d}{suffix})"
        try:
            reply = self._send_dashboard_with_reconnect(command, self.config.command_timeout_sec)
        except Exception as exc:
            return MotionResult(
                False,
                error_id=-1,
                message=f"inverse kinematics check transport failed: {exc}",
                ik_reply=f"{command} -> no reply",
            )
        error_id = _reply_error_id(reply)
        joints = _parse_braced_values(reply)
        if error_id != 0 or len(joints) != 6:
            return MotionResult(
                False,
                error_id=error_id if error_id is not None else -1,
                message="inverse kinematics check failed",
                ik_reply=reply,
            )
        return MotionResult(
            True,
            error_id=0,
            message="ik check passed",
            ik_reply=reply,
            ik_joints=joints,
        )

    def _check_ready_for_motion(self) -> MotionResult:
        if not self.config.motion_status_check:
            return MotionResult(True, error_id=0, message="motion status check disabled")

        state = self.latest_state()
        if state.stamp > 0.0:
            if state.error_status:
                error_detail = self.get_error_id()
                return MotionResult(
                    False,
                    error_id=error_detail.value if error_detail.value else -1,
                    message=(
                        "robot is in error status; call /get_error_id for details and "
                        f"/clear_error after removing the cause; {error_detail.message}"
                    ),
                    raw_reply=error_detail.raw_reply,
                )
            if not state.enable_status:
                return MotionResult(
                    False,
                    error_id=-1,
                    message=(
                        f"robot is not enabled (enable_status={state.enable_status}, "
                        f"robot_mode={state.robot_mode} {ROBOT_MODE_TEXT.get(state.robot_mode, '')}); "
                        "call /enable_robot first"
                    ),
                )
            return MotionResult(True, error_id=0, message="robot status check passed")

        mode_result = self.robot_mode()
        if not mode_result.success:
            return MotionResult(
                False,
                error_id=mode_result.error_id,
                message=f"could not verify robot mode before motion: {mode_result.message}",
                raw_reply=mode_result.raw_reply,
            )

        mode = mode_result.value
        if mode == ERROR_ROBOT_MODE:
            error_detail = self.get_error_id()
            return MotionResult(
                False,
                error_id=error_detail.value if error_detail.value else -1,
                message=(
                    "robot_mode=9 ERROR; call /get_error_id and /clear_error; "
                    f"{error_detail.message}"
                ),
                raw_reply=error_detail.raw_reply or mode_result.raw_reply,
            )
        if mode in DISABLED_ROBOT_MODES:
            return MotionResult(
                False,
                error_id=-1,
                message=(
                    f"robot is not enabled (robot_mode={mode} {ROBOT_MODE_TEXT.get(mode, '')}); "
                    "call /enable_robot first"
                ),
                raw_reply=mode_result.raw_reply,
            )
        if mode not in ENABLED_ROBOT_MODES:
            return MotionResult(
                False,
                error_id=-1,
                message=f"robot mode is not ready for motion: {mode_result.message}",
                raw_reply=mode_result.raw_reply,
            )
        return MotionResult(True, error_id=0, message="robot mode check passed")

    def _check_joint_limits(
        self,
        kind: str,
        target: Sequence[float],
        ik_joints: Sequence[float],
    ) -> MotionResult:
        if not self.config.joint_limit_check:
            return MotionResult(True, error_id=0, message="joint limit check disabled")

        if kind == "movej":
            joints = target
            source = "target"
        elif len(ik_joints) >= 6:
            joints = ik_joints
            source = "ik_solution"
        else:
            return MotionResult(True, error_id=0, message="no joint target to check")

        lower = self.config.joint_lower_limits_deg
        upper = self.config.joint_upper_limits_deg
        if len(lower) < 6 or len(upper) < 6:
            return MotionResult(
                False,
                error_id=-1,
                message="configured joint limits must contain 6 lower and 6 upper values",
            )

        margin = max(0.0, float(self.config.joint_limit_margin_deg))
        violations = []
        for index, value in enumerate(joints[:6]):
            low = float(lower[index]) - margin
            high = float(upper[index]) + margin
            joint_value = float(value)
            if joint_value < low or joint_value > high:
                violations.append(
                    f"joint{index + 1}={joint_value:.3f} not in "
                    f"[{float(lower[index]):.3f},{float(upper[index]):.3f}] deg"
                )

        if violations:
            return MotionResult(
                False,
                error_id=-1,
                message=(
                    f"{kind} {source} outside configured joint limits: "
                    + "; ".join(violations)
                ),
            )
        return MotionResult(True, error_id=0, message="joint limit check passed")

    def _check_post_motion_state(
        self,
        kind: str,
        target: Sequence[float],
        command_started_at: float,
        raw_reply: str,
        wait_reply: str,
        ik_result: MotionResult,
    ) -> Optional[MotionResult]:
        if not self.config.post_motion_check:
            return None

        raw = f"{raw_reply} | Sync -> {wait_reply}"
        state = self._wait_for_feedback_after(
            max(command_started_at, time.time()),
            self.config.post_motion_check_timeout_sec,
        )
        if state is None:
            return MotionResult(
                False,
                error_id=-1,
                message="motion accepted but no fresh feedback was received after Sync",
                raw_reply=raw,
                ik_reply=ik_result.ik_reply,
                ik_joints=ik_result.ik_joints,
            )

        if state.error_status or state.robot_mode == ERROR_ROBOT_MODE:
            error_detail = self.get_error_id()
            error_text = error_detail.message or "no active error id returned"
            return MotionResult(
                False,
                error_id=error_detail.value if error_detail.value else -1,
                message=(
                    "motion accepted but robot entered error status after Sync; "
                    f"robot_mode={state.robot_mode} {ROBOT_MODE_TEXT.get(state.robot_mode, '')}; "
                    f"error_status={state.error_status}; {error_text}"
                ),
                raw_reply=f"{raw} | GetErrorID -> {error_detail.raw_reply}",
                ik_reply=ik_result.ik_reply,
                ik_joints=ik_result.ik_joints,
            )

        if kind == "movej":
            actual = state.joints
            max_delta = _max_abs_delta(actual, target)
            if max_delta > self.config.joint_arrival_tolerance_deg:
                return MotionResult(
                    False,
                    error_id=-1,
                    message=(
                        f"{kind} accepted but target was not reached; "
                        f"max_joint_delta_deg={max_delta:.3f}"
                    ),
                    raw_reply=raw,
                    ik_reply=ik_result.ik_reply,
                    ik_joints=ik_result.ik_joints,
                )
            return None

        actual_pose = state.tcp_pose
        pos_delta = _max_abs_delta(actual_pose[:3], target[:3])
        rot_delta = _max_abs_delta(actual_pose[3:], target[3:])
        if (
            pos_delta > self.config.tcp_position_tolerance_mm
            or rot_delta > self.config.tcp_rotation_tolerance_deg
        ):
            return MotionResult(
                False,
                error_id=-1,
                message=(
                    f"{kind} accepted but TCP target was not reached; "
                    f"max_position_delta_mm={pos_delta:.3f}; "
                    f"max_rotation_delta_deg={rot_delta:.3f}"
                ),
                raw_reply=raw,
                ik_reply=ik_result.ik_reply,
                ik_joints=ik_result.ik_joints,
            )
        return None

    def _wait_for_feedback_after(
        self,
        min_stamp: float,
        timeout_sec: float,
    ) -> Optional[FeedbackState]:
        deadline = time.time() + max(0.0, timeout_sec)
        while time.time() <= deadline:
            state = self.latest_state()
            if state.stamp >= min_stamp:
                return state
            time.sleep(0.05)
        state = self.latest_state()
        return state if state.stamp >= min_stamp else None

    def _effective_motion_options(
        self,
        kind: str,
        speed: int,
        acceleration: int,
    ) -> Tuple[int, int]:
        if kind in ("movej", "movejp", "movep"):
            default_speed = self.config.default_speed_j
            default_acc = self.config.default_acc_j
        else:
            default_speed = self.config.default_speed_l
            default_acc = self.config.default_acc_l
        speed_value = int(speed) if int(speed) > 0 else int(default_speed)
        acc_value = int(acceleration) if int(acceleration) > 0 else int(default_acc)
        for name, value in (("speed", speed_value), ("acceleration", acc_value)):
            if value and not 1 <= value <= 100:
                raise ValueError(f"{name} ratio must be 0 or within 1..100")
        return speed_value, acc_value

    def _build_motion_command(
        self,
        kind: str,
        target: Sequence[float],
        user: int,
        tool: int,
        speed: int,
        acceleration: int,
    ) -> str:
        if kind == "movejp":
            return self._cartesian_command(
                "MovJ",
                target,
                user,
                tool,
                "SpeedJ",
                "AccJ",
                speed,
                acceleration,
            )
        if kind == "movel":
            return self._cartesian_command(
                "MovL",
                target,
                user,
                tool,
                "SpeedL",
                "AccL",
                speed,
                acceleration,
            )
        if kind == "movej":
            options = []
            if speed:
                options.append(f"SpeedJ={speed:d}")
            if acceleration:
                options.append(f"AccJ={acceleration:d}")
            return self._command("JointMovJ", target, options)
        if kind == "movep":
            options = []
            if speed:
                options.append(f"SpeedJ={speed:d}")
            if acceleration:
                options.append(f"AccJ={acceleration:d}")
            return self._command("JointMovJ", target, options)
        raise ValueError(f"unsupported move kind: {kind}")

    def _motion_success_message(self, kind: str) -> str:
        if kind == "movep":
            return "movep accepted via InverseSolution -> JointMovJ"
        return f"{kind} accepted"

    def _cartesian_command(
        self,
        name: str,
        target: Sequence[float],
        user: int,
        tool: int,
        speed_name: str,
        acc_name: str,
        speed: int,
        acceleration: int,
    ) -> str:
        options = [f"User={user:d}", f"Tool={tool:d}"]
        if speed:
            options.append(f"{speed_name}={speed:d}")
        if acceleration:
            options.append(f"{acc_name}={acceleration:d}")
        return self._command(name, target, options)

    def _command(self, name: str, target: Sequence[float], options: Sequence[str]) -> str:
        fields = [_format_values(target)]
        fields.extend(options)
        return f"{name}({','.join(fields)})"

    def _send_dashboard(self, command: str, timeout_sec: float) -> str:
        if self.dashboard is None:
            raise RuntimeError("dashboard client is not connected")
        return self._send(self.dashboard, command, timeout_sec)

    def _send_dashboard_with_reconnect(self, command: str, timeout_sec: float) -> str:
        try:
            reply = self._send_dashboard(command, timeout_sec)
        except Exception as exc:
            self._log(
                f"dashboard command transport failed, reconnecting "
                f"{self.config.dashboard_port}: {exc}"
            )
            self._reconnect_dashboard_client()
            return self._send_dashboard(command, timeout_sec)

        if reply.strip():
            return reply

        self._log(
            f"empty dashboard reply, reconnecting {self.config.dashboard_port} "
            "and retrying once"
        )
        self._reconnect_dashboard_client()
        retry_reply = self._send_dashboard(command, timeout_sec)
        if retry_reply.strip():
            return retry_reply
        raise RuntimeError(f"empty dashboard reply after reconnect for {command}")

    def _send_move(self, command: str, timeout_sec: float) -> str:
        if self.move_client is None:
            raise RuntimeError("move client is not connected")
        return self._send(self.move_client, command, timeout_sec)

    def _send_move_with_reconnect(self, command: str, timeout_sec: float) -> str:
        try:
            reply = self._send_move(command, timeout_sec)
        except Exception as exc:
            self._log(f"move command transport failed, reconnecting {self.config.move_port}: {exc}")
            self._reconnect_move_client()
            return self._send_move(command, timeout_sec)

        if reply.strip():
            return reply

        self._log(f"empty move reply, reconnecting {self.config.move_port} and retrying once")
        self._reconnect_move_client()
        retry_reply = self._send_move(command, timeout_sec)
        if retry_reply.strip():
            return retry_reply
        raise RuntimeError(f"empty move reply after reconnect for {command}")

    def _send(self, client, command: str, timeout_sec: float) -> str:
        with self._command_lock:
            socket_obj = getattr(client, "socket_dobot", None)
            old_timeout = None
            if socket_obj not in (None, 0):
                old_timeout = socket_obj.gettimeout()
                if timeout_sec and timeout_sec > 0.0:
                    socket_obj.settimeout(timeout_sec)
            try:
                return _as_text(client.sendRecvMsg(command))
            finally:
                if socket_obj not in (None, 0) and old_timeout is not None:
                    socket_obj.settimeout(old_timeout)

    def _reconnect_move_client(self) -> None:
        with self._client_lock:
            if self.move_client is not None:
                try:
                    self.move_client.close()
                except Exception:
                    pass
            self.move_client = DobotApiMove(self.config.robot_ip, self.config.move_port)

    def _reconnect_dashboard_client(self) -> None:
        with self._client_lock:
            if self.dashboard is not None:
                try:
                    self.dashboard.close()
                except Exception:
                    pass
            self.dashboard = DobotApiDashboard(
                self.config.robot_ip,
                self.config.dashboard_port,
            )

    def _reconnect_move_after_failure(self, label: str, raw_reply: str) -> None:
        try:
            self._log(
                f"{label} failed on {self.config.move_port}, reconnecting move channel; "
                f"raw_reply={raw_reply}"
            )
            self._reconnect_move_client()
        except Exception as exc:
            self._log(f"move channel reconnect after {label} failure failed: {exc}")

    def _first_braced_int(self, reply: str) -> Optional[int]:
        values = _parse_braced_values(reply, expected_count=0)
        if values:
            return int(values[0])
        return None

    def _motion_failure_message(
        self,
        kind: str,
        error_id: Optional[int],
        raw_reply: str,
    ) -> str:
        details = [f"{kind} command rejected"]
        if error_id is not None:
            details.append(f"error_id={error_id}")
            details.append(_format_error_details([error_id]))

        state = self.latest_state()
        if state.stamp > 0.0:
            details.append(
                f"robot_mode={state.robot_mode} {ROBOT_MODE_TEXT.get(state.robot_mode, '')}".strip()
            )
            details.append(f"enable_status={state.enable_status}")
            details.append(f"error_status={state.error_status}")
            if not state.enable_status:
                details.append("robot is not enabled; call /enable_robot first")
            if state.error_status:
                error_detail = self.get_error_id()
                if error_detail.message:
                    details.append(error_detail.message)
                if error_detail.raw_reply:
                    details.append(f"get_error_id={error_detail.raw_reply}")
        elif not raw_reply.strip():
            details.append("empty reply after reconnect")

        return "; ".join(details)

    def _teach_record_loop(self) -> None:
        period = (
            1.0 / self.config.teach_sample_rate_hz
            if self.config.teach_sample_rate_hz > 0
            else 0.2
        )
        last_point = None
        while True:
            with self._teach_lock:
                if not self._teach_recording:
                    return
                started_at = self._teach_started_at

            state = self.latest_state()
            if state.stamp >= started_at and len(state.joints) >= 6:
                point = {
                    "t": max(0.0, state.stamp - started_at),
                    "stamp": state.stamp,
                    "joints_deg": [float(value) for value in state.joints[:6]],
                    "tcp_pose": [float(value) for value in state.tcp_pose[:6]],
                }
                if self._should_record_teach_point(point, last_point):
                    with self._teach_lock:
                        if self._teach_recording:
                            self._teach_points.append(point)
                            last_point = point
            time.sleep(period)

    def _should_record_teach_point(self, point: dict, last_point: Optional[dict]) -> bool:
        if last_point is None:
            return True
        joint_delta = _max_abs_delta(point["joints_deg"], last_point["joints_deg"])
        tcp_delta = _max_abs_delta(point["tcp_pose"][:3], last_point["tcp_pose"][:3])
        return (
            joint_delta >= float(self.config.teach_min_joint_delta_deg)
            or tcp_delta >= float(self.config.teach_min_tcp_delta_mm)
        )

    def _teach_replay_mode(self, requested_mode: str) -> str:
        mode = str(requested_mode or self.config.teach_replay_mode or "movej")
        return mode.strip().lower()

    def _resample_teach_points(
        self,
        points: Sequence[dict],
        rate_hz: float,
    ) -> List[List[float]]:
        if not points:
            return []

        rate = float(rate_hz)
        if rate <= 0.0:
            raise ValueError("teach_servoj_rate_hz must be greater than 0")

        times = self._trajectory_times(points)
        total_time = times[-1] if times else 0.0
        if total_time <= 0.0 or len(points) == 1:
            return [[float(value) for value in points[0]["joints_deg"][:6]]]

        period = 1.0 / rate
        samples = []
        index = 0
        target_time = 0.0
        while target_time < total_time:
            while index + 1 < len(times) and times[index + 1] < target_time:
                index += 1
            samples.append(self._interpolate_joints(points, times, index, target_time))
            target_time += period

        samples.append([float(value) for value in points[-1]["joints_deg"][:6]])
        return samples

    def _trajectory_times(self, points: Sequence[dict]) -> List[float]:
        times = []
        monotonic = True
        for index, point in enumerate(points):
            value = float(point.get("t", 0.0))
            if index and value < times[-1]:
                monotonic = False
            times.append(value)

        if monotonic and times and times[-1] > 0.0:
            first = times[0]
            return [max(0.0, value - first) for value in times]

        period = (
            1.0 / self.config.teach_sample_rate_hz
            if self.config.teach_sample_rate_hz > 0
            else 0.2
        )
        return [index * period for index in range(len(points))]

    def _interpolate_joints(
        self,
        points: Sequence[dict],
        times: Sequence[float],
        index: int,
        target_time: float,
    ) -> List[float]:
        if index + 1 >= len(points):
            return [float(value) for value in points[-1]["joints_deg"][:6]]

        start_time = times[index]
        end_time = times[index + 1]
        if end_time <= start_time:
            ratio = 0.0
        else:
            ratio = (target_time - start_time) / (end_time - start_time)
        ratio = min(1.0, max(0.0, ratio))

        start = points[index]["joints_deg"]
        end = points[index + 1]["joints_deg"]
        return [
            float(start_value) + (float(end_value) - float(start_value)) * ratio
            for start_value, end_value in zip(start[:6], end[:6])
        ]

    def _send_servoj_samples(self, samples: Sequence[Sequence[float]]) -> int:
        if not samples:
            return 0

        rate = float(self.config.teach_servoj_rate_hz)
        if rate <= 0.0:
            raise ValueError("teach_servoj_rate_hz must be greater than 0")
        period = 1.0 / rate

        t = float(self.config.teach_servoj_t)
        lookahead_time = float(self.config.teach_servoj_lookahead_time)
        gain = float(self.config.teach_servoj_gain)
        self._validate_servoj_options(t, lookahead_time, gain)

        self.connect()
        sent_count = 0
        try:
            for joints in samples:
                command = self._build_servoj_command(joints, t, lookahead_time, gain)
                self._send_move_without_reply(command)
                sent_count += 1
                time.sleep(period)
        finally:
            try:
                self._reconnect_move_client()
            except Exception as exc:
                self._log(f"move channel reconnect after servoj replay failed: {exc}")
        return sent_count

    def _validate_servoj_options(
        self,
        t: float,
        lookahead_time: float,
        gain: float,
    ) -> None:
        if not 0.02 <= t <= 3600.0:
            raise ValueError("teach_servoj_t must be within 0.02..3600.0")
        if not 20.0 <= lookahead_time <= 100.0:
            raise ValueError("teach_servoj_lookahead_time must be within 20.0..100.0")
        if not 200.0 <= gain <= 1000.0:
            raise ValueError("teach_servoj_gain must be within 200.0..1000.0")

    def _build_servoj_command(
        self,
        joints: Sequence[float],
        t: float,
        lookahead_time: float,
        gain: float,
    ) -> str:
        return (
            f"ServoJ({_format_values(joints)},"
            f"t={t:.6f},lookahead_time={lookahead_time:.6f},gain={gain:.6f})"
        )

    def _send_move_without_reply(self, command: str) -> None:
        with self._command_lock:
            if self.move_client is None:
                raise RuntimeError("move client is not connected")
            socket_obj = getattr(self.move_client, "socket_dobot", None)
            if socket_obj in (None, 0):
                raise RuntimeError("move socket is not connected")
            socket_obj.sendall(str.encode(command, "utf-8"))

    def _save_trajectory(self, name: str, points: Sequence[dict], path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        data = {
            "name": name,
            "created_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
            "robot_model": self.config.robot_model,
            "sample_rate_hz": self.config.teach_sample_rate_hz,
            "joint_names": [f"joint{index}" for index in range(1, 7)],
            "joint_zero_deg": self.config.joint_zero_deg,
            "joint_lower_limits_deg": self.config.joint_lower_limits_deg,
            "joint_upper_limits_deg": self.config.joint_upper_limits_deg,
            "points": list(points),
        }
        with path.open("w", encoding="utf-8") as file:
            json.dump(data, file, ensure_ascii=False, indent=2)
            file.write("\n")

    def _load_trajectory(self, path: Path) -> dict:
        with path.open(encoding="utf-8") as file:
            data = json.load(file)
        points = data.get("points")
        if not isinstance(points, list):
            raise ValueError("trajectory points must be a list")
        for index, point in enumerate(points):
            joints = point.get("joints_deg") if isinstance(point, dict) else None
            if not isinstance(joints, list) or len(joints) < 6:
                raise ValueError(f"trajectory point {index} does not contain 6 joints")
        return data

    def _trajectory_dir(self) -> Path:
        return Path(self.config.teach_trajectory_dir).expanduser()

    def _trajectory_path(self, name: str) -> Path:
        filename = self._normalize_trajectory_name(name, generate=False) or name
        return self._trajectory_dir() / f"{filename}.json"

    def _normalize_trajectory_name(self, name: str, generate: bool = True) -> str:
        text = str(name or "").strip()
        if text.endswith(".json"):
            text = text[:-5]
        text = re.sub(r"[^A-Za-z0-9_.-]+", "_", text).strip("._")
        if text:
            return text
        if not generate:
            return ""
        return datetime.now(timezone.utc).strftime("teach_%Y%m%d_%H%M%S")

    def _log(self, message: str) -> None:
        if self.log_callback is not None:
            self.log_callback(message)
