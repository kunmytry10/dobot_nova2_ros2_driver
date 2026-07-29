import argparse
import json
import re
import socket
import time
from pathlib import Path
from typing import Iterable, List, Optional

from .controller import DashboardResult


def parse_error_ids(reply: str) -> List[int]:
    text = str(reply or "")
    match = re.search(r"\{(.*)\}", text, re.DOTALL)
    source = match.group(1) if match is not None else text
    return [int(value) for value in re.findall(r"-?\d+", source) if int(value) > 0]


def detect_limit_joint(error_ids: Iterable[int], files_dir: Path = None) -> Optional[int]:
    alarm_file = _alarm_file(files_dir)
    try:
        alarms = json.loads(alarm_file.read_text(encoding="utf-8"))
    except Exception:
        alarms = []
    wanted = {int(error_id) for error_id in error_ids}
    for alarm in alarms:
        if int(alarm.get("id", -1)) not in wanted:
            continue
        descriptions = [
            str(alarm.get("zh_CN", {}).get("description", "")),
            str(alarm.get("en", {}).get("description", "")),
        ]
        for description in descriptions:
            match = re.search(r"关节\s*([1-6])\s*[正负]向限位", description)
            if match:
                return int(match.group(1))
            match = re.search(r"Joint\s*([1-6])", description, re.IGNORECASE)
            if match and "limit" in description.lower():
                return int(match.group(1))
    return None


class LimitRecoveryManager:
    """Own the guarded prepare/release/lock lifecycle for one limit alarm."""

    def __init__(self, controller, release_timeout_sec: float = 12.0):
        self.controller = controller
        self.release_timeout_sec = float(release_timeout_sec)
        self.joint = 0
        self.brake_released = False
        self.released_at = 0.0

    @property
    def active(self) -> bool:
        return self.joint > 0

    def prepare(self) -> DashboardResult:
        if self.brake_released:
            return DashboardResult(False, message="a recovery brake is already released")
        if self.active:
            return DashboardResult(False, message="limit recovery is already prepared")
        self.controller.move_jog(stop=True)
        errors = self.controller.get_error_id()
        if not errors.success:
            return errors
        error_ids = [int(value) for value in errors.values]
        if not error_ids:
            return DashboardResult(False, message="no active limit alarm")
        unsupported = [value for value in error_ids if value < 64 or value > 75]
        if unsupported:
            return DashboardResult(
                False,
                message=f"non-limit alarms are active: {unsupported}",
                raw_reply=errors.raw_reply,
            )
        joints = {detect_limit_joint([value]) for value in error_ids}
        joints.discard(None)
        if len(joints) != 1:
            return DashboardResult(
                False,
                message=(
                    "limit recovery requires one unambiguous joint; "
                    f"alarms={error_ids}"
                ),
                raw_reply=errors.raw_reply,
            )
        joint = int(next(iter(joints)))
        disabled = self.controller.disable_robot()
        if not disabled.success:
            return disabled
        cleared = self.controller.clear_error()
        if not cleared.success:
            return cleared
        mode = self.controller.robot_mode()
        if not mode.success:
            return mode
        self.joint = joint
        return DashboardResult(
            True,
            0,
            f"limit recovery prepared for joint {joint}; robot disabled",
            " | ".join(
                [errors.raw_reply, disabled.raw_reply, cleared.raw_reply, mode.raw_reply]
            ),
        )

    def release(self) -> DashboardResult:
        joint = int(self.joint)
        if joint < 1 or joint > 6:
            return DashboardResult(False, message="limit recovery is not prepared")
        if self.brake_released:
            return DashboardResult(False, message=f"joint {joint} brake is already released")
        mode = self.controller.robot_mode()
        if not mode.success:
            return mode
        if mode.value not in {3, 4}:
            return DashboardResult(
                False,
                message=(
                    "brake release rejected: robot is not safely disabled; "
                    f"{mode.message}"
                ),
                raw_reply=mode.raw_reply,
            )
        result = self.controller.dashboard_command(
            f"BrakeControl({joint},1)", "limit_recovery_release"
        )
        if result.success:
            self.brake_released = True
            self.released_at = time.monotonic()
            result.message = f"joint {joint} brake released"
        return result

    def lock(self) -> DashboardResult:
        joint = int(self.joint)
        if joint < 1 or joint > 6:
            return DashboardResult(False, message="limit recovery is not prepared")
        if not self.brake_released:
            self.joint = 0
            return DashboardResult(
                True, 0, f"joint {joint} brake is locked; recovery cancelled"
            )
        result = self.controller.dashboard_command(
            f"BrakeControl({joint},0)", "limit_recovery_lock"
        )
        if not result.success:
            return result
        self.brake_released = False
        self.released_at = 0.0
        cleared = self.controller.clear_error()
        mode = self.controller.robot_mode()
        errors = self.controller.get_error_id()
        result.success = cleared.success and mode.success and errors.success
        result.message = (
            f"joint {joint} brake locked; robot remains disabled; {errors.message}"
        )
        result.raw_reply = " | ".join(
            [result.raw_reply, cleared.raw_reply, mode.raw_reply, errors.raw_reply]
        )
        self.joint = 0
        return result

    def watchdog(self) -> Optional[DashboardResult]:
        if not self.brake_released:
            return None
        if time.monotonic() - self.released_at < self.release_timeout_sec:
            return None
        result = self.lock()
        if not result.success:
            self.released_at = time.monotonic()
        return result


def _alarm_file(files_dir: Path = None) -> Path:
    if files_dir is not None:
        return Path(files_dir) / "alarm_controller.json"
    candidates = [Path(__file__).resolve().parents[1] / "files" / "alarm_controller.json"]
    try:
        from ament_index_python.packages import get_package_share_directory

        candidates.append(
            Path(get_package_share_directory("dobot_ros2"))
            / "files"
            / "alarm_controller.json"
        )
    except Exception:
        pass
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return candidates[0]


class DashboardClient:
    def __init__(self, robot_ip: str, port: int, timeout_sec: float):
        self.robot_ip = robot_ip
        self.port = int(port)
        self.timeout_sec = float(timeout_sec)

    def command(self, command: str, retry: int = 1) -> str:
        last_error = None
        for _ in range(max(1, int(retry) + 1)):
            try:
                with socket.create_connection(
                    (self.robot_ip, self.port), timeout=self.timeout_sec
                ) as sock:
                    sock.settimeout(self.timeout_sec)
                    sock.sendall(f"{command}\n".encode("utf-8"))
                    return sock.recv(4096).decode("utf-8", errors="replace")
            except OSError as exc:
                last_error = exc
        raise RuntimeError(f"{command} failed: {last_error}")


def _print_reply(label: str, reply: str) -> None:
    print(f"{label}: {reply!r}")


def _require_enter(message: str, assume_yes: bool) -> None:
    print(message)
    if not assume_yes:
        input("确认后按 Enter 继续；按 Ctrl+C 取消并保持/恢复抱闸。")


def recover_limit(args) -> int:
    client = DashboardClient(args.robot_ip, args.port, args.timeout_sec)
    print("== Dobot 关节限位抱死恢复向导 ==")
    print(f"Dashboard: {args.robot_ip}:{args.port}")
    errors_reply = client.command("GetErrorID()", retry=1)
    _print_reply("GetErrorID", errors_reply)
    error_ids = parse_error_ids(errors_reply)
    joint = int(args.joint) if int(args.joint) > 0 else detect_limit_joint(error_ids)
    if joint is None:
        print("没有自动识别到限位关节。请用 make recover-limit JOINT=1..6 手动指定。")
        return 2
    if joint < 1 or joint > 6:
        print(f"JOINT 必须是 1..6，当前是 {joint}")
        return 2

    print(f"将处理关节 {joint} 的限位恢复。")
    print("步骤 1/5: 禁用机器人，避免恢复过程中自动运动。")
    _print_reply("DisableRobot", client.command("DisableRobot()", retry=1))
    print("步骤 2/5: 清除当前报警。")
    _print_reply("ClearError", client.command("ClearError()", retry=1))
    print("步骤 3/5: 检查 Dashboard RobotMode。")
    _print_reply("RobotMode", client.command("RobotMode()", retry=1))

    released = False
    try:
        _require_enter(
            (
                f"步骤 4/5: 准备释放关节 {joint} 的抱闸。"
                "请确认人员、工具和线缆都不在机械臂运动路径内。"
            ),
            args.yes,
        )
        _print_reply(
            f"BrakeControl({joint},1)",
            client.command(f"BrakeControl({joint},1)", retry=1),
        )
        released = True
        _require_enter(
            (
                f"关节 {joint} 抱闸已释放。现在请手动把该关节移出限位位置。"
                "移动完成并确认机械臂稳定后继续。"
            ),
            args.yes,
        )
    finally:
        if released:
            print(f"步骤 5/5: 重新锁定关节 {joint} 抱闸。")
            try:
                _print_reply(
                    f"BrakeControl({joint},0)",
                    client.command(f"BrakeControl({joint},0)", retry=1),
                )
            except Exception as exc:
                print(f"重新抱闸失败，请立即人工确认机械臂安全状态: {exc}")
                raise

    print("最终检查:")
    _print_reply("ClearError", client.command("ClearError()", retry=1))
    _print_reply("RobotMode", client.command("RobotMode()", retry=1))
    _print_reply("GetErrorID", client.command("GetErrorID()", retry=1))
    print("恢复流程结束。确认姿态安全后，再手动执行 make enable 或在手柄上 enable。")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Interactive Dobot joint limit recovery.")
    parser.add_argument("--robot-ip", default="192.168.5.1")
    parser.add_argument("--port", type=int, default=29999)
    parser.add_argument("--joint", type=int, default=0)
    parser.add_argument("--timeout-sec", type=float, default=3.0)
    parser.add_argument("--yes", action="store_true")
    return parser


def main(argv=None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        return recover_limit(args)
    except KeyboardInterrupt:
        print("\n用户取消。若已经释放抱闸，程序会先尝试重新锁定。")
        return 130
    except Exception as exc:
        print(f"恢复失败: {exc}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
