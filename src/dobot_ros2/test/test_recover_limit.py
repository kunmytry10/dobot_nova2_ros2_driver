import sys
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_ROOT = PACKAGE_ROOT.parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from dobot_ros2.recover_limit import (  # noqa: E402
    LimitRecoveryManager,
    detect_limit_joint,
    parse_error_ids,
)
from dobot_ros2.controller import DashboardResult  # noqa: E402


class FakeController:
    def __init__(self):
        self.commands = []

    def move_jog(self, stop=False):
        self.commands.append(("move_jog", bool(stop)))
        return DashboardResult(True, 0)

    def get_error_id(self):
        return DashboardResult(True, 0, "limit", "errors", values=[75])

    def disable_robot(self):
        self.commands.append(("disable",))
        return DashboardResult(True, 0, raw_reply="disabled")

    def clear_error(self):
        self.commands.append(("clear",))
        return DashboardResult(True, 0, raw_reply="cleared")

    def robot_mode(self):
        return DashboardResult(True, 0, "disabled", "mode", value=4)

    def dashboard_command(self, command, label):
        self.commands.append((label, command))
        return DashboardResult(True, 0, raw_reply=command)


def test_parse_error_ids_from_dashboard_reply():
    reply = "0,{[[75],[],[],[],[],[],[]]},GetErrorID();"

    assert parse_error_ids(reply) == [75]


def test_detect_limit_joint_from_alarm_description():
    assert detect_limit_joint([75], PACKAGE_ROOT / "files") == 6
    assert detect_limit_joint([69], PACKAGE_ROOT / "files") == 3
    assert detect_limit_joint([], PACKAGE_ROOT / "files") is None


def test_limit_recovery_manager_requires_prepare_and_can_cancel_safely():
    controller = FakeController()
    manager = LimitRecoveryManager(controller)

    assert not manager.release().success
    assert manager.prepare().success
    assert manager.joint == 6
    assert manager.active
    assert manager.lock().success
    assert not manager.active
    assert not manager.brake_released
    assert not any("BrakeControl" in str(command) for command in controller.commands)


def test_limit_recovery_manager_releases_and_locks_only_detected_joint():
    controller = FakeController()
    manager = LimitRecoveryManager(controller)

    assert manager.prepare().success
    assert manager.release().success
    assert manager.brake_released
    assert manager.lock().success
    assert not manager.brake_released
    assert not manager.active
    assert ("limit_recovery_release", "BrakeControl(6,1)") in controller.commands
    assert ("limit_recovery_lock", "BrakeControl(6,0)") in controller.commands


def test_makefile_exposes_interactive_limit_recovery():
    source = (WORKSPACE_ROOT / "Makefile").read_text()

    assert "recover-limit:" in source
    assert "ROBOT_IP ?= 192.168.5.1" in source
    assert "DASHBOARD_PORT ?= 29999" in source
    assert "ros2 run dobot_ros2 dobot_recover_limit" in source
    assert "--joint \"$(JOINT)\"" in source


def test_driver_exposes_guarded_limit_recovery_service():
    driver = (PACKAGE_ROOT / "dobot_ros2" / "driver_node.py").read_text()
    recovery = (PACKAGE_ROOT / "dobot_ros2" / "recover_limit.py").read_text()
    interfaces = (PACKAGE_ROOT.parent / "dobot_interfaces" / "CMakeLists.txt").read_text()

    assert 'create_service(LimitRecovery, "limit_recovery"' in driver
    assert 'request.action' in driver
    assert "LimitRecoveryManager" in driver
    assert 'f"BrakeControl({joint},1)"' in recovery
    assert 'f"BrakeControl({joint},0)"' in recovery
    assert "_limit_recovery_watchdog" in driver
    assert '"srv/LimitRecovery.srv"' in interfaces
