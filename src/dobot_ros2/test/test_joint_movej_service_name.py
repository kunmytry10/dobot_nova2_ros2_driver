import sys
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
# Tests are run from the workspace root before the package is installed.
sys.path.insert(0, str(PACKAGE_ROOT))

from dobot_ros2.controller import ControllerConfig, DobotController, _format_error_details  # noqa: E402


def test_four_motion_services_are_registered_with_expected_names():
    driver_source = (PACKAGE_ROOT / "dobot_ros2" / "driver_node.py").read_text()

    assert 'create_service(MoveCommand, "movej"' in driver_source
    assert 'create_service(MoveCommand, "movel"' in driver_source
    assert 'create_service(MoveCommand, "movep"' in driver_source
    assert 'create_service(MoveCommand, "movejp"' in driver_source
    assert 'create_service(MoveCommand, "joint_movej"' not in driver_source


def test_motion_command_mapping_matches_ros_abstraction():
    controller = DobotController(ControllerConfig())
    joints = [1, 2, 3, 4, 5, 6]
    pose = [100, 200, 300, 10, 20, 30]

    assert controller._build_motion_command("movej", joints, 0, 0, 5, 6) == (
        "JointMovJ(1.000000,2.000000,3.000000,4.000000,5.000000,6.000000,"
        "SpeedJ=5,AccJ=6)"
    )
    assert controller._build_motion_command("movejp", pose, 0, 0, 5, 6) == (
        "MovJ(100.000000,200.000000,300.000000,10.000000,20.000000,30.000000,"
        "User=0,Tool=0,SpeedJ=5,AccJ=6)"
    )
    assert controller._build_motion_command("movel", pose, 0, 0, 5, 6) == (
        "MovL(100.000000,200.000000,300.000000,10.000000,20.000000,30.000000,"
        "User=0,Tool=0,SpeedL=5,AccL=6)"
    )
    assert controller._build_motion_command("movep", joints, 0, 0, 5, 6) == (
        "JointMovJ(1.000000,2.000000,3.000000,4.000000,5.000000,6.000000,"
        "SpeedJ=5,AccJ=6)"
    )


def test_movep_uses_ik_result_and_jointmovj_not_movp():
    controller_source = (PACKAGE_ROOT / "dobot_ros2" / "controller.py").read_text()

    assert (
        "command_target = ik_result.ik_joints if kind == \"movep\" else target"
        in controller_source
    )
    assert 'return self._command("JointMovJ", target, options)' in controller_source
    assert 'self.config.movep_command' not in controller_source


def test_only_movej_skips_inverse_kinematics():
    controller = DobotController(ControllerConfig())
    result = controller._check_ik("movej", [1, 2, 3, 4, 5, 6], 0, 0)

    assert result.success
    assert result.message == "joint target does not need IK"


def test_ik_check_parameter_can_disable_cartesian_precheck():
    controller = DobotController(ControllerConfig(ik_check=False))
    result = controller._check_ik("movel", [100, 200, 300, 10, 20, 30], 0, 0)

    assert result.success
    assert result.message == "inverse kinematics check disabled"


def test_motion_error_details_decode_alarm_and_tcp_codes():
    assert "规划位置接近肘奇异点" in _format_error_details([27])
    assert "命令不存在" in _format_error_details([-10000])
    assert "参数数量错误" in _format_error_details([-20000])


def test_movep_command_failure_message_includes_decoded_alarm():
    controller = DobotController(ControllerConfig())
    message = controller._motion_failure_message("movep", 27, "27,{},JointMovJ(...);")

    assert "movep command rejected" in message
    assert "规划位置接近肘奇异点" in message
