import sys
import socket

import numpy as np
from pathlib import Path
from types import SimpleNamespace

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
# Tests are run from the workspace root before the package is installed.
sys.path.insert(0, str(PACKAGE_ROOT))

from dobot_ros2.controller import (  # noqa: E402
    ControllerConfig,
    DashboardResult,
    DobotController,
    MotionResult,
    _format_error_details,
)
from dobot_ros2.gripper import DhAgGripper, DobotModbusAgGripper, GripperConfig, _crc  # noqa: E402
from dobot_api import DobotApiFeedBack, MyType  # noqa: E402


def test_four_motion_services_are_registered_with_expected_names():
    driver_source = (PACKAGE_ROOT / "dobot_ros2" / "driver_node.py").read_text()

    assert 'create_service(MoveCommand, "movej"' in driver_source
    assert 'create_service(MoveCommand, "movel"' in driver_source
    assert 'create_service(MoveCommand, "movep"' in driver_source
    assert 'create_service(MoveCommand, "movejp"' in driver_source
    assert 'create_service(MoveCommand, "joint_movej"' not in driver_source


def test_emergency_stop_service_is_registered():
    driver_source = (PACKAGE_ROOT / "dobot_ros2" / "driver_node.py").read_text()

    assert 'create_service(Trigger, "emergency_stop"' in driver_source
    assert "self.controller.emergency_stop()" in driver_source


def test_drag_services_are_registered_without_trajectory_recording():
    driver_source = (PACKAGE_ROOT / "dobot_ros2" / "driver_node.py").read_text()
    controller_source = (PACKAGE_ROOT / "dobot_ros2" / "controller.py").read_text()

    assert 'create_service(Trigger, "drag_start"' in driver_source
    assert 'create_service(Trigger, "drag_stop"' in driver_source
    assert "self.controller.drag_start()" in driver_source
    assert "self.controller.drag_stop()" in driver_source
    assert 'return self._dashboard_command("StartDrag()", "drag_start")' in controller_source
    assert 'return self._dashboard_command("StopDrag()", "drag_stop")' in controller_source


def test_move_jog_service_is_registered():
    driver_source = (PACKAGE_ROOT / "dobot_ros2" / "driver_node.py").read_text()
    interfaces_source = (PACKAGE_ROOT.parent / "dobot_interfaces" / "CMakeLists.txt").read_text()

    assert 'create_service(JogCommand, "move_jog"' in driver_source
    assert "create_publisher(" in driver_source
    assert "String, \"move_jog/diagnostics\", 10" in driver_source
    assert '"srv/JogCommand.srv"' in interfaces_source
    assert "self.controller.move_jog(" in driver_source


def test_move_jog_maps_to_move_port_command():
    controller = DobotController(ControllerConfig())
    calls = []

    def fake_send_move(command, timeout_sec):
        calls.append((command, timeout_sec))
        return f"0,{{}},{command};"

    controller.connect = lambda: None
    controller._send_move_with_reconnect = fake_send_move

    result = controller.move_jog("X+", coord_type=0, user=0, tool=0)
    stop = controller.move_jog("", stop=True)

    assert result.success
    assert stop.success
    assert calls[0][0] == "MoveJog(X+,CoordType=0,User=0,Tool=0)"
    assert calls[1][0] == "MoveJog()"


def test_servo_p_maps_to_documented_command_and_drains_reply():
    controller = DobotController(ControllerConfig())
    calls = []
    controller.connect = lambda: None
    controller._send_move_streaming = lambda command: (
        calls.append(command) or f"0,{{}},{command};"
    )

    result = controller.servo_p([100, 200, 300, 10, 20, 30])

    assert result.success
    assert calls == [
        "ServoP(100.000000,200.000000,300.000000,10.000000,20.000000,30.000000)"
    ]


def test_servo_p_accepts_documented_no_reply_behavior():
    controller = DobotController(ControllerConfig())
    controller.connect = lambda: None
    controller._send_move_streaming = lambda command: ""

    result = controller.servo_p([100, 200, 300, 10, 20, 30])

    assert result.success
    assert result.raw_reply == ""


def test_streaming_send_drains_replies_without_waiting():
    class FakeSocket:
        def __init__(self):
            self.timeout = None
            self.replies = [
                b"0,{},previous;",
                BlockingIOError(),
                b"0,{},current;",
                BlockingIOError(),
            ]
            self.sent = []

        def gettimeout(self):
            return self.timeout

        def settimeout(self, timeout):
            self.timeout = timeout

        def setblocking(self, blocking):
            self.timeout = None if blocking else 0.0

        def recv(self, size):
            del size
            value = self.replies.pop(0)
            if isinstance(value, Exception):
                raise value
            return value

        def sendall(self, data):
            self.sent.append(data)

    controller = DobotController(ControllerConfig())
    socket_obj = FakeSocket()
    controller.move_client = SimpleNamespace(socket_dobot=socket_obj)

    reply = controller._send_move_streaming("ServoP(1,2,3,4,5,6)")

    assert reply == "0,{},previous;0,{},current;"
    assert socket_obj.sent == [b"ServoP(1,2,3,4,5,6)"]
    assert socket_obj.timeout is None
    assert controller._move_channel_streaming


def test_end_servo_stream_reconnects_move_channel():
    controller = DobotController(ControllerConfig())
    calls = []
    controller._reconnect_move_client = lambda: calls.append("reconnect")

    controller.end_servo_stream()

    assert calls == ["reconnect"]


def test_move_socket_disables_nagle_for_servo_streaming():
    class FakeSocket:
        def __init__(self):
            self.options = []

        def setsockopt(self, level, option, value):
            self.options.append((level, option, value))

    socket_obj = FakeSocket()
    controller = DobotController(ControllerConfig())

    controller._configure_move_client(SimpleNamespace(socket_dobot=socket_obj))

    assert socket_obj.options == [(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)]


def test_dashboard_and_move_channels_use_independent_command_locks():
    controller = DobotController(ControllerConfig())
    calls = []
    controller.dashboard = object()
    controller.move_client = object()
    controller._send = lambda client, command, timeout, lock: calls.append(
        (client, command, timeout, lock)
    ) or "0,{},ok;"

    controller._send_dashboard("RobotMode()", 1.0)
    controller._send_move("MoveJog()", 2.0)

    assert controller._dashboard_command_lock is not controller._move_command_lock
    assert calls == [
        (
            controller.dashboard,
            "RobotMode()",
            1.0,
            controller._dashboard_command_lock,
        ),
        (
            controller.move_client,
            "MoveJog()",
            2.0,
            controller._move_command_lock,
        ),
    ]


def test_feedback_reader_reassembles_fragmented_tcp_packet():
    class FakeSocket:
        def __init__(self, chunks):
            self.chunks = list(chunks)

        def setblocking(self, blocking):
            assert blocking

        def recv(self, size):
            del size
            return self.chunks.pop(0)

        def close(self):
            pass

    packet = np.zeros(1, dtype=MyType)
    packet["robot_mode"] = 5
    payload = packet.tobytes()
    client = DobotApiFeedBack.__new__(DobotApiFeedBack)
    client.socket_dobot = FakeSocket([payload[:137], payload[137:]])
    client._feedback_buffer = bytearray()

    result = client.feedBackData()

    assert int(result["robot_mode"][0]) == 5
    assert client._feedback_buffer == bytearray()


def test_feedback_reader_preserves_additional_complete_packet():
    class FakeSocket:
        def __init__(self, payload):
            self.payload = payload
            self.recv_count = 0

        def setblocking(self, blocking):
            assert blocking

        def recv(self, size):
            del size
            self.recv_count += 1
            if self.recv_count > 1:
                raise AssertionError("buffered packet should not read the socket again")
            return self.payload

        def close(self):
            pass

    first = np.zeros(1, dtype=MyType)
    first["robot_mode"] = 5
    second = np.zeros(1, dtype=MyType)
    second["robot_mode"] = 7
    client = DobotApiFeedBack.__new__(DobotApiFeedBack)
    client.socket_dobot = FakeSocket(first.tobytes() + second.tobytes())
    client._feedback_buffer = bytearray()

    assert int(client.feedBackData()["robot_mode"][0]) == 5
    assert int(client.feedBackData()["robot_mode"][0]) == 7
    assert client.socket_dobot.recv_count == 1


def test_normal_servo_stop_keeps_channel_for_immediate_resume():
    controller = DobotController(ControllerConfig())
    calls = []
    controller._move_channel_streaming = True
    controller._reconnect_move_client = lambda: calls.append("reconnect")

    controller.end_servo_stream(reset_channel=False)

    assert calls == []
    assert controller._move_channel_streaming


def test_request_reply_command_resets_streaming_channel_once():
    controller = DobotController(ControllerConfig())
    calls = []
    controller._move_channel_streaming = True

    def fake_reconnect():
        calls.append("reconnect")
        controller._move_channel_streaming = False

    controller._reconnect_move_client = fake_reconnect
    controller._send_move = lambda command, timeout: calls.append(
        (command, timeout)
    ) or "0,{},MoveJog();"

    reply = controller._send_move_with_reconnect("MoveJog()", 1.0)

    assert reply == "0,{},MoveJog();"
    assert calls == ["reconnect", ("MoveJog()", 1.0)]


def test_emergency_stop_maps_to_dashboard_command():
    controller = DobotController(ControllerConfig())
    calls = []

    def fake_dashboard_command(command, label):
        calls.append((command, label))
        return DashboardResult(True, error_id=0, message="accepted")

    controller._dashboard_command = fake_dashboard_command

    result = controller.emergency_stop()

    assert result.success
    assert calls == [("EmergencyStop()", "emergency_stop")]


def test_teach_services_are_registered():
    driver_source = (PACKAGE_ROOT / "dobot_ros2" / "driver_node.py").read_text()
    interfaces_source = (PACKAGE_ROOT.parent / "dobot_interfaces" / "CMakeLists.txt").read_text()

    for service_name in (
        "teach_start",
        "teach_stop",
        "teach_replay",
        "teach_delete",
        "teach_list",
        "teach_status",
    ):
        assert service_name in driver_source

    assert '"srv/TrajectoryCommand.srv"' in interfaces_source
    assert '"srv/TrajectoryList.srv"' in interfaces_source
    assert "string replay_mode" in (
        PACKAGE_ROOT.parent / "dobot_interfaces" / "srv" / "TrajectoryCommand.srv"
    ).read_text()


def test_gripper_services_are_registered():
    driver_source = (PACKAGE_ROOT / "dobot_ros2" / "driver_node.py").read_text()
    interfaces_source = (PACKAGE_ROOT.parent / "dobot_interfaces" / "CMakeLists.txt").read_text()
    config_source = (PACKAGE_ROOT / "config" / "dobot_ros2.yaml").read_text()

    assert 'create_service(Trigger, "gripper_init"' in driver_source
    assert 'create_service(GripperCommand, "gripper_move"' in driver_source
    assert 'create_service(GripperState, "get_gripper_state"' in driver_source
    assert "DobotModbusAgGripper" in driver_source
    assert 'create_publisher(DobotState, "dobot_state"' in driver_source
    assert 'create_publisher(GripperStatus, "gripper_state"' in driver_source
    assert '"srv/GripperCommand.srv"' in interfaces_source
    assert '"srv/GripperState.srv"' in interfaces_source
    assert '"msg/DobotState.msg"' in interfaces_source
    assert '"msg/GripperStatus.msg"' in interfaces_source
    assert 'gripper_transport: "dobot_modbus"' in config_source
    assert "gripper_state_rate_hz: 2.0" in config_source
    assert "gripper_modbus_port: 60000" in config_source
    assert "gripper_stroke_mm: 95.0" in config_source
    assert "gripper_max_force_n: 160.0" in config_source
    assert "float64 force_n" in (
        PACKAGE_ROOT.parent / "dobot_interfaces" / "srv" / "GripperCommand.srv"
    ).read_text()


def test_gripper_modbus_crc_matches_manual_example():
    assert _crc(bytes.fromhex("01 06 01 00 00 01")) == bytes.fromhex("49 F6")


def test_gripper_position_and_force_mapping_for_ag_160_95():
    gripper = DhAgGripper(GripperConfig(stroke_mm=95.0, max_force_n=160.0))

    assert gripper._target_permille(47.5, -1) == 500
    assert gripper._target_permille(-1.0, 250) == 250
    assert gripper._permille_to_mm(1000) == 95.0
    assert gripper._target_force_percent(-1, 80.0) == 50
    assert gripper._target_force_percent(30, -1.0) == 30
    assert gripper._target_force_percent(-1, 5.0) == 20


def test_gripper_dobot_modbus_commands_match_tcp_api():
    calls = []

    def fake_command(command, label):
        calls.append((command, label))
        if command.startswith("ModbusCreate"):
            return DashboardResult(True, error_id=0, raw_reply="0,{0},ModbusCreate();")
        if command.startswith("GetHoldRegs"):
            if ",512," in command:
                return DashboardResult(True, error_id=0, raw_reply="0,{1},GetHoldRegs();")
            if ",513," in command:
                return DashboardResult(True, error_id=0, raw_reply="0,{2},GetHoldRegs();")
            if ",514," in command:
                return DashboardResult(True, error_id=0, raw_reply="0,{500},GetHoldRegs();")
            if ",257," in command:
                return DashboardResult(True, error_id=0, raw_reply="0,{50},GetHoldRegs();")
        return DashboardResult(True, error_id=0, raw_reply="0,{},SetHoldRegs();")

    gripper = DobotModbusAgGripper(
        GripperConfig(enabled=True, transport="dobot_modbus"),
        fake_command,
    )
    result = gripper.move(47.5, -1, -1, 80.0, wait=False, timeout_sec=1.0)

    assert result.success
    assert calls[0][0] == "ModbusCreate(127.0.0.1,60000,1,1)"
    assert ("SetHoldRegs(0,257,1,{50},U16)", "gripper_modbus_write") in calls
    assert ("SetHoldRegs(0,259,1,{500},U16)", "gripper_modbus_write") in calls
    assert ("GetHoldRegs(0,512,1,U16)", "gripper_modbus_read") in calls
    assert result.object_detected


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


def test_movej_joint_limit_check_rejects_out_of_range_target():
    controller = DobotController(ControllerConfig())
    result = controller._check_joint_limits("movej", [0, 0, 157, 0, 0, 0], [0] * 6)

    assert not result.success
    assert "joint3=157.000" in result.message
    assert "[-156.000,156.000]" in result.message


def test_joint_limit_check_can_be_disabled():
    controller = DobotController(ControllerConfig(joint_limit_check=False))
    result = controller._check_joint_limits("movej", [0, 0, 157, 0, 0, 0], [0] * 6)

    assert result.success
    assert result.message == "joint limit check disabled"


def test_trajectory_name_is_sanitized_and_timestamped():
    controller = DobotController(ControllerConfig())

    assert controller._normalize_trajectory_name("../demo 1.json") == "demo_1"
    assert controller._normalize_trajectory_name("", generate=False) == ""
    assert controller._normalize_trajectory_name("").startswith("teach_")


def test_trajectory_save_and_list(tmp_path):
    controller = DobotController(ControllerConfig(teach_trajectory_dir=str(tmp_path)))
    path = controller._trajectory_path("demo")
    points = [
        {
            "t": 0.0,
            "stamp": 1.0,
            "joints_deg": [0, 1, 2, 3, 4, 5],
            "tcp_pose": [100, 200, 300, 0, 0, 0],
        }
    ]

    controller._save_trajectory("demo", points, path)
    results = controller.teach_list()

    assert path.exists()
    assert len(results) == 1
    assert results[0].trajectory_name == "demo"
    assert results[0].point_count == 1


def test_servoj_command_mapping_matches_documented_options():
    controller = DobotController(ControllerConfig())

    assert controller._build_servoj_command([1, 2, 3, 4, 5, 6], 0.1, 50.0, 500.0) == (
        "ServoJ(1.000000,2.000000,3.000000,4.000000,5.000000,6.000000,"
        "t=0.100000,lookahead_time=50.000000,gain=500.000000)"
    )


def test_servoj_resampling_interpolates_recorded_points():
    controller = DobotController(ControllerConfig())
    points = [
        {"t": 0.0, "joints_deg": [0, 0, 0, 0, 0, 0]},
        {"t": 1.0, "joints_deg": [10, 20, 30, 40, 50, 60]},
    ]

    samples = controller._resample_teach_points(points, 2.0)

    assert samples == [
        [0, 0, 0, 0, 0, 0],
        [5, 10, 15, 20, 25, 30],
        [10, 20, 30, 40, 50, 60],
    ]


def test_teach_replay_can_use_servoj_mode(tmp_path):
    controller = DobotController(ControllerConfig(teach_trajectory_dir=str(tmp_path)))
    path = controller._trajectory_path("demo")
    points = [
        {
            "t": 0.0,
            "stamp": 1.0,
            "joints_deg": [0, 0, 0, 0, 0, 0],
            "tcp_pose": [100, 200, 300, 0, 0, 0],
        },
        {
            "t": 1.0,
            "stamp": 2.0,
            "joints_deg": [1, 2, 3, 4, 5, 6],
            "tcp_pose": [101, 201, 301, 0, 0, 0],
        },
    ]
    controller._save_trajectory("demo", points, path)
    sent = []

    def fake_move(*args, **kwargs):
        del args, kwargs
        return MotionResult(True, error_id=0, message="accepted", raw_reply="0,{},JointMovJ();")

    def fake_send_servoj_samples(samples):
        sent.extend(samples)
        return len(samples)

    controller.move = fake_move
    controller._send_servoj_samples = fake_send_servoj_samples

    result = controller.teach_replay("demo", replay_mode="servoj")

    assert result.success
    assert "servoj replay sent" in result.message
    assert sent


def test_motion_error_details_decode_alarm_and_tcp_codes():
    assert "规划位置接近肘奇异点" in _format_error_details([27])
    assert "命令不存在" in _format_error_details([-10000])
    assert "参数数量错误" in _format_error_details([-20000])


def test_movep_command_failure_message_includes_decoded_alarm():
    controller = DobotController(ControllerConfig())
    message = controller._motion_failure_message("movep", 27, "27,{},JointMovJ(...);")

    assert "movep command rejected" in message
    assert "规划位置接近肘奇异点" in message
