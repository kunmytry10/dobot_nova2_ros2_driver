from types import SimpleNamespace

from builtin_interfaces.msg import Time

from dobot_joy.joy_common import JoyMapping
from dobot_joy.joy_teleop import JoyTeleopNode


class _Logger:
    def info(self, _message):
        pass

    def error(self, _message):
        pass

    def warn(self, _message):
        pass


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def test_gripper_target_initializes_once_from_feedback():
    node = SimpleNamespace(
        gripper_target_mm=None,
        gripper_min_opening_mm=0.0,
        gripper_max_opening_mm=95.0,
        get_logger=lambda: _Logger(),
    )

    JoyTeleopNode._initialize_gripper_target(node, 95.0)
    assert node.gripper_target_mm == 95.0

    JoyTeleopNode._initialize_gripper_target(node, 20.0)
    assert node.gripper_target_mm == 95.0


def test_action_is_not_published_before_gripper_feedback():
    node = SimpleNamespace(gripper_target_mm=None)

    assert JoyTeleopNode._publish_action(node) is None


def test_idle_trigger_axes_preserve_discrete_gripper_target_action():
    node = SimpleNamespace(
        trigger_neutral_axes=[0.0] * 6,
        mapping=JoyMapping(),
        active_gripper_axis=None,
        gripper_action="close",
    )

    JoyTeleopNode._handle_gripper_axis(node, [0.0] * 6)

    assert node.gripper_action == "close"


def test_busy_gripper_keeps_latest_command_instead_of_dropping_it():
    diagnostics = []
    node = SimpleNamespace(
        gripper_busy=True,
        gripper_move_pending=None,
        latest_gripper_opening_mm=95.0,
        gripper_target_mm=95.0,
        _publish_diagnostic=diagnostics.append,
    )

    JoyTeleopNode._move_gripper(node, 0.0, "gripper toggle", requested_at=12.0)

    assert node.gripper_move_pending == (0.0, "gripper toggle", 12.0)
    assert node.latest_gripper_opening_mm == 0.0
    assert node.gripper_target_mm == 0.0
    assert diagnostics[-1]["event"] == "gripper_command_queued"


def test_recording_start_syncs_target_after_external_auto_return():
    node = SimpleNamespace(
        latest_gripper_opening_mm=95.0,
        gripper_min_opening_mm=0.0,
        gripper_max_opening_mm=95.0,
        gripper_target_mm=0.0,
        gripper_action="close",
    )

    JoyTeleopNode._sync_gripper_target_for_recording(node)

    assert node.gripper_target_mm == 95.0
    assert node.gripper_action == "hold"


def test_published_gripper_action_is_always_binary():
    publisher = _Publisher()
    node = SimpleNamespace(
        gripper_target_mm=52.0,
        gripper_min_opening_mm=0.0,
        gripper_max_opening_mm=95.0,
        gripper_toggle_threshold_mm=45.0,
        current_axis=None,
        current_cartesian_action=[0.0] * 6,
        control_mode="servo_p",
        teleop_deadman=False,
        coord_type=0,
        user=0,
        tool=0,
        gripper_action="hold",
        action_pub=publisher,
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(to_msg=lambda: Time())
        ),
    )

    JoyTeleopNode._publish_action(node)
    assert publisher.messages[-1].gripper_target_mm == 95.0
    assert publisher.messages[-1].gripper_target_normalized == 1.0

    node.gripper_target_mm = 20.0
    JoyTeleopNode._publish_action(node)
    assert publisher.messages[-1].gripper_target_mm == 0.0
    assert publisher.messages[-1].gripper_target_normalized == 0.0


def test_recording_trigger_release_does_not_create_intermediate_stop():
    moves = []
    node = SimpleNamespace(
        trigger_neutral_axes=[0.0] * 6,
        mapping=JoyMapping(),
        active_gripper_axis=None,
        collection_recording=True,
        gripper_action="hold",
        gripper_stop_pending=False,
        gripper_min_opening_mm=0.0,
        gripper_max_opening_mm=95.0,
        last_gripper_command_time=0.0,
        gripper_command_period_sec=0.0,
        _move_gripper=lambda target, label, allow_busy=False: moves.append(
            (target, label, allow_busy)
        ),
        _stop_gripper_motion=lambda _direction: (_ for _ in ()).throw(
            AssertionError("recording release must not stop mid-stroke")
        ),
    )
    close_axes = [0.0] * 6
    close_axes[node.mapping.lt_axis_index] = 1.0

    JoyTeleopNode._handle_gripper_axis(node, close_axes)
    JoyTeleopNode._handle_gripper_axis(node, [0.0] * 6)

    assert moves == [(0.0, "gripper jog close", True)]
    assert node.active_gripper_axis is None
    assert node.gripper_action == "hold"


def test_enable_toggle_is_locked_while_data_collection_is_active():
    diagnostics = []
    node = SimpleNamespace(
        collection_recording=True,
        _stop_jog=lambda **_kwargs: None,
        get_logger=lambda: _Logger(),
        _start_rumble_pattern=lambda *_args: None,
        _publish_diagnostic=diagnostics.append,
    )

    JoyTeleopNode._toggle_enable(node)

    assert diagnostics == [
        {"event": "enable_toggle_rejected", "reason": "data_collection_active"}
    ]
