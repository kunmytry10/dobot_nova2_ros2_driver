from types import SimpleNamespace

from dobot_joy.joy_common import JoyMapping
from dobot_joy.joy_teleop import JoyTeleopNode


class _Logger:
    def info(self, _message):
        pass


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
