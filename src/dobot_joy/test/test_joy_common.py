import sys
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from dobot_joy.joy_common import (  # noqa: E402
    JoyMapping,
    axis_to_gripper_delta,
    axis_to_jog,
    button_pressed,
    deadman_pressed,
)


def test_deadman_pressed_requires_configured_button():
    assert deadman_pressed([0, 0, 0, 0, 1], 4)
    assert not deadman_pressed([0, 0, 0, 0, 0], 4)
    assert deadman_pressed([], -1)


def test_axis_to_jog_uses_dominant_axis_and_deadzone():
    mapping = JoyMapping()

    assert axis_to_jog([0.0, -0.7, 0.0, 0.0, 0.0], mapping) == "X+"
    assert axis_to_jog([0.0, 0.7, 0.0, 0.0, 0.0], mapping) == "X-"
    assert axis_to_jog([0.8, 0.2, 0.0, 0.0, 0.0], mapping) == "Y-"
    assert axis_to_jog([-0.8, 0.2, 0.0, 0.0, 0.0], mapping) == "Y+"
    assert axis_to_jog([0.0, 0.0, 0.0, 0.0, -0.9], mapping) == "Z-"
    assert axis_to_jog([0.0, 0.0, 0.0, -0.9, 0.0], mapping) == "Rz+"
    assert axis_to_jog([0.1, 0.1, 0.0, 0.0, 0.0], mapping) is None


def test_axis_to_jog_can_disable_motion_until_deadman_is_handled_by_node():
    mapping = JoyMapping()

    assert deadman_pressed([0, 0, 0, 0, 1], 4)
    assert not deadman_pressed([0, 0, 0, 0, 0], 4)
    assert axis_to_jog([0.0, -1.0, 0.0, 0.0, 0.0], mapping) == "X+"


def test_axis_to_gripper_delta_uses_triggers_with_deadzone():
    mapping = JoyMapping(gripper_step_mm=2.0)

    assert axis_to_gripper_delta([0.0, 0.0, 0.8, 0.0, 0.0, 0.0], mapping) == -2.0
    assert axis_to_gripper_delta([0.0, 0.0, 0.0, 0.0, 0.0, 0.9], mapping) == 2.0
    assert axis_to_gripper_delta([0.0, 0.0, 0.9, 0.0, 0.0, 0.7], mapping) == -2.0
    assert axis_to_gripper_delta([0.0, 0.0, 0.1, 0.0, 0.0, 0.0], mapping) is None


def test_button_pressed_handles_missing_or_disabled_indices():
    assert button_pressed([1, 0], 0)
    assert not button_pressed([1, 0], 1)
    assert not button_pressed([1, 0], 3)
    assert not button_pressed([1, 0], -1)
