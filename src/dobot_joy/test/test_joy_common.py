import sys
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from dobot_joy.joy_common import (  # noqa: E402
    JoyMapping,
    axis_to_gripper_delta,
    axis_to_jog,
    button_pressed,
    deadman_pressed,
    gripper_stop_target,
    jog_axis_to_action,
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
    assert axis_to_jog([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0], mapping) == "Rx+"
    assert axis_to_jog([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0], mapping) == "Rx-"
    assert axis_to_jog([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0], mapping) == "Ry+"
    assert axis_to_jog([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0], mapping) == "Ry-"
    assert axis_to_jog([0.1, 0.1, 0.0, 0.0, 0.0], mapping) is None


def test_axis_to_jog_can_disable_motion_until_deadman_is_handled_by_node():
    mapping = JoyMapping()

    assert deadman_pressed([0, 0, 0, 0, 1], 4)
    assert not deadman_pressed([0, 0, 0, 0, 0], 4)
    assert axis_to_jog([0.0, -1.0, 0.0, 0.0, 0.0], mapping) == "X+"


def test_jog_axis_to_action_uses_training_vector_order():
    assert jog_axis_to_action("X+") == [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    assert jog_axis_to_action("Z-") == [0.0, 0.0, -1.0, 0.0, 0.0, 0.0]
    assert jog_axis_to_action("Rx+") == [0.0, 0.0, 0.0, 1.0, 0.0, 0.0]
    assert jog_axis_to_action("Rz-") == [0.0, 0.0, 0.0, 0.0, 0.0, -1.0]
    assert jog_axis_to_action(None) == [0.0] * 6
    assert jog_axis_to_action("invalid") == [0.0] * 6


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


def test_gripper_stop_target_leads_in_motion_direction_to_avoid_rebound():
    assert gripper_stop_target(40.0, "close", 3.0, 0.0, 95.0) == 37.0
    assert gripper_stop_target(40.0, "open", 3.0, 0.0, 95.0) == 43.0
    assert gripper_stop_target(1.0, "close", 3.0, 0.0, 95.0) == 0.0
    assert gripper_stop_target(94.0, "open", 3.0, 0.0, 95.0) == 95.0
    assert gripper_stop_target(40.0, None, 3.0, 0.0, 95.0) == 40.0
