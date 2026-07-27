import sys
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from dobot_keyboard.keyboard_common import (
    ESTOP_KEY,
    KeyboardSafetyConfig,
    apply_delta,
    decide_gripper_opening,
    input_event_to_key_message,
    key_to_delta,
    key_to_jog_axis,
    normalize_key,
    parse_key_message,
    robot_state_allows_motion,
    target_within_limits,
)


def test_key_to_delta_maps_cartesian_steps():
    assert key_to_delta("w", 5.0, 2.0) == [5.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    assert key_to_delta("s", 5.0, 2.0) == [-5.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    assert key_to_delta("a", 5.0, 2.0) == [0.0, 5.0, 0.0, 0.0, 0.0, 0.0]
    assert key_to_delta("d", 5.0, 2.0) == [0.0, -5.0, 0.0, 0.0, 0.0, 0.0]
    assert key_to_delta("r", 5.0, 2.0) == [0.0, 0.0, 5.0, 0.0, 0.0, 0.0]
    assert key_to_delta("f", 5.0, 2.0) == [0.0, 0.0, -5.0, 0.0, 0.0, 0.0]


def test_key_to_delta_maps_rotation_steps():
    assert key_to_delta("z", 5.0, 2.0) == [0.0, 0.0, 0.0, 2.0, 0.0, 0.0]
    assert key_to_delta("x", 5.0, 2.0) == [0.0, 0.0, 0.0, -2.0, 0.0, 0.0]
    assert key_to_delta("t", 5.0, 2.0) == [0.0, 0.0, 0.0, 0.0, 2.0, 0.0]
    assert key_to_delta("g", 5.0, 2.0) == [0.0, 0.0, 0.0, 0.0, -2.0, 0.0]
    assert key_to_delta("c", 5.0, 2.0) == [0.0, 0.0, 0.0, 0.0, 0.0, 2.0]
    assert key_to_delta("v", 5.0, 2.0) == [0.0, 0.0, 0.0, 0.0, 0.0, -2.0]


def test_apply_delta_preserves_pose_order_and_units():
    pose = [300.0, 10.0, 200.0, 180.0, 0.0, 90.0]
    delta = [5.0, -5.0, 0.0, 0.0, 2.0, 0.0]

    assert apply_delta(pose, delta) == [305.0, 5.0, 200.0, 180.0, 2.0, 90.0]


def test_target_within_limits_rejects_out_of_workspace_pose():
    config = KeyboardSafetyConfig(
        min_pose=[150.0, -300.0, 50.0, -180.0, -180.0, -180.0],
        max_pose=[600.0, 300.0, 500.0, 180.0, 180.0, 180.0],
    )

    assert target_within_limits([300.0, 0.0, 200.0, 0.0, 0.0, 0.0], config)[0]
    ok, reason = target_within_limits([100.0, 0.0, 200.0, 0.0, 0.0, 0.0], config)
    assert not ok
    assert "x=100.000" in reason


def test_decide_gripper_opening_toggles_by_current_opening():
    assert decide_gripper_opening(80.0, 45.0, 95.0, 0.0) == 0.0
    assert decide_gripper_opening(20.0, 45.0, 95.0, 0.0) == 95.0


def test_keyboard_estop_key_is_normalized():
    assert normalize_key("e") == ESTOP_KEY
    assert normalize_key("estop") == ESTOP_KEY


def test_robot_state_allows_motion_only_when_enabled_and_error_free():
    assert robot_state_allows_motion(True, True, 5, 1, 0)[0]

    ok, reason = robot_state_allows_motion(False, True, 5, 1, 0)
    assert not ok
    assert "not connected" in reason

    ok, reason = robot_state_allows_motion(True, True, 9, 1, 1)
    assert not ok
    assert "error" in reason

    ok, reason = robot_state_allows_motion(True, True, 4, 0, 0)
    assert not ok
    assert "not enabled" in reason


def test_key_to_jog_axis_maps_cartesian_jog_keys():
    assert key_to_jog_axis("w") == "X+"
    assert key_to_jog_axis("s") == "X-"
    assert key_to_jog_axis("a") == "Y+"
    assert key_to_jog_axis("d") == "Y-"
    assert key_to_jog_axis("r") == "Z+"
    assert key_to_jog_axis("f") == "Z-"
    assert key_to_jog_axis("z") == "Rx+"
    assert key_to_jog_axis("x") == "Rx-"
    assert key_to_jog_axis("t") == "Ry+"
    assert key_to_jog_axis("g") == "Ry-"
    assert key_to_jog_axis("c") == "Rz+"
    assert key_to_jog_axis("v") == "Rz-"


def test_key_message_parser_supports_press_and_release_events():
    assert parse_key_message("down:w") == ("down", "w")
    assert parse_key_message("up:w") == ("up", "w")
    assert parse_key_message("w") == ("press", "w")
    assert parse_key_message("down:space") == ("down", "space")


def test_linux_input_key_events_are_converted_to_messages():
    assert input_event_to_key_message(1, 17, 1) == "down:w"
    assert input_event_to_key_message(1, 17, 0) == "up:w"
    assert input_event_to_key_message(1, 17, 2) is None
    assert input_event_to_key_message(1, 18, 1) == "down:e"
    assert input_event_to_key_message(1, 1, 1) == "down:esc"
    assert input_event_to_key_message(3, 17, 1) is None
