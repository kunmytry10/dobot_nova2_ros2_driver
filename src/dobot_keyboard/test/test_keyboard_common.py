import sys
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from dobot_keyboard.keyboard_common import (
    KeyboardSafetyConfig,
    apply_delta,
    decide_gripper_opening,
    key_to_delta,
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
