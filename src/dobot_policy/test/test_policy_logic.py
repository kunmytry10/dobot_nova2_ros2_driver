import numpy as np
import pytest

from dobot_policy.policy_logic import (
    build_state,
    decode_move_jog,
    joints_within_margin,
    normalized_gripper_target,
    servo_velocity,
    tcp_within_workspace,
    validate_action_chunk,
)


def test_validate_action_chunk_clamps_model_overshoot():
    actions = np.zeros((16, 7), dtype=np.float32)
    actions[0, 0] = 1.026
    actions[1, 1] = -1.006
    result = validate_action_chunk(actions)
    assert result[0, 0] == 1.0
    assert result[1, 1] == -1.0


def test_validate_action_chunk_rejects_invalid_output():
    with pytest.raises(ValueError, match="shape"):
        validate_action_chunk(np.zeros((8, 7)))
    actions = np.zeros((16, 7))
    actions[0, 0] = np.nan
    with pytest.raises(ValueError, match="NaN"):
        validate_action_chunk(actions)


def test_decode_move_jog_matches_training_one_hot_semantics():
    decision = decode_move_jog([0.02, -0.91, 0.1, 0, 0, 0, 1], 0.5, 0.15)
    assert decision.axis_id == "Y-"
    assert decision.reason == "accepted"
    assert decode_move_jog([0.2, 0, 0, 0, 0, 0, 1], 0.5, 0.15).axis_id is None
    ambiguous = decode_move_jog([0.9, 0.85, 0, 0, 0, 0, 1], 0.5, 0.15)
    assert ambiguous.axis_id is None
    assert ambiguous.reason == "ambiguous multi-axis output"
    rotation = decode_move_jog([0, 0, 0, 1, 0, 0, 1], 0.5, 0.15)
    assert rotation.axis_id is None
    assert rotation.reason == "rotation disabled"


def test_servo_velocity_preserves_continuous_axes_and_masks_untrained_rotation():
    result = servo_velocity(
        [1.2, -0.8, 0.02, 0.7, -0.5, 0.1, 1.0],
        deadband=0.03,
        axis_scales=[1, 1, 1, 0, 0, 1],
    )
    np.testing.assert_allclose(result, [1.0, -0.8, 0.0, 0.0, 0.0, 0.1])


def test_servo_velocity_rejects_bad_axis_scales():
    with pytest.raises(ValueError, match="shape"):
        servo_velocity([0.0] * 7, axis_scales=[1.0] * 5)


def test_state_and_gripper_mapping_preserve_dataset_units():
    state = build_state([0, 1, 2, 3, 4, 5], [100, 200, 300, 10, 20, 30], 95)
    assert state.shape == (13,)
    assert state.dtype == np.float32
    assert state.tolist() == [0, 1, 2, 3, 4, 5, 100, 200, 300, 10, 20, 30, 95]
    assert normalized_gripper_target([0, 0, 0, 0, 0, 0, -0.2], 0, 95) == 0
    assert normalized_gripper_target([0, 0, 0, 0, 0, 0, 0.499], 0, 95) == 0
    assert normalized_gripper_target([0, 0, 0, 0, 0, 0, 0.5], 0, 95) == 95
    assert normalized_gripper_target([0, 0, 0, 0, 0, 0, 1.2], 0, 95) == 95


def test_joint_margin_uses_radian_observation_and_degree_limits():
    assert joints_within_margin([0.0] * 6, [-180.0] * 6, [180.0] * 6, 5.0)
    near_limit = np.deg2rad([176.0, 0, 0, 0, 0, 0])
    assert not joints_within_margin(near_limit, [-180.0] * 6, [180.0] * 6, 5.0)
    assert tcp_within_workspace(
        [-280.0, -80.0, 240.0, 0, 0, 0],
        [-400.0, -250.0, 150.0],
        [-80.0, 120.0, 330.0],
    )
    assert not tcp_within_workspace(
        [-280.0, -80.0, 140.0, 0, 0, 0],
        [-400.0, -250.0, 150.0],
        [-80.0, 120.0, 330.0],
    )


def test_policy_launch_enables_driver_side_move_jog_watchdog():
    from pathlib import Path

    package_root = Path(__file__).resolve().parents[1]
    launch_source = (package_root / "launch" / "pi05_tape_grasp.launch.py").read_text()
    driver_source = (
        package_root.parent / "dobot_ros2" / "dobot_ros2" / "driver_node.py"
    ).read_text()
    assert '"move_jog_watchdog_sec": "0.5"' in launch_source
    assert 'self.declare_parameter("move_jog.watchdog_sec", 0.0)' in driver_source
    assert "MoveJog heartbeat watchdog stopped motion" in driver_source
