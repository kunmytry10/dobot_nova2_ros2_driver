from types import SimpleNamespace

import numpy as np
from rclpy.clock import Clock

from dobot_policy.policy_node import DobotPolicyNode


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def test_servo_command_publishes_normalized_velocity_and_explicit_stop():
    publisher = _Publisher()
    node = SimpleNamespace(
        armed=True,
        coord_type=0,
        user=0,
        tool=0,
        servo_pub=publisher,
        _last_servo_velocity=np.zeros(6),
        _motion_started_at=None,
        motion_test_duration_sec=3.0,
        _event=lambda *_args, **_kwargs: None,
        get_clock=lambda: Clock(),
    )

    DobotPolicyNode._set_servo_velocity(node, [1.2, -0.5, 0.25, 0, 0, 0.1])
    command = publisher.messages[-1]
    assert list(command.normalized_velocity) == [1.0, -0.5, 0.25, 0.0, 0.0, 0.1]
    assert command.active is True
    assert command.deadman is True

    DobotPolicyNode._set_servo_velocity(node, np.zeros(6), active=False)
    stop = publisher.messages[-1]
    assert list(stop.normalized_velocity) == [0.0] * 6
    assert stop.active is False
    assert stop.deadman is False


def test_observation_artifacts_are_replayable(tmp_path):
    node = SimpleNamespace(_artifact_dir=tmp_path)
    observation = {
        "global_image": np.full((12, 16, 3), 32, dtype=np.uint8),
        "wrist_image": np.full((10, 14, 3), 192, dtype=np.uint8),
        "state": np.arange(13, dtype=np.float32),
    }

    DobotPolicyNode._save_observation_artifacts(node, 7, observation)

    assert (tmp_path / "request_0007_global.jpg").is_file()
    assert (tmp_path / "request_0007_wrist.jpg").is_file()
    np.testing.assert_array_equal(
        np.load(tmp_path / "request_0007_state.npy"), observation["state"]
    )


def test_motion_only_safety_does_not_require_initialized_gripper():
    now = 100.0
    node = SimpleNamespace(
        _latest={
            "robot": SimpleNamespace(
                connected=True,
                feedback_valid=True,
                error_status=0,
                robot_mode=5,
                enable_status=1,
            ),
            "joints": [0.0] * 6,
            "tcp": [-163.0, -76.0, 285.0, 180.0, 3.0, -92.0],
            "gripper": SimpleNamespace(
                success=False,
                connected=False,
                initialized=False,
            ),
            "images": (object(), object()),
        },
        _received={name: now for name in ("robot", "joints", "tcp", "gripper", "images")},
        source_timeout_sec=1.0,
        joint_lower_limits_deg=[-360.0, -180.0, -156.0, -360.0, -360.0, -360.0],
        joint_upper_limits_deg=[360.0, 180.0, 156.0, 360.0, 360.0, 360.0],
        joint_limit_margin_deg=5.0,
        workspace_min_mm=[-400.0, -250.0, 150.0],
        workspace_max_mm=[-80.0, 120.0, 330.0],
        armed=True,
        gripper_enabled=False,
    )

    assert DobotPolicyNode._safety_reason(node, now) == ""

    node.gripper_enabled = True
    assert "gripper state" in DobotPolicyNode._safety_reason(node, now)
