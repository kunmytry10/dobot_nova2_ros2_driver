import threading
from types import SimpleNamespace

from dobot_ros2.cartesian_servo import (
    clamp_normalized_vector,
    integrate_pose,
    joints_within_margin,
    pose_within_workspace,
    select_hold_pose,
    slew_vector,
)
from dobot_ros2.controller import DashboardResult
from dobot_ros2.driver_node import DobotMotionServer


def test_servo_vector_is_clamped_and_slew_limited():
    assert clamp_normalized_vector([2.0, -2.0]) == [1.0, -1.0, 0.0, 0.0, 0.0, 0.0]
    assert slew_vector([0.0] * 6, [1.0, -1.0, 0.5, 0.0, 0.0, 0.0], 0.2) == [
        0.2,
        -0.2,
        0.2,
        0.0,
        0.0,
        0.0,
    ]


def test_servo_pose_integrates_all_six_axes():
    assert integrate_pose([0.0] * 6, [1.0, 0.5, -1.0, 1.0, 0.0, -0.5], 0.03, 30.0, 10.0) == [
        0.8999999999999999,
        0.44999999999999996,
        -0.8999999999999999,
        0.3,
        0.0,
        -0.15,
    ]


def test_normal_stop_holds_last_command_without_feedback_rollback():
    target = [101.0, 202.0, 303.0, 1.0, 2.0, 3.0]
    delayed_feedback = [99.0, 200.0, 301.0, 0.5, 1.5, 2.5]

    assert select_hold_pose("inactive", target, delayed_feedback) == target
    assert select_hold_pose("command watchdog", target, delayed_feedback) == target
    assert select_hold_pose("feedback watchdog", target, delayed_feedback) == target
    assert select_hold_pose("workspace limit", target, delayed_feedback) == delayed_feedback


def test_servo_workspace_and_joint_margin_are_fail_closed():
    minimum = [-625.0, -625.0, 20.0, -360.0, -360.0, -360.0]
    maximum = [625.0, 625.0, 625.0, 360.0, 360.0, 360.0]
    assert pose_within_workspace([100.0, 100.0, 200.0, 0.0, 0.0, 0.0], minimum, maximum, 625.0)
    assert not pose_within_workspace([500.0, 500.0, 200.0, 0.0, 0.0, 0.0], minimum, maximum, 625.0)
    assert joints_within_margin([0.0] * 6, [-180.0] * 6, [180.0] * 6, 5.0)
    assert not joints_within_margin([179.0] + [0.0] * 5, [-180.0] * 6, [180.0] * 6, 5.0)


def test_command_watchdog_holds_target_without_destroying_stream():
    calls = []
    node = SimpleNamespace(
        servo_active=True,
        servo_pause_reason="",
        servo_target_pose=[100.0, 200.0, 300.0, 0.0, 0.0, 0.0],
        servo_applied_velocity=[1.0] * 6,
        servo_last_send_success=0.0,
        servo_command_lock=threading.Lock(),
        servo_transport_fault_latched=False,
        controller=SimpleNamespace(
            servo_p=lambda pose, ensure_connected: (
                calls.append((list(pose), ensure_connected))
                or DashboardResult(True, error_id=0)
            )
        ),
        get_logger=lambda: SimpleNamespace(warning=lambda message: calls.append(message)),
        _reset_cartesian_servo_stats=lambda now: calls.append(("reset", now)),
        _publish_cartesian_servo_applied=lambda active, status: calls.append(
            (active, status)
        ),
    )

    paused = DobotMotionServer._pause_cartesian_servo(
        node,
        "command watchdog",
        12.0,
    )

    assert paused
    assert node.servo_active
    assert node.servo_pause_reason == "command watchdog"
    assert node.servo_applied_velocity == [0.0] * 6
    assert calls[0] == (node.servo_target_pose, False)
    assert calls[-1] == (False, "command watchdog")
