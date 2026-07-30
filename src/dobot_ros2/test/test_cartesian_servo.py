from dobot_ros2.cartesian_servo import (
    clamp_normalized_vector,
    integrate_pose,
    joints_within_margin,
    pose_within_workspace,
    select_hold_pose,
    slew_vector,
)


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
