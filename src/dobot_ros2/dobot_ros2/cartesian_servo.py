import math
from typing import Sequence


NON_REVERSING_HOLD_REASONS = {
    "inactive",
    "command watchdog",
    "feedback watchdog",
}


def heartbeat_expired(active: bool, last_command: float, now: float, timeout: float):
    return (
        bool(active)
        and float(timeout) > 0.0
        and float(now) - float(last_command) > float(timeout)
    )


def clamp_normalized_vector(values: Sequence[float]):
    result = [max(-1.0, min(1.0, float(value))) for value in values[:6]]
    return result + [0.0] * (6 - len(result))


def slew_vector(
    current: Sequence[float],
    target: Sequence[float],
    max_delta: float,
):
    limit = max(0.0, float(max_delta))
    result = []
    for old, new in zip(clamp_normalized_vector(current), clamp_normalized_vector(target)):
        delta = max(-limit, min(limit, new - old))
        result.append(old + delta)
    return result


def integrate_pose(
    pose: Sequence[float],
    normalized_velocity: Sequence[float],
    dt: float,
    max_translation_speed_mm_s: float,
    max_rotation_speed_deg_s: float,
):
    pose = list(pose[:6])
    if len(pose) != 6:
        raise ValueError("ServoP requires a six-dimensional TCP pose")
    velocity = clamp_normalized_vector(normalized_velocity)
    scale = [float(max_translation_speed_mm_s)] * 3 + [
        float(max_rotation_speed_deg_s)
    ] * 3
    return [
        float(value) + float(command) * speed * max(0.0, float(dt))
        for value, command, speed in zip(pose, velocity, scale)
    ]


def select_hold_pose(
    reason: str,
    target_pose: Sequence[float],
    feedback_pose: Sequence[float],
):
    if reason in NON_REVERSING_HOLD_REASONS and len(target_pose) >= 6:
        return list(target_pose[:6])
    return list(feedback_pose[:6])


def pose_within_workspace(
    pose: Sequence[float],
    minimum: Sequence[float],
    maximum: Sequence[float],
    max_xy_radius_mm: float,
):
    if len(pose) < 6 or len(minimum) < 6 or len(maximum) < 6:
        return False
    if any(
        float(value) < float(low) or float(value) > float(high)
        for value, low, high in zip(pose[:6], minimum[:6], maximum[:6])
    ):
        return False
    radius = float(max_xy_radius_mm)
    return radius <= 0.0 or math.hypot(float(pose[0]), float(pose[1])) <= radius


def joints_within_margin(
    joints_deg: Sequence[float],
    lower_deg: Sequence[float],
    upper_deg: Sequence[float],
    margin_deg: float,
):
    if len(joints_deg) < 6 or len(lower_deg) < 6 or len(upper_deg) < 6:
        return False
    margin = max(0.0, float(margin_deg))
    return all(
        float(low) + margin < float(value) < float(high) - margin
        for value, low, high in zip(joints_deg[:6], lower_deg[:6], upper_deg[:6])
    )
