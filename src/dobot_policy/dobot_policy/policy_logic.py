from dataclasses import dataclass
from typing import Optional, Sequence

import numpy as np


AXIS_NAMES = ("X", "Y", "Z", "Rx", "Ry", "Rz")


@dataclass(frozen=True)
class JogDecision:
    axis_id: Optional[str]
    magnitude: float
    reason: str


def clamp_action(action: Sequence[float]) -> np.ndarray:
    values = np.asarray(action, dtype=np.float64)
    if values.shape != (7,):
        raise ValueError(f"action must have shape (7,), got {values.shape}")
    if not np.isfinite(values).all():
        raise ValueError("action contains NaN or Inf")
    return np.clip(values, -1.0, 1.0)


def validate_action_chunk(actions, expected_horizon: int = 16) -> np.ndarray:
    values = np.asarray(actions, dtype=np.float64)
    expected = (int(expected_horizon), 7)
    if values.shape != expected:
        raise ValueError(f"actions must have shape {expected}, got {values.shape}")
    if not np.isfinite(values).all():
        raise ValueError("actions contain NaN or Inf")
    return np.clip(values, -1.0, 1.0)


def servo_velocity(
    action: Sequence[float],
    deadband: float = 0.0,
    axis_scales: Sequence[float] = (1.0, 1.0, 1.0, 0.0, 0.0, 1.0),
) -> np.ndarray:
    """Map a ServoP policy action to the driver's normalized velocity."""
    scales = np.asarray(axis_scales, dtype=np.float64)
    if scales.shape != (6,):
        raise ValueError(f"servo axis scales must have shape (6,), got {scales.shape}")
    if not np.isfinite(scales).all():
        raise ValueError("servo axis scales contain NaN or Inf")
    values = clamp_action(action)[:6]
    values[np.abs(values) < max(0.0, float(deadband))] = 0.0
    return np.clip(values * scales, -1.0, 1.0)


def decode_move_jog(
    action: Sequence[float],
    threshold: float,
    minimum_axis_margin: float,
    allow_rotation: bool = False,
) -> JogDecision:
    values = clamp_action(action)[:6]
    magnitudes = np.abs(values)
    index = int(np.argmax(magnitudes))
    largest = float(magnitudes[index])
    if largest < max(0.0, float(threshold)):
        return JogDecision(None, largest, "below threshold")

    second = float(np.partition(magnitudes, -2)[-2])
    if largest - second < max(0.0, float(minimum_axis_margin)):
        return JogDecision(None, largest, "ambiguous multi-axis output")
    if index >= 3 and not allow_rotation:
        return JogDecision(None, largest, "rotation disabled")

    sign = "+" if values[index] > 0.0 else "-"
    return JogDecision(f"{AXIS_NAMES[index]}{sign}", largest, "accepted")


def normalized_gripper_target(
    action: Sequence[float],
    minimum_opening_mm: float,
    maximum_opening_mm: float,
    open_threshold: float = 0.5,
) -> float:
    value = float(clamp_action(action)[6])
    minimum = float(minimum_opening_mm)
    maximum = float(maximum_opening_mm)
    if maximum <= minimum:
        raise ValueError("maximum gripper opening must be greater than minimum")
    # The Dobot training dataset encodes gripper state as a binary action:
    # 0.0 closes and 1.0 opens. Policy outputs are continuous, so threshold
    # them before they become a physical command.
    return maximum if value >= float(open_threshold) else minimum


def joints_within_margin(
    joints_rad: Sequence[float],
    lower_deg: Sequence[float],
    upper_deg: Sequence[float],
    margin_deg: float,
) -> bool:
    if len(joints_rad) < 6 or len(lower_deg) < 6 or len(upper_deg) < 6:
        return False
    joints_deg = np.rad2deg(np.asarray(joints_rad[:6], dtype=np.float64))
    margin = max(0.0, float(margin_deg))
    return all(
        float(low) + margin < float(value) < float(high) - margin
        for value, low, high in zip(joints_deg, lower_deg[:6], upper_deg[:6])
    )


def tcp_within_workspace(
    tcp_pose_mm_deg: Sequence[float],
    minimum_mm: Sequence[float],
    maximum_mm: Sequence[float],
) -> bool:
    if len(tcp_pose_mm_deg) < 3 or len(minimum_mm) < 3 or len(maximum_mm) < 3:
        return False
    return all(
        float(low) <= float(value) <= float(high)
        for value, low, high in zip(
            tcp_pose_mm_deg[:3], minimum_mm[:3], maximum_mm[:3]
        )
    )


def build_state(
    joints_rad: Sequence[float],
    tcp_pose_mm_deg: Sequence[float],
    gripper_opening_mm: float,
) -> np.ndarray:
    if len(joints_rad) < 6:
        raise ValueError("six joint positions are required")
    if len(tcp_pose_mm_deg) < 6:
        raise ValueError("six TCP pose values are required")
    state = np.asarray(
        list(joints_rad[:6])
        + list(tcp_pose_mm_deg[:6])
        + [float(gripper_opening_mm)],
        dtype=np.float32,
    )
    if not np.isfinite(state).all():
        raise ValueError("state contains NaN or Inf")
    return state
