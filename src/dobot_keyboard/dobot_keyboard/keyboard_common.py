from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple


MOVE_KEYS = {"w", "s", "a", "d", "r", "f", "z", "x", "t", "g", "c", "v"}
TOGGLE_GRIPPER_KEY = "space"
RESET_SIM_KEY = "q"
ESTOP_KEY = "e"
QUIT_KEY = "esc"


@dataclass(frozen=True)
class KeyboardSafetyConfig:
    min_pose: Sequence[float]
    max_pose: Sequence[float]
    max_xy_radius_mm: Optional[float] = None


def key_to_delta(
    key: str, translation_step_mm: float, rotation_step_deg: float
) -> Optional[List[float]]:
    key = normalize_key(key)
    deltas = {
        "w": [translation_step_mm, 0.0, 0.0, 0.0, 0.0, 0.0],
        "s": [-translation_step_mm, 0.0, 0.0, 0.0, 0.0, 0.0],
        "a": [0.0, translation_step_mm, 0.0, 0.0, 0.0, 0.0],
        "d": [0.0, -translation_step_mm, 0.0, 0.0, 0.0, 0.0],
        "r": [0.0, 0.0, translation_step_mm, 0.0, 0.0, 0.0],
        "f": [0.0, 0.0, -translation_step_mm, 0.0, 0.0, 0.0],
        "z": [0.0, 0.0, 0.0, rotation_step_deg, 0.0, 0.0],
        "x": [0.0, 0.0, 0.0, -rotation_step_deg, 0.0, 0.0],
        "t": [0.0, 0.0, 0.0, 0.0, rotation_step_deg, 0.0],
        "g": [0.0, 0.0, 0.0, 0.0, -rotation_step_deg, 0.0],
        "c": [0.0, 0.0, 0.0, 0.0, 0.0, rotation_step_deg],
        "v": [0.0, 0.0, 0.0, 0.0, 0.0, -rotation_step_deg],
    }
    return deltas.get(key)


def normalize_key(key: str) -> str:
    value = key.strip().lower()
    if value in {" ", "spacebar"}:
        return TOGGLE_GRIPPER_KEY
    if value in {"e", "estop", "emergency_stop"}:
        return ESTOP_KEY
    if value in {"\x1b", "escape"}:
        return QUIT_KEY
    return value


def robot_state_allows_motion(
    connected: bool,
    feedback_valid: bool,
    robot_mode: int,
    enable_status: int,
    error_status: int,
) -> Tuple[bool, str]:
    if not connected:
        return False, "robot is not connected"
    if not feedback_valid:
        return False, "robot feedback is not valid"
    if int(error_status) != 0 or int(robot_mode) == 9:
        return False, f"robot is in error state: robot_mode={int(robot_mode)}, error_status={int(error_status)}"
    if int(enable_status) != 1:
        return False, f"robot is not enabled: enable_status={int(enable_status)}"
    return True, ""


def apply_delta(pose: Sequence[float], delta: Sequence[float]) -> List[float]:
    if len(pose) != 6 or len(delta) != 6:
        raise ValueError("pose and delta must have 6 values")
    return [float(pose[index]) + float(delta[index]) for index in range(6)]


def target_within_limits(
    target: Sequence[float], config: KeyboardSafetyConfig
) -> Tuple[bool, str]:
    if len(target) != 6:
        return False, "target must have 6 values"
    if len(config.min_pose) != 6 or len(config.max_pose) != 6:
        return False, "workspace min/max limits must have 6 values"

    names = ("x", "y", "z", "rx", "ry", "rz")
    for index, name in enumerate(names):
        value = float(target[index])
        minimum = float(config.min_pose[index])
        maximum = float(config.max_pose[index])
        if value < minimum or value > maximum:
            return (
                False,
                f"{name}={value:.3f} outside [{minimum:.3f}, {maximum:.3f}]",
            )

    if config.max_xy_radius_mm is not None:
        x = float(target[0])
        y = float(target[1])
        radius = (x * x + y * y) ** 0.5
        if radius > float(config.max_xy_radius_mm):
            return (
                False,
                f"xy radius={radius:.3f} outside {float(config.max_xy_radius_mm):.3f}",
            )
    return True, ""


def decide_gripper_opening(
    current_opening_mm: float,
    toggle_threshold_mm: float,
    open_opening_mm: float,
    close_opening_mm: float,
) -> float:
    if float(current_opening_mm) > float(toggle_threshold_mm):
        return float(close_opening_mm)
    return float(open_opening_mm)
