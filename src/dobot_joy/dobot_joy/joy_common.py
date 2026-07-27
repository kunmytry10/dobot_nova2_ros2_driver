from dataclasses import dataclass
from typing import Optional, Sequence


@dataclass(frozen=True)
class JoyMapping:
    x_axis_index: int = 1
    x_axis_sign: float = -1.0
    y_axis_index: int = 0
    y_axis_sign: float = -1.0
    z_axis_index: int = 4
    z_axis_sign: float = 1.0
    rz_axis_index: int = 3
    rz_axis_sign: float = -1.0
    lt_axis_index: int = 2
    rt_axis_index: int = 5
    deadzone: float = 0.25
    gripper_step_mm: float = 2.0


def deadman_pressed(buttons: Sequence[int], deadman_button_index: int) -> bool:
    index = int(deadman_button_index)
    if index < 0:
        return True
    return button_pressed(buttons, index)


def button_pressed(buttons: Sequence[int], index: int) -> bool:
    index = int(index)
    return index >= 0 and index < len(buttons) and int(buttons[index]) == 1


def axis_to_jog(axes: Sequence[float], mapping: JoyMapping) -> Optional[str]:
    candidates = [
        _axis_value(axes, mapping.x_axis_index, mapping.x_axis_sign, "X"),
        _axis_value(axes, mapping.y_axis_index, mapping.y_axis_sign, "Y"),
        _axis_value(axes, mapping.z_axis_index, mapping.z_axis_sign, "Z"),
        _axis_value(axes, mapping.rz_axis_index, mapping.rz_axis_sign, "Rz"),
    ]
    candidates = [candidate for candidate in candidates if candidate[0] is not None]
    if not candidates:
        return None
    axis_id, value = max(candidates, key=lambda item: abs(item[1]))
    if abs(value) < float(mapping.deadzone):
        return None
    return f"{axis_id}{'+' if value > 0.0 else '-'}"


def axis_to_gripper_delta(
    axes: Sequence[float],
    mapping: JoyMapping,
    neutral_axes: Optional[Sequence[float]] = None,
) -> Optional[float]:
    lt = _trigger_activation(axes, neutral_axes, mapping.lt_axis_index)
    rt = _trigger_activation(axes, neutral_axes, mapping.rt_axis_index)
    if lt < mapping.deadzone and rt < mapping.deadzone:
        return None
    if lt >= rt:
        return -abs(float(mapping.gripper_step_mm))
    return abs(float(mapping.gripper_step_mm))


def trigger_pressed(
    axes: Sequence[float],
    neutral_axes: Optional[Sequence[float]],
    index: int,
    deadzone: float,
) -> bool:
    return _trigger_activation(axes, neutral_axes, index) >= float(deadzone)


def clamp(value: float, minimum: float, maximum: float) -> float:
    return min(max(float(value), float(minimum)), float(maximum))


def _axis_value(axes: Sequence[float], index: int, sign: float, axis_id: str):
    index = int(index)
    if index < 0 or index >= len(axes):
        return None, 0.0
    return axis_id, float(axes[index]) * float(sign)


def _trigger_activation(
    axes: Sequence[float],
    neutral_axes: Optional[Sequence[float]],
    index: int,
) -> float:
    index = int(index)
    if index < 0 or index >= len(axes):
        return 0.0
    neutral = 0.0
    if neutral_axes is not None and index < len(neutral_axes):
        neutral = float(neutral_axes[index])
    return abs(float(axes[index]) - neutral)
