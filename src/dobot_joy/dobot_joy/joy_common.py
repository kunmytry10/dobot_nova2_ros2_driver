from dataclasses import dataclass
import math
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
    rx_axis_index: int = 6
    rx_axis_sign: float = -1.0
    ry_axis_index: int = 7
    ry_axis_sign: float = -1.0
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
        _axis_value(axes, mapping.rx_axis_index, mapping.rx_axis_sign, "Rx"),
        _axis_value(axes, mapping.ry_axis_index, mapping.ry_axis_sign, "Ry"),
    ]
    candidates = [candidate for candidate in candidates if candidate[0] is not None]
    if not candidates:
        return None
    axis_id, value = max(candidates, key=lambda item: abs(item[1]))
    if abs(value) < float(mapping.deadzone):
        return None
    return f"{axis_id}{'+' if value > 0.0 else '-'}"


def axes_to_cartesian_velocity(
    axes: Sequence[float],
    mapping: JoyMapping,
    response_exponent: float = 1.2,
):
    """Return a simultaneous normalized [X,Y,Z,Rx,Ry,Rz] velocity command."""
    values = [
        _axis_value(axes, mapping.x_axis_index, mapping.x_axis_sign, "X")[1],
        _axis_value(axes, mapping.y_axis_index, mapping.y_axis_sign, "Y")[1],
        _axis_value(axes, mapping.z_axis_index, mapping.z_axis_sign, "Z")[1],
        _axis_value(axes, mapping.rx_axis_index, mapping.rx_axis_sign, "Rx")[1],
        _axis_value(axes, mapping.ry_axis_index, mapping.ry_axis_sign, "Ry")[1],
        _axis_value(axes, mapping.rz_axis_index, mapping.rz_axis_sign, "Rz")[1],
    ]
    values = [
        _shape_axis(value, mapping.deadzone, response_exponent) for value in values
    ]
    return _limit_vector_norm(values[:3]) + _limit_vector_norm(values[3:])


def jog_axis_to_action(axis_id: Optional[str]):
    vector = [0.0] * 6
    if not axis_id:
        return vector
    for index, name in enumerate(("X", "Y", "Z", "Rx", "Ry", "Rz")):
        if str(axis_id) in {f"{name}+", f"{name}-"}:
            vector[index] = -1.0 if str(axis_id).endswith("-") else 1.0
            break
    return vector


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


def gripper_stop_target(
    opening_mm: float,
    direction: Optional[str],
    lead_mm: float,
    minimum_mm: float,
    maximum_mm: float,
) -> float:
    if direction == "close":
        return clamp(opening_mm - abs(float(lead_mm)), minimum_mm, maximum_mm)
    if direction == "open":
        return clamp(opening_mm + abs(float(lead_mm)), minimum_mm, maximum_mm)
    return clamp(opening_mm, minimum_mm, maximum_mm)


def clamp(value: float, minimum: float, maximum: float) -> float:
    return min(max(float(value), float(minimum)), float(maximum))


def _shape_axis(value: float, deadzone: float, exponent: float) -> float:
    magnitude = abs(float(value))
    deadzone = clamp(deadzone, 0.0, 0.99)
    if magnitude <= deadzone:
        return 0.0
    normalized = clamp((magnitude - deadzone) / (1.0 - deadzone), 0.0, 1.0)
    shaped = normalized ** max(1.0, float(exponent))
    return math.copysign(shaped, value)


def _limit_vector_norm(values: Sequence[float]):
    norm = math.sqrt(sum(float(value) ** 2 for value in values))
    if norm <= 1.0 or norm <= 0.0:
        return [float(value) for value in values]
    return [float(value) / norm for value in values]


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
