from dataclasses import dataclass
from typing import Optional, Sequence


@dataclass(frozen=True)
class JoyMapping:
    x_axis_index: int = 1
    y_axis_index: int = 0
    z_axis_index: int = 4
    rz_axis_index: int = 3
    deadzone: float = 0.25


def deadman_pressed(buttons: Sequence[int], deadman_button_index: int) -> bool:
    index = int(deadman_button_index)
    if index < 0:
        return True
    return index < len(buttons) and int(buttons[index]) == 1


def axis_to_jog(axes: Sequence[float], mapping: JoyMapping) -> Optional[str]:
    candidates = [
        _axis_value(axes, mapping.x_axis_index, "X"),
        _axis_value(axes, mapping.y_axis_index, "Y"),
        _axis_value(axes, mapping.z_axis_index, "Z"),
        _axis_value(axes, mapping.rz_axis_index, "Rz"),
    ]
    candidates = [candidate for candidate in candidates if candidate[0] is not None]
    if not candidates:
        return None
    axis_id, value = max(candidates, key=lambda item: abs(item[1]))
    if abs(value) < float(mapping.deadzone):
        return None
    return f"{axis_id}{'+' if value > 0.0 else '-'}"


def _axis_value(axes: Sequence[float], index: int, axis_id: str):
    index = int(index)
    if index < 0 or index >= len(axes):
        return None, 0.0
    return axis_id, float(axes[index])
