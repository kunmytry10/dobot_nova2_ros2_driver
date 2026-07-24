import sys
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from dobot_joy.joy_common import JoyMapping, axis_to_jog, deadman_pressed  # noqa: E402


def test_deadman_pressed_requires_configured_button():
    assert deadman_pressed([0, 0, 0, 0, 1], 4)
    assert not deadman_pressed([0, 0, 0, 0, 0], 4)
    assert deadman_pressed([], -1)


def test_axis_to_jog_uses_dominant_axis_and_deadzone():
    mapping = JoyMapping()

    assert axis_to_jog([0.0, 0.7, 0.0, 0.0, 0.0], mapping) == "X+"
    assert axis_to_jog([0.0, -0.7, 0.0, 0.0, 0.0], mapping) == "X-"
    assert axis_to_jog([0.8, 0.2, 0.0, 0.0, 0.0], mapping) == "Y+"
    assert axis_to_jog([-0.8, 0.2, 0.0, 0.0, 0.0], mapping) == "Y-"
    assert axis_to_jog([0.0, 0.0, 0.0, 0.0, 0.9], mapping) == "Z+"
    assert axis_to_jog([0.0, 0.0, 0.0, -0.9, 0.0], mapping) == "Rz-"
    assert axis_to_jog([0.1, 0.1, 0.0, 0.0, 0.0], mapping) is None
