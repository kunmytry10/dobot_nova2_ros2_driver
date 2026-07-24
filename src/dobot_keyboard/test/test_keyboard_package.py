from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_ROOT = PACKAGE_ROOT.parents[1]


def test_keyboard_package_installs_nodes_and_launch_file():
    package_xml = (PACKAGE_ROOT / "package.xml").read_text()
    setup = (PACKAGE_ROOT / "setup.py").read_text()
    launch = (PACKAGE_ROOT / "launch" / "keyboard_teleop.launch.py").read_text()

    assert "<name>dobot_keyboard</name>" in package_xml
    assert "<exec_depend>dobot_interfaces</exec_depend>" in package_xml
    assert "<exec_depend>std_msgs</exec_depend>" in package_xml
    assert "dobot_keyboard_input = dobot_keyboard.keyboard_input:main" in setup
    assert "dobot_keyboard_teleop = dobot_keyboard.keyboard_teleop:main" in setup
    assert 'share/{package_name}/launch' in setup
    assert "dobot_keyboard_input" in launch
    assert "dobot_keyboard_teleop" in launch
    assert "input_topic" in launch


def test_makefile_exposes_keyboard_workflows():
    source = (WORKSPACE_ROOT / "Makefile").read_text()

    assert "keyboard:" in source
    assert "keyboard-input:" in source
    assert "keyboard-teleop:" in source
    assert "KEYBOARD_TOPIC ?= /keyboard/input" in source
    assert "KEYBOARD_STEP_MM ?= 5.0" in source
    assert "KEYBOARD_ROT_STEP_DEG ?= 2.0" in source
    assert "KEYBOARD_MOTION_SERVICE ?= movep" in source
    assert "--packages-up-to dobot_camera dobot_handeye dobot_keyboard dobot_ros2" in source
    assert "ros2 launch dobot_keyboard keyboard_teleop.launch.py" in source
    assert "ros2 run dobot_keyboard dobot_keyboard_input" in source
    assert "ros2 run dobot_keyboard dobot_keyboard_teleop" in source
