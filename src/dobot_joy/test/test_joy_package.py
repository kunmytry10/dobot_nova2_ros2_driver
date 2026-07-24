from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_ROOT = PACKAGE_ROOT.parents[1]


def test_joy_package_installs_teleop_entrypoint_and_launch_file():
    package_xml = (PACKAGE_ROOT / "package.xml").read_text()
    setup = (PACKAGE_ROOT / "setup.py").read_text()
    launch = (PACKAGE_ROOT / "launch" / "joy_teleop.launch.py").read_text()

    assert "<name>dobot_joy</name>" in package_xml
    assert "<exec_depend>sensor_msgs</exec_depend>" in package_xml
    assert "<exec_depend>joy</exec_depend>" in package_xml
    assert "dobot_joy_teleop = dobot_joy.joy_teleop:main" in setup
    assert "dobot_joy_teleop" in launch
    assert "joy_node" in launch
    assert "deadman_button_index" in launch


def test_makefile_exposes_joy_workflows():
    source = (WORKSPACE_ROOT / "Makefile").read_text()

    assert "joy:" in source
    assert "joy-teleop:" in source
    assert "move-jog:" in source
    assert "jog-stop:" in source
    assert "JOY_TOPIC ?= /joy" in source
    assert "JOY_DEADMAN_BUTTON ?= 4" in source
    assert "--packages-up-to dobot_camera dobot_handeye dobot_keyboard dobot_joy dobot_ros2" in source
    assert "ros2 launch dobot_joy joy_teleop.launch.py" in source
    assert "ros2 run dobot_joy dobot_joy_teleop" in source
    assert "ros2 service call /move_jog dobot_interfaces/srv/JogCommand" in source
