from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_ROOT = PACKAGE_ROOT.parents[1]


def test_joy_package_installs_teleop_entrypoint_and_launch_file():
    package_xml = (PACKAGE_ROOT / "package.xml").read_text()
    setup = (PACKAGE_ROOT / "setup.py").read_text()
    launch = (PACKAGE_ROOT / "launch" / "joy_teleop.launch.py").read_text()

    assert "<name>dobot_joy</name>" in package_xml
    assert "<exec_depend>sensor_msgs</exec_depend>" in package_xml
    assert "<exec_depend>std_msgs</exec_depend>" in package_xml
    assert "<exec_depend>joy</exec_depend>" in package_xml
    assert "dobot_joy_teleop = dobot_joy.joy_teleop:main" in setup
    assert "dobot_joy_teleop" in launch
    assert "joy_node" in launch
    assert "deadman_button_index" in launch
    assert "toggle_gripper_button_index" in launch
    assert "toggle_enable_button_index" in launch
    assert "toggle_drag_button_index" in launch
    assert "enable_rumble" in launch
    assert "autorepeat_rate" in launch
    assert "diagnostics_topic" in launch
    teleop = (PACKAGE_ROOT / "dobot_joy" / "joy_teleop.py").read_text()
    assert "gripper_stop_pending" in teleop
    assert "msg.grip_state == 2" in teleop


def test_makefile_exposes_joy_workflows():
    source = (WORKSPACE_ROOT / "Makefile").read_text()

    assert "joy:" in source
    assert "joy-teleop:" in source
    assert "move-jog:" in source
    assert "jog-stop:" in source
    assert "JOY_TOPIC ?= /joy" in source
    assert "JOY_DEADMAN_BUTTON ?= 4" in source
    assert "JOY_X_AXIS_INDEX ?= 1" in source
    assert "JOY_Y_AXIS_INDEX ?= 0" in source
    assert "JOY_RX_AXIS_INDEX ?= 6" in source
    assert "JOY_RY_AXIS_INDEX ?= 7" in source
    assert "JOY_GRIPPER_INIT ?= true" in source
    assert "JOY_GRIPPER_STEP_MM ?= 2.0" in source
    assert "JOY_GRIPPER_STOP_LEAD_MM ?= 3.0" in source
    assert "JOY_ENABLE_RUMBLE ?= true" in source
    assert "JOY_X_AXIS_SIGN ?= -1.0" in source
    assert "JOY_Y_AXIS_SIGN ?= -1.0" in source
    assert "JOY_Z_AXIS_SIGN ?= 1.0" in source
    assert "JOY_RZ_AXIS_SIGN ?= -1.0" in source
    assert "JOY_RX_AXIS_SIGN ?= -1.0" in source
    assert "JOY_RY_AXIS_SIGN ?= -1.0" in source
    assert "JOY_AUTOREPEAT_RATE ?= 50.0" in source
    assert "JOY_DIAGNOSTICS_TOPIC ?= /joy/teleop_diagnostics" in source
    assert "gripper_init" in source
    assert "--packages-up-to dobot_camera dobot_handeye dobot_keyboard dobot_joy dobot_ros2" in source
    assert "ros2 launch dobot_joy joy_teleop.launch.py" in source
    assert "ros2 run dobot_joy dobot_joy_teleop" in source
    assert "ros2 service call /move_jog dobot_interfaces/srv/JogCommand" in source


def test_joy_teleop_has_robot_enable_and_drag_toggles():
    teleop = (PACKAGE_ROOT / "dobot_joy" / "joy_teleop.py").read_text()
    launch = (PACKAGE_ROOT / "launch" / "joy_teleop.launch.py").read_text()
    makefile = (WORKSPACE_ROOT / "Makefile").read_text()

    assert "enable_robot_client" in teleop
    assert "disable_robot_client" in teleop
    assert "drag_start_client" in teleop
    assert "drag_stop_client" in teleop
    assert "joy.toggle_enable_button_index" in teleop
    assert "joy.toggle_drag_button_index" in teleop
    assert "JOY_TOGGLE_ENABLE_BUTTON ?= 3" in makefile
    assert "JOY_TOGGLE_DRAG_BUTTON ?= 5" in makefile
    assert "toggle_enable_button_index:=$(JOY_TOGGLE_ENABLE_BUTTON)" in makefile
    assert "toggle_drag_button_index:=$(JOY_TOGGLE_DRAG_BUTTON)" in makefile
    assert "toggle_enable_button_index" in launch
    assert "toggle_drag_button_index" in launch
