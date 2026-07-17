from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
PROJECT_ROOT = PACKAGE_ROOT.parents[2]
WORKSPACE_ROOT = PROJECT_ROOT / "workspace"


def test_bringup_launch_is_runtime_entrypoint_with_optional_rviz():
    bringup = PACKAGE_ROOT / "launch" / "dobot_bringup.launch.py"
    source = bringup.read_text()

    assert "dobot_motion_server" in source
    assert "robot_state_publisher" in source
    assert "DeclareLaunchArgument(\"rviz\", default_value=\"false\")" in source
    assert "IfCondition(rviz)" in source
    assert "nova2_robot.urdf" in source


def test_visualization_launch_delegates_to_bringup_with_rviz_enabled():
    source = (PACKAGE_ROOT / "launch" / "dobot_visualization.launch.py").read_text()

    assert "IncludeLaunchDescription" in source
    assert "dobot_bringup.launch.py" in source
    assert "DeclareLaunchArgument(\"rviz\", default_value=\"true\")" in source
    assert "(\"rviz\", LaunchConfiguration(\"rviz\"))" in source


def test_project_makefile_wraps_common_ros_workflows():
    source = (WORKSPACE_ROOT / "Makefile").read_text()

    for target in (
        "build:",
        "driver:",
        "bringup:",
        "rviz:",
        "services:",
        "tf:",
        "state:",
        "errors:",
        "clear:",
        "enable:",
        "disable:",
        "joints:",
        "tcp:",
        "movej:",
        "movejp:",
        "movel:",
        "movep:",
    ):
        assert target in source

    assert "SPEED ?= 2" in source
    assert "ACC ?= 2" in source
    assert "TIMEOUT ?= 20.0" in source
    assert "SHELL := /bin/bash" in source
    assert "source /opt/ros/humble/setup.bash" in source
    assert "docker compose" not in source
    assert 'grep -E "^/tf$$|^/tf_static$$"' in source
    assert "ros2 service call /movep dobot_interfaces/srv/MoveCommand" in source
