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
        "control-ui:",
        "control-ui-only:",
        "services:",
        "topics:",
        "tf:",
        "state:",
        "errors:",
        "clear:",
        "enable:",
        "disable:",
        "estop:",
        "joints:",
        "tcp:",
        "gripper-init:",
        "gripper-state:",
        "gripper-open:",
        "gripper-close:",
        "gripper-move:",
        "camera:",
        "camera-topics:",
        "camera-info:",
        "handeye-check:",
        "handeye-capture:",
        "handeye-solve:",
        "handeye-validate:",
        "handeye-diagnose:",
        "handeye-tf:",
        "teach-start:",
        "teach-stop:",
        "teach-replay:",
        "teach-replay-servoj:",
        "teach-list:",
        "teach-delete:",
        "teach-status:",
        "movej:",
        "movejp:",
        "movel:",
        "movep:",
    ):
        assert target in source

    assert "SPEED ?= 2" in source
    assert "ACC ?= 2" in source
    assert "TIMEOUT ?= 20.0" in source
    assert "TRAJ ?=" in source
    assert "REPLAY_MODE ?=" in source
    assert "CONSOLE_PORT ?= 8080" in source
    assert "GRIPPER_FORCE ?= 50" in source
    assert "GRIPPER_FORCE_N ?= -1.0" in source
    assert "CAMERA_LAUNCH ?= gemini_330_series.launch.py" in source
    assert "HANDEYE_DATASET_ROOT ?= handeye_datasets" in source
    assert "HANDEYE_RESULT_FILE ?=" in source
    assert "HANDEYE_DIAGNOSE_FILE ?=" in source
    assert "HANDEYE_METHOD ?= TSAI" in source
    assert "dobot_control_console.launch.py" in source
    assert "ros2 launch orbbec_camera $(CAMERA_LAUNCH)" in source
    assert "ros2 run dobot_ros2 dobot_handeye_check" in source
    assert "ros2 run dobot_ros2 dobot_handeye_capture" in source
    assert "ros2 run dobot_ros2 dobot_handeye_solve" in source
    assert "ros2 run dobot_ros2 dobot_handeye_validate" in source
    assert "ros2 run dobot_ros2 dobot_handeye_diagnose" in source
    assert "ros2 run dobot_ros2 dobot_handeye_tf" in source
    assert "ros2 service call /emergency_stop std_srvs/srv/Trigger" in source
    assert "ros2 service call /gripper_move dobot_interfaces/srv/GripperCommand" in source
    assert "force_n: $(GRIPPER_FORCE_N)" in source
    assert "ros2 service call /get_gripper_state dobot_interfaces/srv/GripperState" in source
    assert "ros2 service call /teach_replay dobot_interfaces/srv/TrajectoryCommand" in source
    assert "replay_mode: 'servoj'" in source
    assert "ros2 service call /teach_list dobot_interfaces/srv/TrajectoryList" in source
    assert "SHELL := /bin/bash" in source
    assert "WS ?= $(CURDIR)" in source
    assert "WS ?= /home/ros/ws" not in source
    assert "source /opt/ros/humble/setup.bash" in source
    assert "docker compose" not in source
    assert 'grep -E "^/tf$$|^/tf_static$$"' in source
    assert "ros2 service call /movep dobot_interfaces/srv/MoveCommand" in source
    assert "^/dobot_state$$" in source
    assert "^/gripper_state$$" in source


def test_handeye_config_documents_camera_topics_and_board():
    source = (PACKAGE_ROOT / "config" / "dobot_ros2.yaml").read_text()

    assert "handeye:" in source
    assert "/camera/color/image_raw" in source
    assert "/camera/color/camera_info" in source
    assert "camera_color_optical_frame" in source
    assert "Link6" in source
    assert "squares_x: 12" in source
    assert "squares_y: 9" in source
    assert "square_length_m: 0.015" in source
    assert "marker_length_m: 0.01125" in source
    assert 'dictionary: "DICT_5X5_100"' in source


def test_control_console_launch_and_package_entrypoint_are_installed():
    launch = PACKAGE_ROOT / "launch" / "dobot_control_console.launch.py"
    setup = (PACKAGE_ROOT / "setup.py").read_text()

    source = launch.read_text()
    assert "dobot_control_console" in source
    assert "start_driver" in source
    assert "start_state_publisher" in source
    assert "console_port" in source
    assert "dobot_control_console = dobot_ros2.control_console:main" in setup
    assert 'share/{package_name}/web' in setup


def test_handeye_console_entrypoints_are_installed():
    setup = (PACKAGE_ROOT / "setup.py").read_text()

    assert "dobot_handeye_check = dobot_ros2.handeye_check:main" in setup
    assert "dobot_handeye_capture = dobot_ros2.handeye_capture:main" in setup
    assert "dobot_handeye_solve = dobot_ros2.handeye_solve:main" in setup
    assert "dobot_handeye_validate = dobot_ros2.handeye_validate:main" in setup
    assert "dobot_handeye_diagnose = dobot_ros2.handeye_diagnose:main" in setup
    assert "dobot_handeye_tf = dobot_ros2.handeye_tf:main" in setup


def test_control_console_serializes_ros_numpy_arrays():
    source = (PACKAGE_ROOT / "dobot_ros2" / "control_console.py").read_text()

    assert "get_fields_and_field_types" in source
    assert 'hasattr(value, "tolist")' in source


def test_control_console_subscribes_to_state_topics_and_gripper_services():
    source = (PACKAGE_ROOT / "dobot_ros2" / "control_console.py").read_text()

    assert 'create_subscription(DobotState, "dobot_state"' in source
    assert 'create_subscription(GripperStatus, "gripper_state"' in source
    assert 'create_client(GripperCommand, "gripper_move")' in source
    assert 'create_client(GripperState, "get_gripper_state")' in source
    assert '"/api/gripper/move"' in source
