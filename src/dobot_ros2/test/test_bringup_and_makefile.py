from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
PROJECT_ROOT = PACKAGE_ROOT.parents[2]
WORKSPACE_ROOT = PROJECT_ROOT / "workspace"
HANDEYE_PACKAGE_ROOT = WORKSPACE_ROOT / "src" / "dobot_handeye"
CAMERA_PACKAGE_ROOT = WORKSPACE_ROOT / "src" / "dobot_camera"


def test_bringup_launch_is_runtime_entrypoint_with_optional_rviz():
    bringup = PACKAGE_ROOT / "launch" / "dobot_bringup.launch.py"
    source = bringup.read_text()

    assert "dobot_motion_server" in source
    assert "robot_state_publisher" in source
    assert "dobot_handeye" in source
    assert "dobot_handeye_tf" in source
    assert "handeye_tf" in source
    assert "handeye_result_file" in source
    assert "handeye_output_child_frame" in source
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
        "handeye-board-tf:",
        "keyboard:",
        "keyboard-input:",
        "keyboard-teleop:",
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
    assert "CAMERA_NAME ?= camera" in source
    assert "CAMERA_SERIAL ?=" in source
    assert "CAMERA_USB_PORT ?=" in source
    assert "CAMERA_SERIAL_ARG = $(if $(strip $(CAMERA_SERIAL)),serial_number:=$(CAMERA_SERIAL),)" in source
    assert "CAMERA_USB_PORT_ARG = $(if $(strip $(CAMERA_USB_PORT)),usb_port:=$(CAMERA_USB_PORT),)" in source
    assert "HANDEYE_DATASET_ROOT ?= handeye_datasets" in source
    assert "HANDEYE_RESULT_FILE ?=" in source
    assert "HANDEYE_DIAGNOSE_FILE ?=" in source
    assert "HANDEYE_METHOD ?= TSAI" in source
    assert "HANDEYE_STATIC_TF_FILE ?= $(WS)/handeye_result.yaml" in source
    assert "HANDEYE_STATIC_TF_CHILD_FRAME ?= camera_link" in source
    assert "KEYBOARD_TOPIC ?= /keyboard/input" in source
    assert "KEYBOARD_STEP_MM ?= 5.0" in source
    assert "KEYBOARD_ROT_STEP_DEG ?= 2.0" in source
    assert "KEYBOARD_MOTION_SERVICE ?= movep" in source
    assert "dobot_control_console.launch.py" in source
    assert "--packages-up-to dobot_camera dobot_handeye dobot_keyboard dobot_ros2" in source
    assert "ros2 launch dobot_camera gemini305.launch.py" in source
    assert "serial_number:=$(CAMERA_SERIAL) usb_port:=$(CAMERA_USB_PORT)" not in source
    assert "orbbec_camera $(CAMERA_LAUNCH)" not in source
    assert "ros2 run dobot_handeye dobot_handeye_check" in source
    assert "ros2 run dobot_handeye dobot_handeye_capture" in source
    assert "ros2 run dobot_handeye dobot_handeye_solve" in source
    assert "ros2 run dobot_handeye dobot_handeye_validate" in source
    assert "ros2 run dobot_handeye dobot_handeye_diagnose" in source
    assert "ros2 run dobot_handeye dobot_handeye_tf" in source
    assert "--output-child-frame $(HANDEYE_STATIC_TF_CHILD_FRAME)" in source
    assert "ros2 run dobot_handeye dobot_handeye_board_tf" in source
    assert "ros2 launch dobot_keyboard keyboard_teleop.launch.py" in source
    assert "ros2 run dobot_keyboard dobot_keyboard_input" in source
    assert "ros2 run dobot_keyboard dobot_keyboard_teleop" in source
    assert "ros2 service call /emergency_stop std_srvs/srv/Trigger" in source
    assert "ros2 service call /gripper_move dobot_interfaces/srv/GripperCommand" in source
    assert "force_n: $(GRIPPER_FORCE_N)" in source
    assert "ros2 service call /get_gripper_state dobot_interfaces/srv/GripperState" in source
    assert "ros2 service call /teach_replay dobot_interfaces/srv/TrajectoryCommand" in source
    assert "replay_mode: 'servoj'" in source
    assert "ros2 service call /teach_list dobot_interfaces/srv/TrajectoryList" in source
    assert "SHELL := /bin/bash" in source
    assert "WS ?= $(CURDIR)" in source
    assert "ORBBEC_WS ?= $(HOME)/orbbec_305" in source
    assert 'source "$(ORBBEC_WS)/install/setup.bash"' in source
    assert "WS ?= /home/ros/ws" not in source
    assert "source /opt/ros/humble/setup.bash" in source
    assert "docker compose" not in source
    assert 'grep -E "^/tf$$|^/tf_static$$"' in source
    assert "ros2 service call /movep dobot_interfaces/srv/MoveCommand" in source
    assert "^/dobot_state$$" in source
    assert "^/gripper_state$$" in source


def test_description_and_rviz_use_map_as_root_frame():
    urdf = (
        WORKSPACE_ROOT / "src" / "dobot_description" / "urdf" / "nova2_robot.urdf"
    ).read_text()
    rviz = (PACKAGE_ROOT / "rviz" / "nova2.rviz").read_text()

    assert '<link name="map"' in urdf
    assert 'link="map"' in urdf
    assert "dummy_link" not in urdf
    assert "Fixed Frame: map" in rviz
    assert "Target Frame: map" in rviz
    assert "dummy_link" not in rviz


def test_handeye_config_documents_camera_topics_and_board():
    source = (PACKAGE_ROOT / "config" / "dobot_ros2.yaml").read_text()

    assert "handeye:" in source
    assert "/camera/color/image_raw" in source
    assert "/camera/color/camera_info" in source
    assert "camera_color_optical_frame" in source
    assert 'board_frame: "handeye_board"' in source
    assert "Link6" in source
    assert "squares_x: 12" in source
    assert "squares_y: 9" in source
    assert "square_length_m: 0.015" in source
    assert "marker_length_m: 0.01125" in source
    assert 'dictionary: "DICT_5X5_100"' in source


def test_keyboard_config_documents_safe_defaults():
    source = (PACKAGE_ROOT / "config" / "dobot_ros2.yaml").read_text()

    assert "keyboard:" in source
    assert 'input_topic: "/keyboard/input"' in source
    assert "translation_step_mm: 5.0" in source
    assert "rotation_step_deg: 2.0" in source
    assert 'motion_service: "movep"' in source
    assert "workspace_min: [-625.0, -625.0, 20.0, -360.0, -360.0, -360.0]" in source
    assert "workspace_max: [625.0, 625.0, 625.0, 360.0, 360.0, 360.0]" in source
    assert "workspace_max_xy_radius_mm: 625.0" in source
    assert "gripper_opening_open_mm: 95.0" in source
    assert "gripper_toggle_threshold_mm: 45.0" in source


def test_control_console_launch_and_package_entrypoint_are_installed():
    launch = PACKAGE_ROOT / "launch" / "dobot_control_console.launch.py"
    setup = (PACKAGE_ROOT / "setup.py").read_text()

    source = launch.read_text()
    assert "dobot_control_console" in source
    assert "start_driver" in source
    assert "start_state_publisher" in source
    assert "dobot_handeye" in source
    assert "dobot_handeye_tf" in source
    assert "handeye_tf" in source
    assert "handeye_result_file" in source
    assert "handeye_output_child_frame" in source
    assert "console_port" in source
    assert "dobot_control_console = dobot_ros2.control_console:main" in setup
    assert 'share/{package_name}/web' in setup


def test_handeye_console_entrypoints_are_installed():
    driver_setup = (PACKAGE_ROOT / "setup.py").read_text()
    handeye_setup = (HANDEYE_PACKAGE_ROOT / "setup.py").read_text()
    package_xml = (HANDEYE_PACKAGE_ROOT / "package.xml").read_text()

    assert "<name>dobot_handeye</name>" in package_xml
    assert "dobot_handeye_check = dobot_handeye.handeye_check:main" in handeye_setup
    assert "dobot_handeye_capture = dobot_handeye.handeye_capture:main" in handeye_setup
    assert "dobot_handeye_solve = dobot_handeye.handeye_solve:main" in handeye_setup
    assert "dobot_handeye_validate = dobot_handeye.handeye_validate:main" in handeye_setup
    assert "dobot_handeye_diagnose = dobot_handeye.handeye_diagnose:main" in handeye_setup
    assert "dobot_handeye_tf = dobot_handeye.handeye_tf:main" in handeye_setup
    assert "dobot_handeye_board_tf = dobot_handeye.handeye_board_tf:main" in handeye_setup
    assert "dobot_handeye_check = dobot_ros2.handeye_check:main" not in driver_setup


def test_camera_wrapper_package_launches_official_orbbec_driver():
    package_xml = (CAMERA_PACKAGE_ROOT / "package.xml").read_text()
    setup = (CAMERA_PACKAGE_ROOT / "setup.py").read_text()
    launch = (CAMERA_PACKAGE_ROOT / "launch" / "gemini305.launch.py").read_text()

    assert "<name>dobot_camera</name>" in package_xml
    assert "<exec_depend>orbbec_camera</exec_depend>" in package_xml
    assert "share/{package_name}/launch" in setup
    assert "PythonLaunchDescriptionSource" in launch
    assert "FindPackageShare(\"orbbec_camera\")" in launch
    assert "gemini_330_series.launch.py" in launch
    assert "camera_name" in launch
    assert "serial_number" in launch
    assert "usb_port" in launch


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
