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
    assert "<exec_depend>message_filters</exec_depend>" in package_xml
    assert "dobot_joy_teleop = dobot_joy.joy_teleop:main" in setup
    assert "dobot_data_collection = dobot_joy.data_collection:main" in setup
    assert "dobot_data_validate = dobot_joy.data_validate:main" in setup
    assert "dobot_joy_teleop" in launch
    assert "joy_node" in launch
    assert "deadman_button_index" in launch
    assert "toggle_gripper_button_index" in launch
    assert "toggle_enable_button_index" in launch
    assert "toggle_drag_button_index" in launch
    assert "enable_rumble" in launch
    assert "autorepeat_rate" in launch
    assert "diagnostics_topic" in launch
    assert "start_data_collection" in launch
    assert "dataset_root" in launch
    assert "lerobot_enabled" in launch
    assert "lerobot_dataset_root" in launch
    assert "lerobot_repo_id" in launch
    assert "wrist_image_topic" in launch
    assert "global_image_topic" in launch
    assert "max_image_skew_sec" in launch
    assert "task_file" in launch
    assert "limit_recovery_hold_sec" in launch
    assert "data_reject_hold_sec" in launch
    teleop = (PACKAGE_ROOT / "dobot_joy" / "joy_teleop.py").read_text()
    assert "gripper_stop_pending" in teleop
    assert "msg.grip_state == 2" in teleop
    assert "create_publisher(JoyFeedback," in teleop
    assert "JoyFeedbackArray" not in teleop
    assert '"/joy/teleop_action"' in teleop
    assert "self.gripper_target_mm = None" in teleop
    assert "self.gripper_move_pending = None" in teleop
    assert "MultiThreadedExecutor(num_threads=3)" in teleop
    assert "depth=1" in teleop
    assert "callback_group=self.control_callback_group" in teleop
    assert "if msg.initialized and not msg.moving:" in teleop
    assert "self._initialize_gripper_target(msg.opening_mm)" in teleop
    assert "if self.gripper_target_mm is None:" in teleop
    recorder = (PACKAGE_ROOT / "dobot_joy" / "data_collection.py").read_text()
    assert '"steps.jsonl"' in recorder
    assert '"dobot_teleoperation_episode"' in recorder
    assert '"action"' in recorder
    assert '"format_version": 2' in recorder
    assert '"LeRobotDataset v3.0"' in recorder
    assert "lerobot_export.py" in recorder
    assert "ApproximateTimeSynchronizer" in recorder


def test_makefile_exposes_joy_workflows():
    source = (WORKSPACE_ROOT / "Makefile").read_text()

    assert "joy:" in source
    assert "joy-teleop:" in source
    assert "move-jog:" in source
    assert "jog-stop:" in source
    assert "JOY_TOPIC ?= /joy" in source
    assert "JOY_DEADMAN_BUTTON ?= 4" in source
    assert "JOY_CONTROL_MODE ?= move_jog" in source
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
    assert "JOY_DATASET_ROOT ?= $(DATA_ROOT)/collections/move_jog" in source
    assert "JOY_TASK_FILE ?= $(JOY_DATASET_ROOT)/current_task.txt" in source
    assert "JOY_GLOBAL_IMAGE_TOPIC ?= /global_camera/color/image_raw" in source
    assert "JOY_LEROBOT_ENABLED ?= true" in source
    assert "JOY_LEROBOT_PYTHON ?= $(WS)/.venv-lerobot/bin/python" in source
    assert "lerobot-setup:" in source
    assert "data-lerobot-validate:" in source
    assert "$(if $(strip $(JOY_TASK)),task_instruction:" in source
    assert "JOY_MAX_IMAGE_SKEW_SEC) task_instruction:" not in source
    assert "gripper_init" in source
    assert (
        "--packages-up-to dobot_camera dobot_handeye "
        "dobot_keyboard dobot_joy dobot_ros2"
    ) in source
    assert "ros2 launch dobot_joy joy_teleop.launch.py" in source
    assert "ros2 run dobot_joy dobot_joy_teleop" in source
    assert "ros2 service call /move_jog dobot_interfaces/srv/JogCommand" in source
    assert "data-start:" in source
    assert "data-stop:" in source
    assert "data-status:" in source
    assert "data-accept:" in source
    assert "data-reject:" in source
    assert "data-task:" in source
    assert "data-validate:" in source

    launch = (PACKAGE_ROOT / "launch" / "joy_teleop.launch.py").read_text()
    assert 'DeclareLaunchArgument("control_mode", default_value="move_jog")' in launch


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
