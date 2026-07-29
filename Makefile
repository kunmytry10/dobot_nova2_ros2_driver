SHELL := /bin/bash

.PHONY: build driver bringup rviz control-ui control-ui-only system logs-latest services topics tf frames state errors clear enable disable estop drag-start drag-stop recover-limit joints tcp gripper-init gripper-state gripper-open gripper-close gripper-move camera camera-wrist camera-global camera-view camera-topics camera-info camera-check handeye-check handeye-capture handeye-solve handeye-validate handeye-diagnose handeye-tf handeye-board-tf keyboard keyboard-jog keyboard-input keyboard-jog-input keyboard-teleop joy joy-teleop lerobot-setup data-start data-stop data-accept data-reject data-status data-task data-validate data-lerobot-export data-lerobot-validate move-jog jog-stop teach-start teach-stop teach-replay teach-replay-servoj teach-list teach-delete teach-status movej movejp movel movep

WS ?= $(CURDIR)
ORBBEC_WS ?= $(HOME)/orbbec_305
PARAMS ?= $(WS)/src/dobot_ros2/config/dobot_ros2.yaml
ROBOT_IP ?= 192.168.5.1
DASHBOARD_PORT ?= 29999
JOINT ?= 0
U ?= 0
T ?= 0
SPEED ?= 2
ACC ?= 2
WAIT ?= true
TIMEOUT ?= 20.0
J ?= []
P ?= []
AXIS ?=
TRAJ ?=
OVERWRITE ?= false
REPLAY_MODE ?=
CONSOLE_HOST ?= 0.0.0.0
CONSOLE_PORT ?= 8080
GRIPPER_OPENING_MM ?= -1.0
GRIPPER_POS ?= 1000
GRIPPER_FORCE ?= 50
GRIPPER_FORCE_N ?= -1.0
CAMERA_LAUNCH ?= gemini_330_series.launch.py
CAMERA_NAME ?= camera
CAMERA_SERIAL ?=
CAMERA_USB_PORT ?=
CAMERA_SERIAL_ARG = $(if $(strip $(CAMERA_SERIAL)),serial_number:=$(CAMERA_SERIAL),)
CAMERA_USB_PORT_ARG = $(if $(strip $(CAMERA_USB_PORT)),usb_port:=$(CAMERA_USB_PORT),)
DUAL_CAMERA_SERIAL_ARG = $(if $(strip $(CAMERA_SERIAL)),wrist_serial_number:=$(CAMERA_SERIAL),)
DUAL_CAMERA_USB_PORT_ARG = $(if $(strip $(CAMERA_USB_PORT)),wrist_usb_port:=$(CAMERA_USB_PORT),)
REALSENSE_NAME ?= global_camera
REALSENSE_SERIAL ?=
REALSENSE_USB_PORT ?=
REALSENSE_COLOR_PROFILE ?= 640,480,30
REALSENSE_ENABLE_DEPTH ?= false
REALSENSE_SERIAL_ARG = $(if $(strip $(REALSENSE_SERIAL)),serial_no:=$(REALSENSE_SERIAL),)
REALSENSE_USB_PORT_ARG = $(if $(strip $(REALSENSE_USB_PORT)),usb_port_id:=$(REALSENSE_USB_PORT),)
DUAL_REALSENSE_SERIAL_ARG = $(if $(strip $(REALSENSE_SERIAL)),global_serial_no:=$(REALSENSE_SERIAL),)
DUAL_REALSENSE_USB_PORT_ARG = $(if $(strip $(REALSENSE_USB_PORT)),global_usb_port_id:=$(REALSENSE_USB_PORT),)
DATASET ?=
EPISODE ?=
UV ?= $(HOME)/.local/bin/uv
LEROBOT_COMMIT ?= 95211b98f1cd6b638bda84a8d28f9e41323229dd
HANDEYE_DATASET_ROOT ?= handeye_datasets
HANDEYE_DATASET_NAME ?=
HANDEYE_SAMPLES_DIR ?= handeye_samples
HANDEYE_RESULT_FILE ?=
HANDEYE_DIAGNOSE_FILE ?=
HANDEYE_STATIC_TF_FILE ?= $(WS)/handeye_result.yaml
HANDEYE_STATIC_TF_CHILD_FRAME ?= camera_link
HANDEYE_PARENT_FRAME ?= Link6
HANDEYE_CHILD_FRAME ?= camera_color_optical_frame
HANDEYE_METHOD ?= TSAI
KEYBOARD_TOPIC ?= /keyboard/input
KEYBOARD_DEV ?= /dev/input/event0
KEYBOARD_STEP_MM ?= 5.0
KEYBOARD_ROT_STEP_DEG ?= 2.0
KEYBOARD_MOTION_SERVICE ?= movep
KEYBOARD_GRIPPER_INIT ?= true
KEYBOARD_JOG_COORD_TYPE ?= 0
JOY_TOPIC ?= /joy
JOY_DIAGNOSTICS_TOPIC ?= /joy/teleop_diagnostics
JOY_DEV ?= /dev/input/js0
JOY_DEADMAN_BUTTON ?= 4
JOY_ESTOP_BUTTON ?= 1
JOY_TOGGLE_ENABLE_BUTTON ?= 3
JOY_TOGGLE_DRAG_BUTTON ?= 5
JOY_DEADZONE ?= 0.25
JOY_COORD_TYPE ?= 0
JOY_AUTOREPEAT_RATE ?= 50.0
JOY_GRIPPER_INIT ?= true
JOY_GRIPPER_STEP_MM ?= 2.0
JOY_GRIPPER_STOP_LEAD_MM ?= 3.0
JOY_GRIPPER_FORCE ?= 50
JOY_ENABLE_RUMBLE ?= true
JOY_JOINT_LIMIT_MARGIN_DEG ?= 5.0
JOY_X_AXIS_INDEX ?= 1
JOY_Y_AXIS_INDEX ?= 0
JOY_RX_AXIS_INDEX ?= 6
JOY_RY_AXIS_INDEX ?= 7
JOY_X_AXIS_SIGN ?= -1.0
JOY_Y_AXIS_SIGN ?= -1.0
JOY_Z_AXIS_SIGN ?= 1.0
JOY_RZ_AXIS_SIGN ?= -1.0
JOY_RX_AXIS_SIGN ?= -1.0
JOY_RY_AXIS_SIGN ?= -1.0
JOY_START_DATA_COLLECTION ?= true
JOY_DATASET_ROOT ?= $(WS)/data_collection
JOY_WRIST_IMAGE_TOPIC ?= /camera/color/image_raw
JOY_WRIST_CAMERA_INFO_TOPIC ?= /camera/color/camera_info
JOY_GLOBAL_IMAGE_TOPIC ?= /global_camera/color/image_raw
JOY_GLOBAL_CAMERA_INFO_TOPIC ?= /global_camera/color/camera_info
JOY_DATA_SAMPLE_RATE_HZ ?= 10.0
JOY_MAX_IMAGE_SKEW_SEC ?= 0.05
JOY_TASK ?=
JOY_TASK_FILE ?= $(JOY_DATASET_ROOT)/current_task.txt
JOY_LEROBOT_ENABLED ?= true
JOY_LEROBOT_PYTHON ?= $(WS)/.venv-lerobot/bin/python
JOY_LEROBOT_DATASET_ROOT ?= $(JOY_DATASET_ROOT)/lerobot_tape_pi05
JOY_LEROBOT_REPO_ID ?= local/dobot_nova2_tape_pi05
JOY_LEROBOT_EXPORT_TIMEOUT_SEC ?= 900.0
JOY_DATA_REJECT_HOLD_SEC ?= 2.0
JOY_LIMIT_RECOVERY_HOLD_SEC ?= 3.0
JOY_LIMIT_RECOVERY_TIMEOUT_SEC ?= 10.0
ROBOT_MODE ?= bringup
SYSTEM_VIEW ?= false
SYSTEM_START_CAMERA ?= true
SYSTEM_START_JOY ?= true
SYSTEM_START_DATA_COLLECTION ?= true
SYSTEM_INIT_GRIPPER ?= true
SYSTEM_LOG_ROOT ?= $(WS)/logs

ROS_SETUP = source /opt/ros/humble/setup.bash
ORBBEC_ENV = if [ -f "$(ORBBEC_WS)/install/setup.bash" ]; then source "$(ORBBEC_WS)/install/setup.bash"; fi
ROS_ENV = $(ROS_SETUP) && $(ORBBEC_ENV) && cd $(WS) && source install/setup.bash

build:
	$(ROS_SETUP) && $(ORBBEC_ENV) && cd $(WS) && colcon build --symlink-install --packages-up-to dobot_camera dobot_handeye dobot_keyboard dobot_joy dobot_ros2

driver:
	$(ROS_ENV) && ros2 run dobot_ros2 dobot_motion_server --ros-args --params-file $(PARAMS)

bringup:
	$(ROS_ENV) && ros2 launch dobot_ros2 dobot_bringup.launch.py params_file:=$(PARAMS) rviz:=false handeye_result_file:=$(HANDEYE_STATIC_TF_FILE) handeye_output_child_frame:=$(HANDEYE_STATIC_TF_CHILD_FRAME)

rviz:
	$(ROS_ENV) && ros2 launch dobot_ros2 dobot_bringup.launch.py params_file:=$(PARAMS) rviz:=true handeye_result_file:=$(HANDEYE_STATIC_TF_FILE) handeye_output_child_frame:=$(HANDEYE_STATIC_TF_CHILD_FRAME)

control-ui:
	$(ROS_ENV) && ros2 launch dobot_ros2 dobot_control_console.launch.py params_file:=$(PARAMS) console_host:=$(CONSOLE_HOST) console_port:=$(CONSOLE_PORT) start_driver:=true start_state_publisher:=true handeye_result_file:=$(HANDEYE_STATIC_TF_FILE) handeye_output_child_frame:=$(HANDEYE_STATIC_TF_CHILD_FRAME)

control-ui-only:
	$(ROS_ENV) && ros2 launch dobot_ros2 dobot_control_console.launch.py params_file:=$(PARAMS) console_host:=$(CONSOLE_HOST) console_port:=$(CONSOLE_PORT) start_driver:=false start_state_publisher:=false

services:
	$(ROS_ENV) && ros2 service list | grep -E "get_robot_state|get_joint_state|get_tcp_pose|get_gripper_state|clear_error|enable_robot|disable_robot|emergency_stop|drag_start|drag_stop|get_error_id|gripper|teach|move"

topics:
	$(ROS_ENV) && ros2 topic list | grep -E "^/joint_states$$|^/tcp_pose$$|^/dobot_state$$|^/gripper_state$$|^/keyboard/input$$|^/joy$$|^/tf$$|^/tf_static$$"

tf:
	$(ROS_ENV) && ros2 topic list | grep -E "^/tf$$|^/tf_static$$"

frames:
	$(ROS_ENV) && ros2 run tf2_tools view_frames

state:
	$(ROS_ENV) && ros2 service call /get_robot_state dobot_interfaces/srv/GetRobotState "{}"

errors:
	$(ROS_ENV) && ros2 service call /get_error_id std_srvs/srv/Trigger "{}"

clear:
	$(ROS_ENV) && ros2 service call /clear_error std_srvs/srv/Trigger "{}"

enable:
	$(ROS_ENV) && ros2 service call /enable_robot std_srvs/srv/Trigger "{}"

disable:
	$(ROS_ENV) && ros2 service call /disable_robot std_srvs/srv/Trigger "{}"

estop:
	$(ROS_ENV) && ros2 service call /emergency_stop std_srvs/srv/Trigger "{}"

drag-start:
	$(ROS_ENV) && ros2 service call /drag_start std_srvs/srv/Trigger "{}"

drag-stop:
	$(ROS_ENV) && ros2 service call /drag_stop std_srvs/srv/Trigger "{}"

recover-limit:
	$(ROS_ENV) && ros2 run dobot_ros2 dobot_recover_limit --robot-ip $(ROBOT_IP) --port $(DASHBOARD_PORT) --joint "$(JOINT)"

joints:
	$(ROS_ENV) && ros2 service call /get_joint_state dobot_interfaces/srv/GetJointState "{}"

tcp:
	$(ROS_ENV) && ros2 service call /get_tcp_pose dobot_interfaces/srv/GetTcpPose "{}"

gripper-init:
	$(ROS_ENV) && ros2 service call /gripper_init std_srvs/srv/Trigger "{}"

gripper-state:
	$(ROS_ENV) && ros2 service call /get_gripper_state dobot_interfaces/srv/GripperState "{}"

gripper-open:
	$(ROS_ENV) && ros2 service call /gripper_move dobot_interfaces/srv/GripperCommand "{opening_mm: 95.0, force_percent: $(GRIPPER_FORCE), force_n: $(GRIPPER_FORCE_N), wait: $(WAIT), timeout_sec: $(TIMEOUT)}"

gripper-close:
	$(ROS_ENV) && ros2 service call /gripper_move dobot_interfaces/srv/GripperCommand "{opening_mm: 0.0, force_percent: $(GRIPPER_FORCE), force_n: $(GRIPPER_FORCE_N), wait: $(WAIT), timeout_sec: $(TIMEOUT)}"

gripper-move:
	$(ROS_ENV) && ros2 service call /gripper_move dobot_interfaces/srv/GripperCommand "{opening_mm: $(GRIPPER_OPENING_MM), position_permille: $(GRIPPER_POS), force_percent: $(GRIPPER_FORCE), force_n: $(GRIPPER_FORCE_N), wait: $(WAIT), timeout_sec: $(TIMEOUT)}"

camera:
	$(ROS_ENV) && ros2 launch dobot_camera dual_camera.launch.py orbbec_launch_file:=$(CAMERA_LAUNCH) wrist_camera_name:=$(CAMERA_NAME) global_camera_name:=$(REALSENSE_NAME) global_color_profile:=$(REALSENSE_COLOR_PROFILE) global_enable_depth:=$(REALSENSE_ENABLE_DEPTH) $(DUAL_CAMERA_SERIAL_ARG) $(DUAL_CAMERA_USB_PORT_ARG) $(DUAL_REALSENSE_SERIAL_ARG) $(DUAL_REALSENSE_USB_PORT_ARG)

camera-wrist:
	$(ROS_ENV) && ros2 launch dobot_camera gemini305.launch.py orbbec_launch_file:=$(CAMERA_LAUNCH) camera_name:=$(CAMERA_NAME) $(CAMERA_SERIAL_ARG) $(CAMERA_USB_PORT_ARG)

camera-global:
	$(ROS_ENV) && ros2 launch dobot_camera realsense_global.launch.py camera_name:=$(REALSENSE_NAME) rgb_camera.color_profile:=$(REALSENSE_COLOR_PROFILE) enable_depth:=$(REALSENSE_ENABLE_DEPTH) $(REALSENSE_SERIAL_ARG) $(REALSENSE_USB_PORT_ARG)

camera-view:
	$(ROS_ENV) && ros2 launch dobot_camera camera_view.launch.py

camera-topics:
	$(ROS_ENV) && ros2 topic list | grep -E "^/camera/|^/global_camera/"

camera-info:
	$(ROS_ENV) && ros2 topic echo /camera/color/camera_info --once && ros2 topic echo /global_camera/color/camera_info --once

camera-check:
	$(ROS_ENV) && (timeout 6s ros2 topic hz /camera/color/image_raw || true) && (timeout 6s ros2 topic hz /global_camera/color/image_raw || true)

handeye-check:
	$(ROS_ENV) && ros2 run dobot_handeye dobot_handeye_check --ros-args --params-file $(PARAMS)

handeye-capture:
	$(ROS_ENV) && ros2 run dobot_handeye dobot_handeye_capture --dataset-root $(HANDEYE_DATASET_ROOT) --dataset-name "$(HANDEYE_DATASET_NAME)" --ros-args --params-file $(PARAMS)

handeye-solve:
	$(ROS_ENV) && ros2 run dobot_handeye dobot_handeye_solve --dataset "$(DATASET)" --samples-dir $(HANDEYE_SAMPLES_DIR) --result-file "$(HANDEYE_RESULT_FILE)" --parent-frame $(HANDEYE_PARENT_FRAME) --child-frame $(HANDEYE_CHILD_FRAME) --method $(HANDEYE_METHOD)

handeye-validate:
	$(ROS_ENV) && ros2 run dobot_handeye dobot_handeye_validate --dataset "$(DATASET)" --result-file "$(HANDEYE_RESULT_FILE)"

handeye-diagnose:
	$(ROS_ENV) && ros2 run dobot_handeye dobot_handeye_diagnose --dataset "$(DATASET)" --diagnose-file "$(HANDEYE_DIAGNOSE_FILE)"

handeye-tf:
	$(ROS_ENV) && ros2 run dobot_handeye dobot_handeye_tf --dataset "$(DATASET)" --result-file "$(HANDEYE_RESULT_FILE)" --output-child-frame $(HANDEYE_STATIC_TF_CHILD_FRAME)

handeye-board-tf:
	$(ROS_ENV) && ros2 run dobot_handeye dobot_handeye_board_tf --ros-args --params-file $(PARAMS)

keyboard:
	$(ROS_ENV) && if [ "$(KEYBOARD_GRIPPER_INIT)" = "true" ]; then timeout 10s ros2 service call /gripper_init std_srvs/srv/Trigger "{}" || true; fi && ros2 launch dobot_keyboard keyboard_teleop.launch.py params_file:=$(PARAMS) input_topic:=$(KEYBOARD_TOPIC) translation_step_mm:=$(KEYBOARD_STEP_MM) rotation_step_deg:=$(KEYBOARD_ROT_STEP_DEG) motion_service:=$(KEYBOARD_MOTION_SERVICE) speed:=$(SPEED) acceleration:=$(ACC) wait:=$(WAIT) timeout_sec:=$(TIMEOUT)

keyboard-jog:
	$(ROS_ENV) && if [ "$(KEYBOARD_GRIPPER_INIT)" = "true" ]; then timeout 10s ros2 service call /gripper_init std_srvs/srv/Trigger "{}" || true; fi && ros2 run dobot_keyboard dobot_keyboard_jog_runner --params-file $(PARAMS) --input-topic $(KEYBOARD_TOPIC) --device $(KEYBOARD_DEV) --jog-coord-type $(KEYBOARD_JOG_COORD_TYPE) --user $(U) --tool $(T)

keyboard-input:
	$(ROS_ENV) && ros2 run dobot_keyboard dobot_keyboard_input --ros-args -p input_topic:=$(KEYBOARD_TOPIC)

keyboard-jog-input:
	$(ROS_ENV) && ros2 run dobot_keyboard dobot_keyboard_jog_input --ros-args -p input_topic:=$(KEYBOARD_TOPIC) -p device:=$(KEYBOARD_DEV)

keyboard-teleop:
	$(ROS_ENV) && ros2 run dobot_keyboard dobot_keyboard_teleop --ros-args --params-file $(PARAMS) -p keyboard.input_topic:=$(KEYBOARD_TOPIC) -p keyboard.translation_step_mm:=$(KEYBOARD_STEP_MM) -p keyboard.rotation_step_deg:=$(KEYBOARD_ROT_STEP_DEG) -p keyboard.motion_service:=$(KEYBOARD_MOTION_SERVICE) -p keyboard.speed:=$(SPEED) -p keyboard.acceleration:=$(ACC) -p keyboard.wait:=$(WAIT) -p keyboard.timeout_sec:=$(TIMEOUT)

system:
	@set -o pipefail; run_dir="$(SYSTEM_LOG_ROOT)/run_$$(date +%Y%m%d_%H%M%S)_$$BASHPID"; mkdir -p "$$run_dir/ros"; $(ROS_ENV) && export ROS_LOG_DIR="$$run_dir/ros" RCUTILS_COLORIZED_OUTPUT=1 && printf 'system log: %s\n' "$$run_dir" && ros2 launch dobot_ros2 dobot_system.launch.py robot_mode:=$(ROBOT_MODE) start_camera:=$(SYSTEM_START_CAMERA) start_view:=$(SYSTEM_VIEW) start_joy:=$(SYSTEM_START_JOY) start_data_collection:=$(SYSTEM_START_DATA_COLLECTION) init_gripper:=$(SYSTEM_INIT_GRIPPER) run_log_dir:="$$run_dir" params_file:=$(PARAMS) handeye_result_file:=$(HANDEYE_STATIC_TF_FILE) orbbec_launch_file:=$(CAMERA_LAUNCH) wrist_camera_name:=$(CAMERA_NAME) $(if $(strip $(CAMERA_SERIAL)),wrist_serial_number:=$(CAMERA_SERIAL),) $(if $(strip $(CAMERA_USB_PORT)),wrist_usb_port:=$(CAMERA_USB_PORT),) global_camera_name:=$(REALSENSE_NAME) $(if $(strip $(REALSENSE_SERIAL)),global_serial_no:=$(REALSENSE_SERIAL),) $(if $(strip $(REALSENSE_USB_PORT)),global_usb_port_id:=$(REALSENSE_USB_PORT),) global_color_profile:=$(REALSENSE_COLOR_PROFILE) global_enable_depth:=$(REALSENSE_ENABLE_DEPTH) joy_topic:=$(JOY_TOPIC) joy_dev:=$(JOY_DEV) joy_autorepeat_rate:=$(JOY_AUTOREPEAT_RATE) dataset_root:=$(JOY_DATASET_ROOT) wrist_image_topic:=$(JOY_WRIST_IMAGE_TOPIC) wrist_camera_info_topic:=$(JOY_WRIST_CAMERA_INFO_TOPIC) global_image_topic:=$(JOY_GLOBAL_IMAGE_TOPIC) global_camera_info_topic:=$(JOY_GLOBAL_CAMERA_INFO_TOPIC) data_sample_rate_hz:=$(JOY_DATA_SAMPLE_RATE_HZ) max_image_skew_sec:=$(JOY_MAX_IMAGE_SKEW_SEC) task_file:=$(JOY_TASK_FILE) lerobot_enabled:=$(JOY_LEROBOT_ENABLED) lerobot_python:=$(JOY_LEROBOT_PYTHON) lerobot_dataset_root:=$(JOY_LEROBOT_DATASET_ROOT) lerobot_repo_id:=$(JOY_LEROBOT_REPO_ID) lerobot_export_timeout_sec:=$(JOY_LEROBOT_EXPORT_TIMEOUT_SEC) diagnostics_topic:=$(JOY_DIAGNOSTICS_TOPIC) deadman_button_index:=$(JOY_DEADMAN_BUTTON) estop_button_index:=$(JOY_ESTOP_BUTTON) toggle_enable_button_index:=$(JOY_TOGGLE_ENABLE_BUTTON) toggle_drag_button_index:=$(JOY_TOGGLE_DRAG_BUTTON) deadzone:=$(JOY_DEADZONE) coord_type:=$(JOY_COORD_TYPE) x_axis_index:=$(JOY_X_AXIS_INDEX) y_axis_index:=$(JOY_Y_AXIS_INDEX) rx_axis_index:=$(JOY_RX_AXIS_INDEX) ry_axis_index:=$(JOY_RY_AXIS_INDEX) x_axis_sign:=$(JOY_X_AXIS_SIGN) y_axis_sign:=$(JOY_Y_AXIS_SIGN) z_axis_sign:=$(JOY_Z_AXIS_SIGN) rz_axis_sign:=$(JOY_RZ_AXIS_SIGN) rx_axis_sign:=$(JOY_RX_AXIS_SIGN) ry_axis_sign:=$(JOY_RY_AXIS_SIGN) gripper_step_mm:=$(JOY_GRIPPER_STEP_MM) gripper_stop_lead_mm:=$(JOY_GRIPPER_STOP_LEAD_MM) gripper_force_percent:=$(JOY_GRIPPER_FORCE) enable_rumble:=$(JOY_ENABLE_RUMBLE) joint_limit_margin_deg:=$(JOY_JOINT_LIMIT_MARGIN_DEG) limit_recovery_hold_sec:=$(JOY_LIMIT_RECOVERY_HOLD_SEC) limit_recovery_release_timeout_sec:=$(JOY_LIMIT_RECOVERY_TIMEOUT_SEC) data_reject_hold_sec:=$(JOY_DATA_REJECT_HOLD_SEC) 2>&1 | tee "$$run_dir/launch.log"

logs-latest:
	@latest="$$(find "$(SYSTEM_LOG_ROOT)" -mindepth 1 -maxdepth 1 -type d -name 'run_*' -printf '%T@ %p\n' 2>/dev/null | sort -n | tail -n 1 | cut -d' ' -f2-)"; test -n "$$latest" && printf '%s\n' "$$latest" || printf 'no system logs\n'

joy:
	$(ROS_ENV) && if [ "$(JOY_GRIPPER_INIT)" = "true" ]; then timeout 10s ros2 service call /gripper_init std_srvs/srv/Trigger "{}" || true; fi && ros2 launch dobot_joy joy_teleop.launch.py joy_topic:=$(JOY_TOPIC) diagnostics_topic:=$(JOY_DIAGNOSTICS_TOPIC) dev:=$(JOY_DEV) autorepeat_rate:=$(JOY_AUTOREPEAT_RATE) deadman_button_index:=$(JOY_DEADMAN_BUTTON) estop_button_index:=$(JOY_ESTOP_BUTTON) toggle_enable_button_index:=$(JOY_TOGGLE_ENABLE_BUTTON) toggle_drag_button_index:=$(JOY_TOGGLE_DRAG_BUTTON) deadzone:=$(JOY_DEADZONE) coord_type:=$(JOY_COORD_TYPE) gripper_step_mm:=$(JOY_GRIPPER_STEP_MM) gripper_stop_lead_mm:=$(JOY_GRIPPER_STOP_LEAD_MM) gripper_force_percent:=$(JOY_GRIPPER_FORCE) enable_rumble:=$(JOY_ENABLE_RUMBLE) joint_limit_margin_deg:=$(JOY_JOINT_LIMIT_MARGIN_DEG) x_axis_index:=$(JOY_X_AXIS_INDEX) y_axis_index:=$(JOY_Y_AXIS_INDEX) rx_axis_index:=$(JOY_RX_AXIS_INDEX) ry_axis_index:=$(JOY_RY_AXIS_INDEX) x_axis_sign:=$(JOY_X_AXIS_SIGN) y_axis_sign:=$(JOY_Y_AXIS_SIGN) z_axis_sign:=$(JOY_Z_AXIS_SIGN) rz_axis_sign:=$(JOY_RZ_AXIS_SIGN) rx_axis_sign:=$(JOY_RX_AXIS_SIGN) ry_axis_sign:=$(JOY_RY_AXIS_SIGN) start_data_collection:=$(JOY_START_DATA_COLLECTION) dataset_root:=$(JOY_DATASET_ROOT) wrist_image_topic:=$(JOY_WRIST_IMAGE_TOPIC) wrist_camera_info_topic:=$(JOY_WRIST_CAMERA_INFO_TOPIC) global_image_topic:=$(JOY_GLOBAL_IMAGE_TOPIC) global_camera_info_topic:=$(JOY_GLOBAL_CAMERA_INFO_TOPIC) data_sample_rate_hz:=$(JOY_DATA_SAMPLE_RATE_HZ) max_image_skew_sec:=$(JOY_MAX_IMAGE_SKEW_SEC) $(if $(strip $(JOY_TASK)),task_instruction:="$(JOY_TASK)") task_file:=$(JOY_TASK_FILE) lerobot_enabled:=$(JOY_LEROBOT_ENABLED) lerobot_python:=$(JOY_LEROBOT_PYTHON) lerobot_dataset_root:=$(JOY_LEROBOT_DATASET_ROOT) lerobot_repo_id:=$(JOY_LEROBOT_REPO_ID) lerobot_export_timeout_sec:=$(JOY_LEROBOT_EXPORT_TIMEOUT_SEC) limit_recovery_hold_sec:=$(JOY_LIMIT_RECOVERY_HOLD_SEC) limit_recovery_release_timeout_sec:=$(JOY_LIMIT_RECOVERY_TIMEOUT_SEC) data_reject_hold_sec:=$(JOY_DATA_REJECT_HOLD_SEC)

joy-teleop:
	$(ROS_ENV) && ros2 run dobot_joy dobot_joy_teleop --ros-args -p joy.topic:=$(JOY_TOPIC) -p joy.deadman_button_index:=$(JOY_DEADMAN_BUTTON) -p joy.estop_button_index:=$(JOY_ESTOP_BUTTON) -p joy.toggle_enable_button_index:=$(JOY_TOGGLE_ENABLE_BUTTON) -p joy.toggle_drag_button_index:=$(JOY_TOGGLE_DRAG_BUTTON) -p joy.deadzone:=$(JOY_DEADZONE) -p joy.coord_type:=$(JOY_COORD_TYPE) -p joy.gripper_step_mm:=$(JOY_GRIPPER_STEP_MM) -p joy.gripper_stop_lead_mm:=$(JOY_GRIPPER_STOP_LEAD_MM) -p joy.gripper_force_percent:=$(JOY_GRIPPER_FORCE) -p joy.enable_rumble:=$(JOY_ENABLE_RUMBLE) -p joy.joint_limit_margin_deg:=$(JOY_JOINT_LIMIT_MARGIN_DEG) -p joy.data_reject_hold_sec:=$(JOY_DATA_REJECT_HOLD_SEC) -p joy.x_axis_index:=$(JOY_X_AXIS_INDEX) -p joy.y_axis_index:=$(JOY_Y_AXIS_INDEX) -p joy.rx_axis_index:=$(JOY_RX_AXIS_INDEX) -p joy.ry_axis_index:=$(JOY_RY_AXIS_INDEX) -p joy.x_axis_sign:=$(JOY_X_AXIS_SIGN) -p joy.y_axis_sign:=$(JOY_Y_AXIS_SIGN) -p joy.z_axis_sign:=$(JOY_Z_AXIS_SIGN) -p joy.rz_axis_sign:=$(JOY_RZ_AXIS_SIGN) -p joy.rx_axis_sign:=$(JOY_RX_AXIS_SIGN) -p joy.ry_axis_sign:=$(JOY_RY_AXIS_SIGN)

lerobot-setup:
	$(UV) venv --python 3.12 $(WS)/.venv-lerobot
	$(UV) pip install --python $(JOY_LEROBOT_PYTHON) "lerobot[dataset] @ git+https://github.com/huggingface/lerobot.git@$(LEROBOT_COMMIT)"

data-start:
	$(ROS_ENV) && ros2 service call /data_collection/start std_srvs/srv/Trigger "{}"

data-stop:
	$(ROS_ENV) && ros2 service call /data_collection/stop std_srvs/srv/Trigger "{}"

data-accept:
	$(ROS_ENV) && ros2 service call /data_collection/accept std_srvs/srv/Trigger "{}"

data-reject:
	$(ROS_ENV) && ros2 service call /data_collection/reject std_srvs/srv/Trigger "{}"

data-status:
	$(ROS_ENV) && ros2 service call /data_collection/status std_srvs/srv/Trigger "{}"

data-task:
	@mkdir -p "$(dir $(JOY_TASK_FILE))"
	@if [ -n "$(strip $(TASK))" ]; then printf '%s\n' "$(TASK)" > "$(JOY_TASK_FILE)"; fi
	@if [ -f "$(JOY_TASK_FILE)" ]; then printf 'current data task: '; cat "$(JOY_TASK_FILE)"; else printf 'current data task is empty\n'; fi

data-validate:
	$(ROS_ENV) && ros2 run dobot_joy dobot_data_validate "$(EPISODE)"

data-lerobot-export:
	$(JOY_LEROBOT_PYTHON) $(WS)/src/dobot_joy/dobot_joy/lerobot_export.py export "$(EPISODE)" "$(JOY_LEROBOT_DATASET_ROOT)" "$(JOY_LEROBOT_REPO_ID)" "$(JOY_DATA_SAMPLE_RATE_HZ)"

data-lerobot-validate:
	$(JOY_LEROBOT_PYTHON) $(WS)/src/dobot_joy/dobot_joy/lerobot_export.py validate "$(JOY_LEROBOT_DATASET_ROOT)" "$(JOY_LEROBOT_REPO_ID)"

move-jog:
	$(ROS_ENV) && ros2 service call /move_jog dobot_interfaces/srv/JogCommand "{axis_id: '$(AXIS)', stop: false, coord_type: $(JOY_COORD_TYPE), user: $(U), tool: $(T)}"

jog-stop:
	$(ROS_ENV) && ros2 service call /move_jog dobot_interfaces/srv/JogCommand "{stop: true}"

teach-start:
	$(ROS_ENV) && ros2 service call /teach_start dobot_interfaces/srv/TrajectoryCommand "{name: '$(TRAJ)', overwrite: $(OVERWRITE)}"

teach-stop:
	$(ROS_ENV) && ros2 service call /teach_stop dobot_interfaces/srv/TrajectoryCommand "{name: '$(TRAJ)'}"

teach-replay:
	$(ROS_ENV) && ros2 service call /teach_replay dobot_interfaces/srv/TrajectoryCommand "{name: '$(TRAJ)', speed: $(SPEED), acceleration: $(ACC), replay_mode: '$(REPLAY_MODE)', override_wait: true, wait: $(WAIT), timeout_sec: $(TIMEOUT)}"

teach-replay-servoj:
	$(ROS_ENV) && ros2 service call /teach_replay dobot_interfaces/srv/TrajectoryCommand "{name: '$(TRAJ)', speed: $(SPEED), acceleration: $(ACC), replay_mode: 'servoj', override_wait: true, wait: $(WAIT), timeout_sec: $(TIMEOUT)}"

teach-list:
	$(ROS_ENV) && ros2 service call /teach_list dobot_interfaces/srv/TrajectoryList "{}"

teach-delete:
	$(ROS_ENV) && ros2 service call /teach_delete dobot_interfaces/srv/TrajectoryCommand "{name: '$(TRAJ)'}"

teach-status:
	$(ROS_ENV) && ros2 service call /teach_status std_srvs/srv/Trigger "{}"

movej:
	$(ROS_ENV) && ros2 service call /movej dobot_interfaces/srv/MoveCommand "{target: $(J), user: $(U), tool: $(T), speed: $(SPEED), acceleration: $(ACC), wait: $(WAIT), timeout_sec: $(TIMEOUT)}"

movejp:
	$(ROS_ENV) && ros2 service call /movejp dobot_interfaces/srv/MoveCommand "{target: $(P), user: $(U), tool: $(T), speed: $(SPEED), acceleration: $(ACC), wait: $(WAIT), timeout_sec: $(TIMEOUT)}"

movel:
	$(ROS_ENV) && ros2 service call /movel dobot_interfaces/srv/MoveCommand "{target: $(P), user: $(U), tool: $(T), speed: $(SPEED), acceleration: $(ACC), wait: $(WAIT), timeout_sec: $(TIMEOUT)}"

movep:
	$(ROS_ENV) && ros2 service call /movep dobot_interfaces/srv/MoveCommand "{target: $(P), user: $(U), tool: $(T), speed: $(SPEED), acceleration: $(ACC), wait: $(WAIT), timeout_sec: $(TIMEOUT)}"
