SHELL := /bin/bash

.PHONY: build driver bringup rviz control-ui control-ui-only services topics tf frames state errors clear enable disable estop joints tcp gripper-init gripper-state gripper-open gripper-close gripper-move camera camera-topics camera-info handeye-check handeye-capture handeye-solve handeye-validate handeye-diagnose handeye-tf handeye-board-tf keyboard keyboard-input keyboard-teleop teach-start teach-stop teach-replay teach-replay-servoj teach-list teach-delete teach-status movej movejp movel movep

WS ?= $(CURDIR)
ORBBEC_WS ?= $(HOME)/orbbec_305
PARAMS ?= $(WS)/src/dobot_ros2/config/dobot_ros2.yaml
U ?= 0
T ?= 0
SPEED ?= 2
ACC ?= 2
WAIT ?= true
TIMEOUT ?= 20.0
J ?= []
P ?= []
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
DATASET ?=
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
KEYBOARD_STEP_MM ?= 5.0
KEYBOARD_ROT_STEP_DEG ?= 2.0
KEYBOARD_MOTION_SERVICE ?= movep

ROS_SETUP = source /opt/ros/humble/setup.bash
ORBBEC_ENV = if [ -f "$(ORBBEC_WS)/install/setup.bash" ]; then source "$(ORBBEC_WS)/install/setup.bash"; fi
ROS_ENV = $(ROS_SETUP) && $(ORBBEC_ENV) && cd $(WS) && source install/setup.bash

build:
	$(ROS_SETUP) && $(ORBBEC_ENV) && cd $(WS) && colcon build --symlink-install --packages-up-to dobot_camera dobot_handeye dobot_keyboard dobot_ros2

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
	$(ROS_ENV) && ros2 service list | grep -E "get_robot_state|get_joint_state|get_tcp_pose|get_gripper_state|clear_error|enable_robot|disable_robot|emergency_stop|get_error_id|gripper|teach|move"

topics:
	$(ROS_ENV) && ros2 topic list | grep -E "^/joint_states$$|^/tcp_pose$$|^/dobot_state$$|^/gripper_state$$|^/tf$$|^/tf_static$$"

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
	$(ROS_ENV) && ros2 launch dobot_camera gemini305.launch.py orbbec_launch_file:=$(CAMERA_LAUNCH) camera_name:=$(CAMERA_NAME) $(CAMERA_SERIAL_ARG) $(CAMERA_USB_PORT_ARG)

camera-topics:
	$(ROS_ENV) && ros2 topic list | grep -E "^/camera/"

camera-info:
	$(ROS_ENV) && ros2 topic echo /camera/color/camera_info --once

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
	$(ROS_ENV) && ros2 launch dobot_keyboard keyboard_teleop.launch.py params_file:=$(PARAMS) input_topic:=$(KEYBOARD_TOPIC) translation_step_mm:=$(KEYBOARD_STEP_MM) rotation_step_deg:=$(KEYBOARD_ROT_STEP_DEG) motion_service:=$(KEYBOARD_MOTION_SERVICE) speed:=$(SPEED) acceleration:=$(ACC) wait:=$(WAIT) timeout_sec:=$(TIMEOUT)

keyboard-input:
	$(ROS_ENV) && ros2 run dobot_keyboard dobot_keyboard_input --ros-args -p input_topic:=$(KEYBOARD_TOPIC)

keyboard-teleop:
	$(ROS_ENV) && ros2 run dobot_keyboard dobot_keyboard_teleop --ros-args --params-file $(PARAMS) -p keyboard.input_topic:=$(KEYBOARD_TOPIC) -p keyboard.translation_step_mm:=$(KEYBOARD_STEP_MM) -p keyboard.rotation_step_deg:=$(KEYBOARD_ROT_STEP_DEG) -p keyboard.motion_service:=$(KEYBOARD_MOTION_SERVICE) -p keyboard.speed:=$(SPEED) -p keyboard.acceleration:=$(ACC) -p keyboard.wait:=$(WAIT) -p keyboard.timeout_sec:=$(TIMEOUT)

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
