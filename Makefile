SHELL := /bin/bash

.PHONY: build driver bringup rviz control-ui control-ui-only services topics tf frames state errors clear enable disable estop joints tcp gripper-init gripper-state gripper-open gripper-close gripper-move teach-start teach-stop teach-replay teach-replay-servoj teach-list teach-delete teach-status movej movejp movel movep

WS ?= $(CURDIR)
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

ROS_SETUP = source /opt/ros/humble/setup.bash
ROS_ENV = $(ROS_SETUP) && cd $(WS) && source install/setup.bash

build:
	$(ROS_SETUP) && cd $(WS) && colcon build --symlink-install --packages-up-to dobot_ros2

driver:
	$(ROS_ENV) && ros2 run dobot_ros2 dobot_motion_server --ros-args --params-file $(PARAMS)

bringup:
	$(ROS_ENV) && ros2 launch dobot_ros2 dobot_bringup.launch.py params_file:=$(PARAMS) rviz:=false

rviz:
	$(ROS_ENV) && ros2 launch dobot_ros2 dobot_bringup.launch.py params_file:=$(PARAMS) rviz:=true

control-ui:
	$(ROS_ENV) && ros2 launch dobot_ros2 dobot_control_console.launch.py params_file:=$(PARAMS) console_host:=$(CONSOLE_HOST) console_port:=$(CONSOLE_PORT) start_driver:=true start_state_publisher:=true

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
