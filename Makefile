SHELL := /bin/bash

.PHONY: build driver bringup rviz services tf frames state errors clear enable disable joints tcp movej movejp movel movep

WS ?= /home/ros/ws
PARAMS ?= $(WS)/src/dobot_ros2/config/dobot_ros2.yaml
U ?= 0
T ?= 0
SPEED ?= 2
ACC ?= 2
WAIT ?= true
TIMEOUT ?= 20.0
J ?= []
P ?= []

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

services:
	$(ROS_ENV) && ros2 service list | grep -E "get_robot_state|get_joint_state|get_tcp_pose|clear_error|enable_robot|disable_robot|get_error_id|move"

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

joints:
	$(ROS_ENV) && ros2 service call /get_joint_state dobot_interfaces/srv/GetJointState "{}"

tcp:
	$(ROS_ENV) && ros2 service call /get_tcp_pose dobot_interfaces/srv/GetTcpPose "{}"

movej:
	$(ROS_ENV) && ros2 service call /movej dobot_interfaces/srv/MoveCommand "{target: $(J), user: $(U), tool: $(T), speed: $(SPEED), acceleration: $(ACC), wait: $(WAIT), timeout_sec: $(TIMEOUT)}"

movejp:
	$(ROS_ENV) && ros2 service call /movejp dobot_interfaces/srv/MoveCommand "{target: $(P), user: $(U), tool: $(T), speed: $(SPEED), acceleration: $(ACC), wait: $(WAIT), timeout_sec: $(TIMEOUT)}"

movel:
	$(ROS_ENV) && ros2 service call /movel dobot_interfaces/srv/MoveCommand "{target: $(P), user: $(U), tool: $(T), speed: $(SPEED), acceleration: $(ACC), wait: $(WAIT), timeout_sec: $(TIMEOUT)}"

movep:
	$(ROS_ENV) && ros2 service call /movep dobot_interfaces/srv/MoveCommand "{target: $(P), user: $(U), tool: $(T), speed: $(SPEED), acceleration: $(ACC), wait: $(WAIT), timeout_sec: $(TIMEOUT)}"
