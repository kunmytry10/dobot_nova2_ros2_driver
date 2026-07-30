cmake_minimum_required(VERSION 3.10)
project(dobot_interfaces)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DobotState.msg"
  "msg/GripperStatus.msg"
  "msg/TeleopAction.msg"
  "msg/CartesianServoCommand.msg"
  "srv/GetJointState.srv"
  "srv/GetRobotState.srv"
  "srv/GetTcpPose.srv"
  "srv/GripperCommand.srv"
  "srv/GripperState.srv"
  "srv/JogCommand.srv"
  "srv/LimitRecovery.srv"
  "srv/MoveCommand.srv"
  "srv/TrajectoryCommand.srv"
  "srv/TrajectoryList.srv"
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
