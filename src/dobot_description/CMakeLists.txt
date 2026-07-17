cmake_minimum_required(VERSION 3.10)
project(dobot_description)

find_package(ament_cmake REQUIRED)

install(FILES
  urdf/nova2_robot.urdf
  DESTINATION share/${PROJECT_NAME}/urdf
)

install(DIRECTORY
  meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
