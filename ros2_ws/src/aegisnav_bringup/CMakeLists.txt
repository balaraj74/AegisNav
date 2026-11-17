cmake_minimum_required(VERSION 3.8)
project(aegisnav_bringup)
find_package(ament_cmake REQUIRED)
install(DIRECTORY launch config worlds maps rviz models
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)
ament_package()
