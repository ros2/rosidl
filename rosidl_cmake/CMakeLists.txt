cmake_minimum_required(VERSION 2.8.3)

project(rosidl_cmake NONE)

find_package(ament_cmake REQUIRED)

ament_package(
  CONFIG_EXTRAS "rosidl_cmake-extras.cmake"
)

install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)
