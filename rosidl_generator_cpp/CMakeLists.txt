cmake_minimum_required(VERSION 2.8.3)

project(rosidl_generator_cpp)

find_package(ament_cmake REQUIRED)

ament_export_dependencies(rosidl_cmake)

ament_package(
  CONFIG_EXTRAS "rosidl_generator_cpp-extras.cmake"
)

install(
  PROGRAMS bin/rosidl_generator_cpp
  DESTINATION lib/rosidl_generator_cpp
)
install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)
