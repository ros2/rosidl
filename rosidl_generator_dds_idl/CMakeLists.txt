cmake_minimum_required(VERSION 2.8.3)

project(rosidl_generator_dds_idl)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_export_dependencies(rosidl_cmake)

ament_python_install_package(${PROJECT_NAME})

ament_package(
  CONFIG_EXTRAS "rosidl_generator_dds_idl-extras.cmake"
)

install(
  PROGRAMS bin/rosidl_generator_dds_idl
  DESTINATION lib/rosidl_generator_dds_idl
)
install(
  DIRECTORY cmake resource
  DESTINATION share/${PROJECT_NAME}
)
