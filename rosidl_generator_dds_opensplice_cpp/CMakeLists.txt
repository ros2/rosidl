cmake_minimum_required(VERSION 2.8.3)

project(rosidl_generator_dds_opensplice_cpp)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_export_dependencies(rosidl_cmake)
ament_export_dependencies(rosidl_generator_dds_idl)

ament_python_install_package(${PROJECT_NAME})

ament_package(
  CONFIG_EXTRAS "rosidl_generator_dds_opensplice_cpp-extras.cmake"
)

install(
  PROGRAMS bin/rosidl_generator_dds_opensplice_cpp
  DESTINATION lib/rosidl_generator_dds_opensplice_cpp
)
install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)
