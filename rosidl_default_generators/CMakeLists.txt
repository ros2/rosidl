cmake_minimum_required(VERSION 2.8.3)

project(rosidl_default_generators)

find_package(ament_cmake REQUIRED)

ament_export_dependencies(rosidl_cmake)
ament_export_dependencies(rosidl_generator_cpp)
ament_export_dependencies(rosidl_generator_dds_cpp)
ament_export_dependencies(rosidl_generator_dds_idl)

set(_dds_vendor "$ENV{ROS_DDS_IMPLEMENTATION}")
if("${_dds_vendor}" STREQUAL "connext")
  ament_export_dependencies(rosidl_generator_dds_connext_cpp)
elseif("${_dds_vendor}" STREQUAL "opensplice")
  ament_export_dependencies(rosidl_generator_dds_opensplice_cpp)
else()
  message(FATAL_ERROR "Set the environment variable 'ROS_DDS_IMPLEMENTATION' to either 'connext' or 'opensplice'.")
endif()

ament_package()
