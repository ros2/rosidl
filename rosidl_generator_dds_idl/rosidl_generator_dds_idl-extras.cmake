# copied from rosidl_generator_dds_idl/rosidl_generator_dds_idl-extras.cmake

find_package(ament_cmake_core REQUIRED)
ament_register_extension("rosidl_generate_interfaces" "rosidl_generator_dds_idl"
  "rosidl_generator_dds_idl_generate_interfaces.cmake")

set(rosidl_generator_dds_idl_BIN "${rosidl_generator_dds_idl_DIR}/../../../lib/rosidl_generator_dds_idl/rosidl_generator_dds_idl")
set(rosidl_generator_dds_idl_TEMPLATE_DIR "${rosidl_generator_dds_idl_DIR}/../resource")
