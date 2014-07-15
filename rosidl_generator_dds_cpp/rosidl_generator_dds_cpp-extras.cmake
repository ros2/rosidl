# copied from rosidl_generator_dds_cpp/rosidl_generator_dds_cpp-extras.cmake

find_package(ament_cmake_core REQUIRED)
ament_register_extension("rosidl_generate_interfaces" "rosidl_generator_dds_cpp"
  "rosidl_generator_dds_cpp_generate_interfaces.cmake")

set(rosidl_generator_dds_cpp_BIN "${rosidl_generator_dds_cpp_DIR}/../../../lib/rosidl_generator_dds_cpp/rosidl_generator_dds_cpp")
set(rosidl_generator_dds_cpp_TEMPLATE_DIR "${rosidl_generator_dds_cpp_DIR}/../resource")
