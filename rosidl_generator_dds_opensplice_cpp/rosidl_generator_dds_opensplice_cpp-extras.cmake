# copied from rosidl_generator_dds_opensplice_cpp/rosidl_generator_dds_opensplice_cpp-extras.cmake

find_package(ament_cmake_core REQUIRED)
ament_register_extension("rosidl_generate_interfaces" "rosidl_generator_dds_opensplice_cpp"
  "rosidl_generator_dds_opensplice_cpp_generate_interfaces.cmake")

find_package(opensplice REQUIRED COMPONENTS CXX)

set(rosidl_generator_dds_opensplice_cpp_BIN "${rosidl_generator_dds_opensplice_cpp_DIR}/../../../lib/rosidl_generator_dds_opensplice_cpp/rosidl_generator_dds_opensplice_cpp")
