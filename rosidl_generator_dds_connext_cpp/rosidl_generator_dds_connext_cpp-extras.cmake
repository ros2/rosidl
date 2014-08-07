# copied from rosidl_generator_dds_connext_cpp/rosidl_generator_dds_connext_cpp-extras.cmake

find_package(ament_cmake_core REQUIRED)
ament_register_extension("rosidl_generate_interfaces" "rosidl_generator_dds_connext_cpp"
  "rosidl_generator_dds_connext_cpp_generate_interfaces.cmake")

find_package(ndds_cpp REQUIRED)

set(CONNEXT_DDSGEN2 "/usr/bin/rtiddsgen2")
set(CONNEXT_INCLUDE_DIRS ${ndds_cpp_INCLUDE_DIRS})
set(CONNEXT_LIBRARIES ${ndds_cpp_LIBRARIES})
set(CONNEXT_DEFINITIONS ${ndds_cpp_DEFINITIONS})

set(rosidl_generator_dds_connext_cpp_BIN "${rosidl_generator_dds_connext_cpp_DIR}/../../../lib/rosidl_generator_dds_connext_cpp/rosidl_generator_dds_connext_cpp")
set(rosidl_generator_dds_connext_cpp_TEMPLATE_DIR "${rosidl_generator_dds_connext_cpp_DIR}/../resource")
