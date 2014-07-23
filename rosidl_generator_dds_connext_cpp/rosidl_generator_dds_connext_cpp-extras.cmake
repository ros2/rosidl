# copied from rosidl_generator_dds_connext_cpp/rosidl_generator_dds_connext_cpp-extras.cmake

find_package(ament_cmake_core REQUIRED)
ament_register_extension("rosidl_generate_interfaces" "rosidl_generator_dds_connext_cpp"
  "rosidl_generator_dds_connext_cpp_generate_interfaces.cmake")

#find_package(connext REQUIRED)
set(CONNEXT_DDSGEN2 "/usr/bin/rtiddsgen2")
set(CONNEXT_INCLUDE_DIRS "/usr/include/ndds")
set(CONNEXT_LIBRARIES "/usr/lib/libndds_c.so.5.1.0")

set(rosidl_generator_dds_connext_cpp_BIN "${rosidl_generator_dds_connext_cpp_DIR}/../../../lib/rosidl_generator_dds_connext_cpp/rosidl_generator_dds_connext_cpp")
