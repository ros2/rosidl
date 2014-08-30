# copied from rosidl_generator_cpp/rosidl_generator_cpp-extras.cmake

find_package(ament_cmake_core REQUIRED)
ament_register_extension("rosidl_generate_interfaces" "rosidl_generator_cpp"
  "rosidl_generator_cpp_generate_interfaces.cmake")

set(rosidl_generator_cpp_BIN "${rosidl_generator_cpp_DIR}/../../../lib/rosidl_generator_cpp/rosidl_generator_cpp")
normalize_path(rosidl_generator_cpp_BIN "${rosidl_generator_cpp_BIN}")
set(rosidl_generator_cpp_TEMPLATE_DIR "${rosidl_generator_cpp_DIR}/../resource")
normalize_path(rosidl_generator_cpp_TEMPLATE_DIR "${rosidl_generator_cpp_TEMPLATE_DIR}")
