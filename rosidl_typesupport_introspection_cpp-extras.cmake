# copied from rosidl_typesupport_introspection_cpp/rosidl_typesupport_introspection_cpp-extras.cmake

find_package(ament_cmake_core REQUIRED)
# TODO
# instead of being an extension for "rosidl_generate_interfaces"
# this should be an extension of "rosidl_generator_cpp"
# which can then ensure that there is only one
ament_register_extension("rosidl_generate_interfaces" "rosidl_typesupport_introspection_cpp"
  "rosidl_typesupport_introspection_cpp_generate_interfaces.cmake")

set(rosidl_typesupport_introspection_cpp_BIN "${rosidl_typesupport_introspection_cpp_DIR}/../../../lib/rosidl_typesupport_introspection_cpp/rosidl_typesupport_introspection_cpp")
set(rosidl_typesupport_introspection_cpp_TEMPLATE_DIR "${rosidl_typesupport_introspection_cpp_DIR}/../resource")
