macro(rosidl_generator_cpp_extras BIN GENERATOR_FILES TEMPLATE_DIR)
  find_package(ament_cmake_core QUIET REQUIRED)
  ament_register_extension(
    "rosidl_generate_interfaces"
    "rosidl_generator_cpp"
    "rosidl_generator_cpp_generate_interfaces.cmake")

  normalize_path(BIN "${BIN}")
  set(rosidl_generator_cpp_BIN "${BIN}")

  normalize_path(GENERATOR_FILES "${GENERATOR_FILES}")
  set(rosidl_generator_cpp_GENERATOR_FILES "${GENERATOR_FILES}")

  normalize_path(TEMPLATE_DIR "${TEMPLATE_DIR}")
  set(rosidl_generator_cpp_TEMPLATE_DIR "${TEMPLATE_DIR}")
endmacro()
