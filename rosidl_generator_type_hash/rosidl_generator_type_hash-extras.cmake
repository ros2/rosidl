include("${CMAKE_CURRENT_LIST_DIR}/register_type_hash.cmake")
rosidl_generator_type_hash_extras(
  "${rosidl_generator_type_hash_DIR}/../../../lib/rosidl_generator_type_hash/rosidl_generator_type_hash"
  "${rosidl_generator_type_hash_DIR}/../../../@PYTHON_INSTALL_DIR@/rosidl_generator_type_hash/__init__.py"
  "${rosidl_generator_type_hash_DIR}/../resource"
)
