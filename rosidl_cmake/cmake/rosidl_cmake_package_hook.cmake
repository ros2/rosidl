# generate and register extra file for rosidl generation
set(_generated_extra_file
  "${CMAKE_CURRENT_BINARY_DIR}/rosidl_cmake/rosidl_cmake-extras.cmake")
configure_file(
  "${rosidl_cmake_DIR}/rosidl_cmake-extras.cmake.in"
  "${_generated_extra_file}"
  @ONLY
)
list(APPEND ${PROJECT_NAME}_CONFIG_EXTRAS "${_generated_extra_file}")
