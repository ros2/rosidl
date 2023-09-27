cmake_minimum_required(VERSION 3.5)

project(rosidl_cmake NONE)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

set(rosidl_cmake_generate_interfaces_BIN "${CMAKE_CURRENT_SOURCE_DIR}/bin/rosidl_cmake_generate_interfaces")
normalize_path(rosidl_cmake_generate_interfaces_BIN "${rosidl_cmake_generate_interfaces_BIN}")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package(
  CONFIG_EXTRAS "rosidl_cmake-extras.cmake"
)

install(
  PROGRAMS bin/rosidl_cmake_generate_interfaces
  DESTINATION lib/rosidl_cmake
)

install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)
