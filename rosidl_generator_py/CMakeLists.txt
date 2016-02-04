cmake_minimum_required(VERSION 2.8.3)

project(rosidl_generator_py)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_generator_c REQUIRED)

ament_export_dependencies(rosidl_cmake)

ament_python_install_package(${PROJECT_NAME})

ament_package(
  CONFIG_EXTRAS "cmake/register_py.cmake" "rosidl_generator_py-extras.cmake.in"
)

install(
  PROGRAMS bin/rosidl_generator_py
  DESTINATION lib/rosidl_generator_py
)
install(
  DIRECTORY cmake resource
  DESTINATION share/${PROJECT_NAME}
)
