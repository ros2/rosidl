cmake_minimum_required(VERSION 3.5)

project(python_cmake_module NONE)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package(
  CONFIG_EXTRAS "python_cmake_module-extras.cmake"
)

install(DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME})
