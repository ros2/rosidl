cmake_minimum_required(VERSION 3.5)

project(rosidl_runtime_cpp)

find_package(ament_cmake REQUIRED)

ament_export_dependencies(rosidl_cmake)
ament_export_include_directories(include)

ament_index_register_resource("rosidl_runtime_packages")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
