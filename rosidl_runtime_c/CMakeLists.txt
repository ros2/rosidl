cmake_minimum_required(VERSION 3.5)

project(rosidl_runtime_c)

find_package(ament_cmake_ros REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
