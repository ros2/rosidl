cmake_minimum_required(VERSION 3.5)

project(rosidl_runtime_cpp)

find_package(ament_cmake REQUIRED)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
  "$<INSTALL_INTERFACE:include>")

ament_export_include_directories(include)
ament_export_targets(${PROJECT_NAME})

ament_index_register_resource("rosidl_runtime_packages")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}
)

ament_package()
