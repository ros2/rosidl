cmake_minimum_required(VERSION 2.8.3)

project(rosidl_typesupport_introspection_cpp NONE)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_export_dependencies(rosidl_cmake)
# The reason the impl folder is exported is that it contains the implementation
# for the get_type_support_handle function and defines the opensplice specific
# version of this function.
ament_export_include_directories(include include/${PROJECT_NAME}/impl)

ament_python_install_package(${PROJECT_NAME})

include_directories(include)

if(AMENT_ENABLE_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package(
  CONFIG_EXTRAS "rosidl_typesupport_introspection_cpp-extras.cmake.in"
)

install(
  PROGRAMS bin/rosidl_typesupport_introspection_cpp
  DESTINATION lib/rosidl_typesupport_introspection_cpp
)
install(
  DIRECTORY cmake resource
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY include/
  DESTINATION include
)
