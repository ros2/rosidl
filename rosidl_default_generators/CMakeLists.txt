cmake_minimum_required(VERSION 2.8.3)

project(rosidl_default_generators NONE)

find_package(ament_cmake REQUIRED)

ament_export_dependencies(rosidl_cmake)

ament_export_dependencies(rosidl_generator_c)
ament_export_dependencies(rosidl_generator_cpp)

ament_export_dependencies(rosidl_typesupport_introspection_cpp)

ament_export_dependencies(rosidl_typesupport_connext_cpp)
ament_export_dependencies(rosidl_typesupport_opensplice_cpp)

if(AMENT_ENABLE_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
