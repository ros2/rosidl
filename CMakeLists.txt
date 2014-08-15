cmake_minimum_required(VERSION 2.8.3)

set(CMAKE_CXX_FLAGS "-std=c++0x")

project(rosidl_typesupport_introspection_cpp)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_export_dependencies(rosidl_cmake)
ament_export_include_directories(include include/${PROJECT_NAME}/impl)

ament_python_install_package(${PROJECT_NAME})

include_directories(include)
add_library(rosidl_typesupport_introspection_cpp SHARED src/identifier.cpp)
ament_export_libraries(rosidl_typesupport_introspection_cpp)

ament_package(
  CONFIG_EXTRAS "rosidl_typesupport_introspection_cpp-extras.cmake"
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
install(
  TARGETS rosidl_typesupport_introspection_cpp
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
