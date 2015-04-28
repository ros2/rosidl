cmake_minimum_required(VERSION 2.8.3)

project(rosidl_typesupport_introspection_cpp)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_export_dependencies(rosidl_cmake)
# The reason the impl folder is exported is that it contains the
# implementation for the get_*_type_support_handle functions and defines the
# rosidl_typesupport_introspection_cpp specific version of these functions.
ament_export_include_directories(include include/${PROJECT_NAME}/impl)

ament_python_install_package(${PROJECT_NAME})

add_library(${PROJECT_NAME} SHARED src/identifier.cpp)
if(WIN32)
  target_compile_definitions(${PROJECT_NAME}
    PRIVATE "-DROSIDL_TYPESUPPORT_INTROSPECTION_CPP_BUILDING_DLL")
endif()
target_include_directories(${PROJECT_NAME}
  PUBLIC
  include
)
ament_export_libraries(${PROJECT_NAME})

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
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
