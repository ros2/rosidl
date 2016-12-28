cmake_minimum_required(VERSION 3.5)

project(rosidl_typesupport_introspection_c)

if(NOT WIN32)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c11 -Wall -Wextra")
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_export_dependencies(rosidl_cmake)
ament_export_dependencies(rosidl_generator_c)
# The reason the impl folder is exported is that it contains the implementation
# for the get_*_type_support_handle functions and defines the opensplice
# specific version of these functions.

ament_export_include_directories(include)

ament_python_install_package(${PROJECT_NAME})

add_library(${PROJECT_NAME} SHARED src/identifier.c)
if(WIN32)
  target_compile_definitions(${PROJECT_NAME}
    PRIVATE "ROSIDL_TYPESUPPORT_INTROSPECTION_C_BUILDING_DLL")
endif()
target_include_directories(${PROJECT_NAME}
  PUBLIC
  include
)
ament_export_libraries(${PROJECT_NAME})

ament_index_register_resource("rosidl_typesupport_c")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package(
  CONFIG_EXTRAS "rosidl_typesupport_introspection_c-extras.cmake.in"
)

install(
  PROGRAMS bin/rosidl_typesupport_introspection_c
  DESTINATION lib/rosidl_typesupport_introspection_c
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
