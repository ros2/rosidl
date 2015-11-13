cmake_minimum_required(VERSION 2.8.3)

project(rosidl_generator_c C)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_export_dependencies(rosidl_cmake)
ament_export_include_directories(include)

ament_python_install_package(${PROJECT_NAME})

if(AMENT_ENABLE_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  set(message_files
    "msg/Empty.msg"
  )

  include(cmake/register_c.cmake)
  set(rosidl_generator_c_DIR "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

  rosidl_generator_c_extras(
    "${CMAKE_CURRENT_SOURCE_DIR}/bin/rosidl_generator_c"
    "${CMAKE_CURRENT_SOURCE_DIR}/rosidl_generator_c/__init__.py"
    "${CMAKE_CURRENT_SOURCE_DIR}/resource"
  )

  rosidl_generate_interfaces(${PROJECT_NAME} ${message_files}
    SKIP_INSTALL
  )

  add_executable(test_compilation_c test/test_compilation.c)
  if(TARGET test_interfaces_c)
    add_dependencies(test_interfaces_c ${PROJECT_NAME})
  endif()
  # include the built files directly, instead of their install location
  include_directories(include "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}")
endif()

ament_package(
  CONFIG_EXTRAS "rosidl_generator_c-extras.cmake.in"
)

install(
  PROGRAMS bin/rosidl_generator_c
  DESTINATION lib/rosidl_generator_c
)
install(
  DIRECTORY cmake resource
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY include/
  DESTINATION include
)
