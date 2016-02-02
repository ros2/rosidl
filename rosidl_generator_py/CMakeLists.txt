cmake_minimum_required(VERSION 2.8.3)

project(rosidl_generator_py)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_generator_c REQUIRED)

ament_export_dependencies(rosidl_cmake)

ament_python_install_package(${PROJECT_NAME})

if(AMENT_ENABLE_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  set(message_files
    "msg/Bool.msg"
    "msg/Byte.msg"
    "msg/Char.msg"
    "msg/Constants.msg"
    "msg/Empty.msg"
    "msg/Float32.msg"
    "msg/Float64.msg"
    "msg/Int16.msg"
    "msg/Int32.msg"
    "msg/Int64.msg"
    "msg/Int8.msg"
    "msg/Nested.msg"
    "msg/Primitives.msg"
    "msg/Strings.msg"
    "msg/Uint16.msg"
    "msg/Uint32.msg"
    "msg/Uint64.msg"
    "msg/Uint8.msg"
    "msg/Various.msg"
  )

  include(cmake/register_py.cmake)
  set(rosidl_generator_py_DIR "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

  rosidl_generator_py_extras(
    "${CMAKE_CURRENT_SOURCE_DIR}/bin/rosidl_generator_py"
    "${CMAKE_CURRENT_SOURCE_DIR}/rosidl_generator_py/__init__.py"
    "${CMAKE_CURRENT_SOURCE_DIR}/resource"
  )

  rosidl_generate_interfaces(${PROJECT_NAME}_interfaces ${message_files}
    SKIP_INSTALL
  )

  ament_add_nose_test(test_interfaces_py "test/test_interfaces.py"
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_py:${CMAKE_CURRENT_SOURCE_DIR}
    WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_py"
  )
endif()

ament_package(
  CONFIG_EXTRAS "cmake/register_py.cmake" "rosidl_generator_py-extras.cmake.in"
)

install(
  PROGRAMS bin/rosidl_generator_py
  DESTINATION lib/rosidl_generator_py
)
install(
  DIRECTORY cmake resource
  DESTINATION share/${PROJECT_NAME}
)
