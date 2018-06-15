cmake_minimum_required(VERSION 3.5)

project(rosidl_generator_py)

find_package(ament_cmake REQUIRED)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(rmw)
ament_export_dependencies(rosidl_cmake)

ament_index_register_resource("rosidl_generator_packages")

ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  find_package(rmw REQUIRED)
  find_package(rosidl_cmake REQUIRED)
  find_package(rosidl_generator_c REQUIRED)

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  set(message_files
    #"msg/Bool.msg"
    #"msg/Byte.msg"
    #"msg/Char.msg"
    "msg/Constants.msg"
    "msg/Empty.msg"
    #"msg/Float32.msg"
    #"msg/Float64.msg"
    #"msg/Int16.msg"
    #"msg/Int32.msg"
    #"msg/Int64.msg"
    #"msg/Int8.msg"
    "msg/Nested.msg"
    "msg/Primitives.msg"
    "msg/StringArrays.msg"
    "msg/Strings.msg"
    #"msg/Uint16.msg"
    #"msg/Uint32.msg"
    #"msg/Uint64.msg"
    #"msg/Uint8.msg"
    "msg/Various.msg"
  )

  include(cmake/register_py.cmake)
  include(cmake/rosidl_generator_py_get_typesupports.cmake)

  set(rosidl_generator_py_DIR "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

  # Need to call extras before get_typesupports, to register the extension
  rosidl_generator_py_extras(
    "${CMAKE_CURRENT_SOURCE_DIR}/bin/rosidl_generator_py"
    "${CMAKE_CURRENT_SOURCE_DIR}/rosidl_generator_py/__init__.py"
    "${CMAKE_CURRENT_SOURCE_DIR}/resource"
  )

  # If no message files were generated, skip the test.
  rosidl_generator_py_get_typesupports(_typesupport_impls)
  if(_typesupport_impls STREQUAL "")
    message(WARNING "No valid typesupport found for Python generator, skipping tests.")
  else()
    rosidl_generate_interfaces(${PROJECT_NAME} ${message_files}
      ADD_LINTER_TESTS
      SKIP_INSTALL
    )

    set(_append_library_dirs "")
    if(WIN32)
      set(_append_library_dirs "${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>")
    endif()

    ament_add_pytest_test(test_interfaces_py "test/test_interfaces.py"
      APPEND_ENV PYTHONPATH="${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_py:${CMAKE_CURRENT_SOURCE_DIR}"
      APPEND_LIBRARY_DIRS "${_append_library_dirs}"
      WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_py"
    )
  endif()
endif()

ament_package(
  CONFIG_EXTRAS "cmake/rosidl_generator_py_get_typesupports.cmake"
    "cmake/register_py.cmake"
    "rosidl_generator_py-extras.cmake.in"
)

install(
  PROGRAMS bin/rosidl_generator_py
  DESTINATION lib/rosidl_generator_py
)
install(
  DIRECTORY cmake resource
  DESTINATION share/${PROJECT_NAME}
)
