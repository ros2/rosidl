cmake_minimum_required(VERSION 3.20)

project(rosidl_parser NONE)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

ament_python_install_package(${PROJECT_NAME})

set(_standalone_parser_file "${CMAKE_CURRENT_BINARY_DIR}/_standalone_parser.py")
add_custom_command(
  OUTPUT "${_standalone_parser_file}"
  COMMAND Python3::Interpreter
    "${CMAKE_CURRENT_SOURCE_DIR}/rosidl_parser/generate_standalone_parser.py"
    "--output" "${_standalone_parser_file}"
  DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/rosidl_parser/grammar.lark"
    "${CMAKE_CURRENT_SOURCE_DIR}/rosidl_parser/generate_standalone_parser.py"
  COMMENT "Generating standalone Lark parser for ${PROJECT_NAME}"
  VERBATIM
)
add_custom_target(${PROJECT_NAME}_standalone_parser ALL
  DEPENDS "${_standalone_parser_file}"
)
install(
  FILES "${_standalone_parser_file}"
  DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_mypy REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(pytest test)
endif()

install(
  PROGRAMS bin/idl2png
  DESTINATION lib/${PROJECT_NAME})
