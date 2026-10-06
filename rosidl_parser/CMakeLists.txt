cmake_minimum_required(VERSION 3.20)

project(rosidl_parser NONE)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

ament_python_install_package(${PROJECT_NAME})

set(_grammar_cache_file "${CMAKE_CURRENT_BINARY_DIR}/grammar.lark.bin")
add_custom_command(
  OUTPUT "${_grammar_cache_file}"
  COMMAND Python3::Interpreter
    "${CMAKE_CURRENT_SOURCE_DIR}/rosidl_parser/generate_grammar_cache.py"
    "--output" "${_grammar_cache_file}"
  DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/rosidl_parser/grammar.lark"
    "${CMAKE_CURRENT_SOURCE_DIR}/rosidl_parser/generate_grammar_cache.py"
  COMMENT "Generating pre-compiled Lark grammar cache for ${PROJECT_NAME}"
  VERBATIM
)
add_custom_target(${PROJECT_NAME}_grammar_cache ALL
  DEPENDS "${_grammar_cache_file}"
)
install(
  FILES "${_grammar_cache_file}"
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
