cmake_minimum_required(VERSION 3.5)

project(rosidl_runtime_cpp)

find_package(ament_cmake REQUIRED)
find_package(rosidl_runtime_c REQUIRED)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${PROJECT_NAME} INTERFACE
  rosidl_runtime_c::rosidl_runtime_c)

if(MSVC)
  target_compile_definitions(${PROJECT_NAME} INTERFACE
  _SILENCE_CXX17_CODECVT_HEADER_DEPRECATION_WARNING)
endif()

# Export old-style CMake variables
ament_export_include_directories("include/${PROJECT_NAME}")

# Export modern CMake targets
ament_export_targets(${PROJECT_NAME})

ament_index_register_resource("rosidl_runtime_packages")

ament_export_dependencies(rosidl_runtime_c)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(performance_test_fixture REQUIRED)
  # Give cppcheck hints about macro definitions coming from outside this package
  get_target_property(ament_cmake_cppcheck_ADDITIONAL_INCLUDE_DIRS
    performance_test_fixture::performance_test_fixture INTERFACE_INCLUDE_DIRECTORIES)

  file(GLOB type_description_includes "include/rosidl_runtime_cpp/type_description/*.hpp")

  set(AMENT_LINT_AUTO_FILE_EXCLUDE ${type_description_includes})
  ament_lint_auto_find_test_dependencies()

  # Run specific linters on copied generated code
  if(ament_cmake_cppcheck_FOUND)
    ament_cppcheck(
      TESTNAME "cppcheck_copied_type_description_interfaces"
      ${type_description_includes})
  endif()
  if(ament_cmake_cpplint_FOUND)
    ament_cpplint(
      TESTNAME "cpplint_copied_type_description_interfaces"
      # the generated code might contain longer lines for templated types
      MAX_LINE_LENGTH 999
      ${type_description_includes})
  endif()
  if(ament_cmake_uncrustify_FOUND)
    ament_uncrustify(
      TESTNAME "uncrustify_copied_type_description_interfaces"
      # the generated code might contain longer lines for templated types
      MAX_LINE_LENGTH 0
      ${type_description_includes})
  endif()

  find_package(ament_cmake_gtest REQUIRED)

  # Default to C++17
  if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
  endif()

  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
  endif()

  ament_add_gtest(test_bounded_vector test/test_bounded_vector.cpp)
  if(TARGET test_bounded_vector)
    target_link_libraries(test_bounded_vector ${PROJECT_NAME})
  endif()

  add_performance_test(benchmark_bounded_vector test/benchmark/benchmark_bounded_vector.cpp)
  if(TARGET benchmark_bounded_vector)
    target_link_libraries(benchmark_bounded_vector ${PROJECT_NAME})
  endif()
endif()

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}
)

ament_package()
