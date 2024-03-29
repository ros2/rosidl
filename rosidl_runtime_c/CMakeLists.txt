cmake_minimum_required(VERSION 3.5)

project(rosidl_runtime_c C)

# Default to C11
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_C_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_ros REQUIRED)
find_package(rcutils REQUIRED)
find_package(rosidl_typesupport_interface REQUIRED)

file(GLOB type_description_sources "src/type_description/*.c")
file(GLOB type_description_includes "include/rosidl_runtime_c/type_description/*.h")
add_library(${PROJECT_NAME}
  "src/message_type_support.c"
  "src/primitives_sequence_functions.c"
  "src/sequence_bound.c"
  "src/service_type_support.c"
  "src/string_functions.c"
  "src/type_hash.c"
  "src/u16string_functions.c"
  "src/type_description_utils.c"
  ${type_description_sources}
)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${PROJECT_NAME} PUBLIC
  rcutils::rcutils
  rosidl_typesupport_interface::rosidl_typesupport_interface
)
if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_C_COMPILER_ID MATCHES "Clang")
  set_target_properties(${PROJECT_NAME} PROPERTIES
    COMPILE_OPTIONS -Wall -Wextra -Wpedantic)
endif()
if(WIN32)
  target_compile_definitions(${PROJECT_NAME}
    PRIVATE "ROSIDL_GENERATOR_C_BUILDING_DLL")
endif()

if(BUILD_TESTING AND NOT RCUTILS_DISABLE_FAULT_INJECTION)
  target_compile_definitions(${PROJECT_NAME} PUBLIC RCUTILS_ENABLE_FAULT_INJECTION)
endif()

ament_export_dependencies(rcutils rosidl_typesupport_interface)

# Export old-style CMake variables
ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(${PROJECT_NAME})

# Export modern CMake targets
ament_export_targets(${PROJECT_NAME})

ament_index_register_resource("rosidl_runtime_packages")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(performance_test_fixture REQUIRED)
  # Give cppcheck hints about macro definitions coming from outside this package
  get_target_property(ament_cmake_cppcheck_ADDITIONAL_INCLUDE_DIRS performance_test_fixture::performance_test_fixture
    INTERFACE_INCLUDE_DIRECTORIES)

  set(AMENT_LINT_AUTO_FILE_EXCLUDE ${type_description_includes} ${type_description_sources})
  ament_lint_auto_find_test_dependencies()

  # Run specific linters on copied generated code
  if(ament_cmake_cppcheck_FOUND)
    ament_cppcheck(
      TESTNAME "cppcheck_copied_type_description_interfaces"
      ${type_description_includes} ${type_description_sources})
  endif()
  if(ament_cmake_cpplint_FOUND)
    ament_cpplint(
      TESTNAME "cpplint_copied_type_description_interfaces"
      # the generated code might contain longer lines for templated types
      MAX_LINE_LENGTH 999
      ${type_description_includes} ${type_description_sources})
  endif()
  if(ament_cmake_uncrustify_FOUND)
    ament_uncrustify(
      TESTNAME "uncrustify_copied_type_description_interfaces"
      # the generated code might contain longer lines for templated types
      MAX_LINE_LENGTH 0
      ${type_description_includes} ${type_description_sources})
  endif()

  # For gtest
  enable_language(CXX)
  # Default to C++17
  if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
  endif()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_message_type_support test/test_message_type_support.cpp)
  if(TARGET test_message_type_support)
    target_link_libraries(test_message_type_support ${PROJECT_NAME})
  endif()

  ament_add_gtest(test_primitives_sequence_functions test/test_primitives_sequence_functions.cpp)
  if(TARGET test_primitives_sequence_functions)
    target_link_libraries(test_primitives_sequence_functions ${PROJECT_NAME})
  endif()

  ament_add_gtest(test_sequence_bound test/test_sequence_bound.cpp)
  if(TARGET test_sequence_bound)
    target_link_libraries(test_sequence_bound ${PROJECT_NAME})
  endif()

  ament_add_gtest(test_service_type_support test/test_service_type_support.cpp)
  if(TARGET test_service_type_support)
    target_link_libraries(test_service_type_support ${PROJECT_NAME})
  endif()

  ament_add_gtest(test_string_functions test/test_string_functions.cpp)
  if(TARGET test_string_functions)
    target_link_libraries(test_string_functions ${PROJECT_NAME})
    target_compile_definitions(test_string_functions PUBLIC RCUTILS_ENABLE_FAULT_INJECTION)
  endif()

  ament_add_gtest(test_type_hash test/test_type_hash.cpp)
  if(TARGET test_type_hash)
    target_link_libraries(test_type_hash ${PROJECT_NAME})
  endif()

  ament_add_gtest(test_u16string_functions test/test_u16string_functions.cpp)
  if(TARGET test_u16string_functions)
    target_link_libraries(test_u16string_functions ${PROJECT_NAME})
    target_compile_definitions(test_u16string_functions PUBLIC RCUTILS_ENABLE_FAULT_INJECTION)
  endif()

  ament_add_gtest(test_type_description_utils test/test_type_description_utils.cpp)
  if(TARGET test_type_description_utils)
    target_link_libraries(test_type_description_utils ${PROJECT_NAME})
  endif()

  add_performance_test(benchmark_string_conversion test/benchmark/benchmark_string_conversion.cpp)
  if(TARGET benchmark_string_conversion)
    target_link_libraries(benchmark_string_conversion ${PROJECT_NAME})
  endif()
endif()

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
