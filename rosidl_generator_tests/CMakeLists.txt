cmake_minimum_required(VERSION 3.8)
project(rosidl_generator_tests)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Default to C11
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(rosidl_cmake REQUIRED)
  find_package(rosidl_generator_cpp REQUIRED)
  find_package(rosidl_runtime_c REQUIRED)
  find_package(rosidl_runtime_cpp REQUIRED)
  find_package(test_interface_files REQUIRED)
  ament_lint_auto_find_test_dependencies()

  rosidl_generate_interfaces(${PROJECT_NAME}
    ${test_interface_files_MSG_FILES}
    ${test_interface_files_SRV_FILES}
    ADD_LINTER_TESTS
    SKIP_INSTALL
  )

  rosidl_get_typesupport_target(cpp_generator_target "${PROJECT_NAME}" "rosidl_generator_cpp")

  macro(add_generator_cpp_test)
    ament_add_gtest(${test_name} test/rosidl_generator_cpp/${test_name}.cpp)
    if(TARGET ${test_name})
      add_dependencies(${test_name} ${PROJECT_NAME})
      target_link_libraries(${test_name}
        ${cpp_generator_target}
        rosidl_runtime_cpp::rosidl_runtime_cpp
      )
    endif()
  endmacro()

  if(MSVC)
    # https://developercommunity.visualstudio.com/content/problem/919371/c2017-illegal-escape-sequence-when-using-in-a-raw.html
    set_source_files_properties(
      test/rosidl_generator_cpp/test_traits.cpp PROPERTIES COMPILE_FLAGS "/Zc:preprocessor")
  endif()

  set(generator_cpp_test_names
    test_msg_builder
    test_msg_initialization
    test_srv_initialization
    test_interfaces
    test_msg_datatype
    test_name
    test_traits
  )

  foreach(test_name ${generator_cpp_test_names})
    add_generator_cpp_test()
  endforeach()

  rosidl_get_typesupport_target(c_generator_target "${PROJECT_NAME}" "rosidl_generator_c")

  add_executable(
    test_compilation_c
    test/rosidl_generator_c/test_compilation.c
    test/rosidl_generator_c/separate_compilation.c
  )
  ament_add_test(
    test_compilation_c
    COMMAND "$<TARGET_FILE:test_compilation_c>"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  )
  target_link_libraries(test_compilation_c
    ${c_generator_target}
    rosidl_runtime_c::rosidl_runtime_c
  )

  add_executable(test_interfaces_c test/rosidl_generator_c/test_interfaces.c)
  ament_add_test(
    test_interfaces_c
    COMMAND "$<TARGET_FILE:test_interfaces_c>"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  )
  target_link_libraries(test_interfaces_c
    ${c_generator_target}
    rosidl_runtime_c::rosidl_runtime_c
  )

  add_executable(
    test_invalid_initialization_c
    test/rosidl_generator_c/test_invalid_initialization.c
  )
  ament_add_test(
    test_invalid_initialization_c
    COMMAND "$<TARGET_FILE:test_invalid_initialization_c>"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  )
  target_link_libraries(test_invalid_initialization_c
    ${c_generator_target}
    rosidl_runtime_c::rosidl_runtime_c
  )
endif()

ament_package()
