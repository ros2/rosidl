cmake_minimum_required(VERSION 3.20)
project(rosidl_typesupport_introspection_tests)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_ros_core REQUIRED)
  ament_lint_auto_find_test_dependencies()

  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
  endif()
  if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    # When building under Clang, the "TYPED_TEST_SUITE" macros in the tests print the following
    # warning: "must specify at least one argument for '...' parameter of variadic macro".
    # Since this isn't really a problem, and will be allowed under C++20, just disable the warning.
    add_compile_options(-Wno-gnu-zero-variadic-macro-arguments)
  endif()

  find_package(rosidl_cmake REQUIRED)
  find_package(rosidl_generator_c REQUIRED)
  find_package(rosidl_generator_cpp REQUIRED)
  find_package(rosidl_typesupport_introspection_c REQUIRED)
  find_package(rosidl_typesupport_introspection_cpp REQUIRED)
  find_package(test_interface_files REQUIRED)

  # Drop BoundedPlainSequences as BoundedSequences
  # already encompasses all the same member types
  list(
    FILTER test_interface_files_MSG_FILES
    EXCLUDE REGEX ".*BoundedPlainSequences.*")
  rosidl_generate_interfaces(${PROJECT_NAME}
    ${test_interface_files_MSG_FILES}
    ${test_interface_files_SRV_FILES}
    # ${test_interface_files_ACTION_FILES}
    SKIP_INSTALL
  )

  find_package(rosidl_buffer REQUIRED)
  find_package(rcutils REQUIRED)
  find_package(rcpputils REQUIRED)
  find_package(rosidl_typesupport_interface REQUIRED)

  add_library(${PROJECT_NAME}_library INTERFACE)
  target_include_directories(${PROJECT_NAME}_library INTERFACE include/)
  target_link_libraries(${PROJECT_NAME}_library INTERFACE
    rosidl_buffer::rosidl_buffer
    rcutils::rcutils
    rcpputils::rcpputils
    rosidl_typesupport_interface::rosidl_typesupport_interface
    rosidl_typesupport_introspection_c::rosidl_typesupport_introspection_c
    rosidl_typesupport_introspection_cpp::rosidl_typesupport_introspection_cpp
    # Use the ROS default C++ standard (C++20)
    ament_cmake_ros_core::ament_ros_cxx_standard)

  find_package(ament_cmake_gtest REQUIRED)
  file(GLOB test_files test/test_*.cpp)
  foreach(test_file ${test_files})
    get_filename_component(target ${test_file} NAME_WE)

    ament_add_gtest(${target} ${test_file}
      # Ensure typesupport introspection libraries can be found
      APPEND_LIBRARY_DIRS "${CMAKE_CURRENT_BINARY_DIR}"
    )

    if(TARGET ${target})
      target_include_directories(${target} PRIVATE test)
      target_link_libraries(${target}
        ${PROJECT_NAME}_library
        ${PROJECT_NAME}__rosidl_generator_c
        ${PROJECT_NAME}__rosidl_generator_cpp
        ${PROJECT_NAME}__rosidl_typesupport_introspection_c
        ${PROJECT_NAME}__rosidl_typesupport_introspection_cpp)
    endif()
  endforeach()
endif()

ament_package()
