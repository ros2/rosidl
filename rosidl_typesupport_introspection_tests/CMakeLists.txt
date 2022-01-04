cmake_minimum_required(VERSION 3.8)
project(rosidl_typesupport_introspection_tests)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_typesupport_introspection_c REQUIRED)
find_package(rosidl_typesupport_introspection_cpp REQUIRED)
find_package(rosidl_typesupport_interface REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(test_msgs REQUIRED)
  ament_add_gtest(
    test_typesupport_c
    test/test_typesupport_c.cpp
  )
  if(TARGET test_typesupport_c)
    target_compile_features(test_typesupport_c PRIVATE cxx_std_17)
    ament_target_dependencies(test_typesupport_c
      "test_msgs"
    )
  endif()
endif()

ament_package()
