cmake_minimum_required(VERSION 3.5)

project(rosidl_actions)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_export_dependencies(ament_cmake)
ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package(
  CONFIG_EXTRAS
    "rosidl_actions-extras.cmake.in"
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # TODO(sloretz) add test here
  # ament_add_pytest_test(pytest test)
endif()

install(
  PROGRAMS bin/rosidl_actions
  DESTINATION lib/rosidl_actions
)

install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)
