cmake_minimum_required(VERSION 3.12)

project(rosidl_generator_type_description)

find_package(ament_cmake_python REQUIRED)
find_package(ament_cmake_ros REQUIRED)

ament_index_register_resource("rosidl_generator_packages")
ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_lint_auto_find_test_dependencies()
  ament_add_pytest_test(pytest_type_hash_generator test)
endif()

ament_package(
  CONFIG_EXTRAS "${PROJECT_NAME}-extras.cmake.in"
)

install(
  PROGRAMS bin/${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY cmake resource
  DESTINATION share/${PROJECT_NAME}
)
