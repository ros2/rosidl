cmake_minimum_required(VERSION 3.5)

project(rosidl_generator_c C)

# Default to C11
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_python REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rosidl_typesupport_interface REQUIRED)

include_directories(include)
add_library(${PROJECT_NAME}
  "src/message_bounds.c"
  "src/message_type_support.c"
  "src/primitives_sequence_functions.c"
  "src/service_type_support.c"
  "src/string_functions.c"
  "src/u16string_functions.c"
)
ament_target_dependencies(${PROJECT_NAME}
  "rosidl_typesupport_interface")
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set_target_properties(${PROJECT_NAME} PROPERTIES
    COMPILE_OPTIONS -Wall -Wextra -Wpedantic)
endif()
if(WIN32)
  target_compile_definitions(${PROJECT_NAME}
    PRIVATE "ROSIDL_GENERATOR_C_BUILDING_DLL")
endif()

ament_export_dependencies(rosidl_cmake)
ament_export_dependencies(rosidl_typesupport_interface)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_index_register_resource("rosidl_generator_packages")

ament_python_install_package(${PROJECT_NAME})

if(BUILD_SHARED_LIBS)
  set(${PROJECT_NAME}_LIBRARY_TYPE "SHARED")
else()
  set(${PROJECT_NAME}_LIBRARY_TYPE "STATIC")
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(test_interface_files REQUIRED)
  ament_lint_auto_find_test_dependencies()

  include(cmake/register_c.cmake)
  # Trick ament_target_dependencies() into thinking this package has been found
  set(rosidl_generator_c_FOUND "1")
  set(rosidl_generator_c_DIR "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
  set(rosidl_generator_c_LIBRARIES "${PROJECT_NAME}")

  rosidl_generator_c_extras(
    "${CMAKE_CURRENT_SOURCE_DIR}/bin/rosidl_generator_c"
    "${CMAKE_CURRENT_SOURCE_DIR}/rosidl_generator_c/__init__.py"
    "${CMAKE_CURRENT_SOURCE_DIR}/resource"
  )

  rosidl_generate_interfaces(${PROJECT_NAME}_interfaces
    ${test_interface_files_MSG_FILES}
    ADD_LINTER_TESTS
    SKIP_INSTALL
  )

  add_executable(test_compilation_c test/test_compilation.c test/separate_compilation.c)
  add_executable(test_interfaces_c test/test_interfaces.c)
  add_executable(test_invalid_initialization_c test/test_invalid_initialization.c)
  add_dependencies(test_interfaces_c ${PROJECT_NAME})
  add_dependencies(test_invalid_initialization_c ${PROJECT_NAME})
  ament_add_test(
    test_compilation_c
    COMMAND "$<TARGET_FILE:test_compilation_c>"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  )
  ament_add_test(
    test_interfaces_c
    COMMAND "$<TARGET_FILE:test_interfaces_c>"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  )
  ament_add_test(
    test_invalid_initialization_c
    COMMAND "$<TARGET_FILE:test_invalid_initialization_c>"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  )

  # include the built files directly, instead of their install location
  include_directories("${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}")
  target_link_libraries(test_compilation_c ${PROJECT_NAME} ${PROJECT_NAME}_interfaces__${PROJECT_NAME})
  target_link_libraries(test_interfaces_c ${PROJECT_NAME} ${PROJECT_NAME}_interfaces__${PROJECT_NAME})
  target_link_libraries(test_invalid_initialization_c ${PROJECT_NAME} ${PROJECT_NAME}_interfaces__${PROJECT_NAME})
endif()

ament_package(
  CONFIG_EXTRAS "rosidl_generator_c-extras.cmake.in"
)

install(
  PROGRAMS bin/rosidl_generator_c
  DESTINATION lib/rosidl_generator_c
)
install(
  DIRECTORY cmake resource
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
