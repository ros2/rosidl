cmake_minimum_required(VERSION 3.5)

project(rosidl_generator_c C)

if(NOT WIN32)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c11 -Wall -Wextra")
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_typesupport_interface REQUIRED)

include_directories(include)
add_library(${PROJECT_NAME} SHARED
  "src/message_type_support.c"
  "src/primitives_array_functions.c"
  "src/service_type_support.c"
  "src/string_functions.c"
)
ament_target_dependencies(${PROJECT_NAME}
  "rosidl_typesupport_interface")
if(NOT WIN32)
  set_target_properties(${PROJECT_NAME} PROPERTIES
    COMPILE_FLAGS "-Wall -Wextra")
endif()
if(WIN32)
  target_compile_definitions(${PROJECT_NAME}
    PRIVATE "ROSIDL_GENERATOR_C_BUILDING_DLL")
endif()

ament_export_dependencies(rosidl_cmake)
ament_export_dependencies(rosidl_typesupport_interface)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  set(message_files
    "msg/Empty.msg"
    "msg/Nested.msg"
    "msg/Primitives.msg"
    "msg/Various.msg"
    "msg/Bool.msg"
    "msg/Byte.msg"
    "msg/Char.msg"
    "msg/Float32.msg"
    "msg/Float64.msg"
    "msg/Int8.msg"
    "msg/Int16.msg"
    "msg/Int32.msg"
    "msg/Int64.msg"
    "msg/Uint8.msg"
    "msg/Uint16.msg"
    "msg/Uint32.msg"
    "msg/Uint64.msg"
    "msg/PrimitiveValues.msg"
    "msg/Strings.msg"
    "msg/PrimitivesUnboundedArrays.msg"
    "msg/PrimitivesBoundedArrays.msg"
    "msg/PrimitivesStaticArrays.msg"
    "msg/Telegram1.msg"
    "msg/Telegram2.msg"
    "msg/Wire.msg"
    "msg/Constants.msg"
  )

  include(cmake/register_c.cmake)
  set(rosidl_generator_c_DIR "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
  set(rosidl_generator_c_LIBRARIES "${PROJECT_NAME}")

  rosidl_generator_c_extras(
    "${CMAKE_CURRENT_SOURCE_DIR}/bin/rosidl_generator_c"
    "${CMAKE_CURRENT_SOURCE_DIR}/rosidl_generator_c/__init__.py"
    "${CMAKE_CURRENT_SOURCE_DIR}/resource"
  )

  rosidl_generate_interfaces(${PROJECT_NAME}_interfaces ${message_files}
    SKIP_INSTALL
  )

  add_executable(test_compilation_c test/test_compilation.c test/separate_compilation.c)
  add_executable(test_interfaces_c test/test_interfaces.c)
  add_dependencies(test_interfaces_c ${PROJECT_NAME})
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
  # include the built files directly, instead of their install location
  include_directories("${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}")
  target_link_libraries(test_compilation_c ${PROJECT_NAME} ${PROJECT_NAME}_interfaces__${PROJECT_NAME})
  target_link_libraries(test_interfaces_c ${PROJECT_NAME} ${PROJECT_NAME}_interfaces__${PROJECT_NAME})
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
