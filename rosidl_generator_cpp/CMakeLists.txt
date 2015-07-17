cmake_minimum_required(VERSION 2.8.3)

project(rosidl_generator_cpp)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_export_dependencies(rosidl_cmake)
ament_export_include_directories(include)

ament_python_install_package(${PROJECT_NAME})

if(AMENT_ENABLE_TESTING)

  find_package(ament_cmake_nose REQUIRED)
  find_package(rosidl_cmake REQUIRED)
  find_package(rosidl_generator_c REQUIRED)

  set(message_files
    "msg/DynamicArrayNested.msg"
    "msg/DynamicArrayPrimitives.msg"
    "msg/Empty.msg"
    "msg/Nested.msg"
    "msg/NestedBounded.msg"
    "msg/Primitives.msg"
    "msg/PrimitivesBounded.msg"
    "msg/StaticArrayNested.msg"
    "msg/StaticArrayNestedBounded.msg"
    "msg/StaticArrayPrimitives.msg"
    "msg/StaticArrayBounded.msg"
  )

  ament_register_extension(
    "rosidl_generate_interfaces"
    "rosidl_generator_cpp"
    "rosidl_generator_cpp_generate_interfaces.cmake")

  set(rosidl_generator_cpp_DIR "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
  set(rosidl_generator_cpp_BIN
    "${rosidl_generator_cpp_DIR}/../bin/rosidl_generator_cpp")
  normalize_path(rosidl_generator_cpp_BIN "${rosidl_generator_cpp_BIN}")

  set(rosidl_generator_cpp_GENERATOR_FILES
    "${rosidl_generator_cpp_DIR}/../rosidl_generator_cpp/__init__.py")
  normalize_path(rosidl_generator_cpp_GENERATOR_FILES
    "${rosidl_generator_cpp_GENERATOR_FILES}")

  set(rosidl_generator_cpp_TEMPLATE_DIR
    "${rosidl_generator_cpp_DIR}/../resource")
  normalize_path(rosidl_generator_cpp_TEMPLATE_DIR
    "${rosidl_generator_cpp_TEMPLATE_DIR}")

  rosidl_generate_interfaces(${PROJECT_NAME} ${message_files}
    SKIP_INSTALL
  )

endif()
ament_package(
  CONFIG_EXTRAS "rosidl_generator_cpp-extras.cmake.in"
)

install(
  PROGRAMS bin/rosidl_generator_cpp
  DESTINATION lib/rosidl_generator_cpp
)
install(
  DIRECTORY cmake resource
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY include/
  DESTINATION include
)

if(AMENT_ENABLE_TESTING)
  if(NOT WIN32)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")
  endif()

  add_executable(test_messages_cpp "test/test_messages.cpp")
  add_dependencies(test_messages_cpp ${PROJECT_NAME})
  include_directories($ENV{AMENT_PREFIX_PATH}/include)
endif()
