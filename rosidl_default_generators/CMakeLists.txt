cmake_minimum_required(VERSION 2.8.3)

project(rosidl_default_generators)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif()

find_package(ament_cmake REQUIRED)

ament_export_dependencies(rosidl_cmake)

ament_export_dependencies(rosidl_generator_c)
ament_export_dependencies(rosidl_generator_cpp)

ament_export_dependencies(rosidl_typesupport_introspection_cpp)

ament_export_dependencies(rosidl_typesupport_connext_cpp)
ament_export_dependencies(rosidl_typesupport_opensplice_cpp)


if(AMENT_ENABLE_TESTING)

  find_package(ament_cmake_nose REQUIRED)
  find_package(rosidl_cmake REQUIRED)
  find_package(rosidl_generator_c REQUIRED)
  find_package(rosidl_generator_cpp REQUIRED)

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

  rosidl_generate_interfaces(${PROJECT_NAME} ${message_files}
    SKIP_INSTALL
  )

endif()

ament_package()


if(AMENT_ENABLE_TESTING)
  #add_executable(test_messages_cpp "test/test_messages.cpp")
  #add_dependencies(test_messages_cpp ${PROJECT_NAME})

  #include_directories(include)
endif()
