cmake_minimum_required(VERSION 3.5)

project(rosidl_generator_cpp)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_export_dependencies(rosidl_cmake)
ament_export_dependencies(rosidl_generator_c)
ament_export_include_directories(include)

ament_index_register_resource("rosidl_generator_packages")
ament_index_register_resource("rosidl_runtime_packages")

ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(rosidl_generator_c REQUIRED)
  ament_lint_auto_find_test_dependencies()

  set(message_files
    "msg/BoundedArrayBounded.msg"
    "msg/BoundedArrayStatic.msg"
    "msg/BoundedArrayUnbounded.msg"

    "msg/Empty.msg"

    "msg/FieldsWithSameTypeSomeDefaults.msg"

    "msg/PrimitiveStaticArrays.msg"

    "msg/PrimitivesBounded.msg"
    "msg/PrimitivesConstants.msg"
    "msg/PrimitivesDefault.msg"
    "msg/PrimitivesStatic.msg"
    "msg/PrimitivesUnbounded.msg"

    "msg/StaticArrayBounded.msg"
    "msg/StaticArrayStatic.msg"
    "msg/StaticArrayUnbounded.msg"

    "msg/String.msg"
    "msg/StringBounded.msg"
    "msg/StringArrayStatic.msg"
    "msg/StringArrays.msg"

    "msg/UnboundedArrayBounded.msg"
    "msg/UnboundedArrayStatic.msg"
    "msg/UnboundedArrayUnbounded.msg"

    "msg/Various.msg"

    "msg/WString.msg"
  )

  set(srv_files
    "srv/ComplexTypeZeroFill.srv"
  )

  include(cmake/register_cpp.cmake)
  set(rosidl_generator_cpp_DIR "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

  rosidl_generator_cpp_extras(
    "${CMAKE_CURRENT_SOURCE_DIR}/bin/rosidl_generator_cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/rosidl_generator_cpp/__init__.py"
    "${CMAKE_CURRENT_SOURCE_DIR}/resource"
  )

  rosidl_generate_interfaces(${PROJECT_NAME}
    ${message_files}
    ${srv_files}
    ADD_LINTER_TESTS
    SKIP_INSTALL
  )

  # Default to C++14
  if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
  endif()

  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
  endif()

  include_directories(include
    ${rosidl_generator_c_INCLUDE_DIRS})

  ament_add_gtest(test_bounded_vector test/test_bounded_vector.cpp)
  ament_add_gtest(test_msg_initialization test/test_msg_initialization.cpp)
  if(TARGET test_msg_initialization)
    add_dependencies(test_msg_initialization ${PROJECT_NAME})
  endif()
  ament_add_gtest(test_srv_initialization test/test_srv_initialization.cpp)
  if(TARGET test_srv_initialization)
    add_dependencies(test_srv_initialization ${PROJECT_NAME})
  endif()
  ament_add_gtest(test_interfaces_cpp test/test_interfaces.cpp)
  if(TARGET test_interfaces_cpp)
    add_dependencies(test_interfaces_cpp ${PROJECT_NAME})
  endif()
  ament_add_gtest(test_msg_datatype test/test_msg_datatype.cpp)
  if(TARGET test_msg_datatype)
    add_dependencies(test_msg_datatype ${PROJECT_NAME})
  endif()
  # include the built files directly, instead of their install location
  include_directories("${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}")
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
