cmake_minimum_required(VERSION 3.20)
project(rosidl_buffer_backend_registry)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_buffer REQUIRED)
find_package(rosidl_buffer_backend REQUIRED)
find_package(rmw REQUIRED)
find_package(rosidl_runtime_cpp REQUIRED)
find_package(pluginlib REQUIRED)

# Library
add_library(${PROJECT_NAME} SHARED
  src/buffer_backend_registry.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_compile_definitions(${PROJECT_NAME}
  PRIVATE "ROSIDL_BUFFER_BACKEND_REGISTRY_BUILDING_DLL"
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  rosidl_buffer_backend::rosidl_buffer_backend
  rmw::rmw
  rosidl_runtime_cpp::rosidl_runtime_cpp
  pluginlib::pluginlib
)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Install library
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include/${PROJECT_NAME}
)

# Export dependencies
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rosidl_buffer_backend rmw rosidl_runtime_cpp pluginlib)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  # Test backend registry and dummy backend (direct instantiation, not via pluginlib)
  ament_add_gtest(test_buffer_backend_registry
    test/test_buffer_backend_registry.cpp
  )
  if(TARGET test_buffer_backend_registry)
    target_include_directories(test_buffer_backend_registry PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/test
    )
    target_link_libraries(test_buffer_backend_registry
      ${PROJECT_NAME}
      rosidl_buffer::rosidl_buffer
      rosidl_buffer_backend::rosidl_buffer_backend
      rmw::rmw
      rosidl_runtime_cpp::rosidl_runtime_cpp
      pluginlib::pluginlib
    )
  endif()
endif()

ament_package()

