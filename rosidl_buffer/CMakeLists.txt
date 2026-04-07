cmake_minimum_required(VERSION 3.20)
project(rosidl_buffer)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/c_helpers.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_compile_definitions(${PROJECT_NAME} PRIVATE "ROSIDL_BUFFER_BUILDING_DLL")

# Install headers
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Install library target
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include/${PROJECT_NAME}
)

# Export modern CMake targets
ament_export_targets(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_buffer test/test_buffer.cpp)
  if(TARGET test_buffer)
    target_link_libraries(test_buffer ${PROJECT_NAME})
  endif()

  ament_add_gtest(test_c_helpers test/test_c_helpers.cpp)
  if(TARGET test_c_helpers)
    target_link_libraries(test_c_helpers ${PROJECT_NAME})
  endif()
endif()

ament_package()
