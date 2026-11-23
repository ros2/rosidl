cmake_minimum_required(VERSION 3.20)
project(rosidl_buffer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros_core REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/c_helpers.cpp
)

# Use the ROS default C++ standard (C++20)
target_link_libraries(${PROJECT_NAME} PRIVATE ament_cmake_ros_core::ament_ros_cxx_standard)

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
    target_link_libraries(test_buffer ${PROJECT_NAME} ament_cmake_ros_core::ament_ros_cxx_standard)
  endif()

  if(NOT WIN32)
    # Loaded with dlopen(RTLD_LOCAL) by test_buffer_dso. Hidden visibility keeps its
    # CpuBufferImpl RTTI separate from the test executable's.
    add_library(test_buffer_dso_plugin MODULE test/buffer_dso_plugin.cpp)
    target_link_libraries(test_buffer_dso_plugin PRIVATE
      ${PROJECT_NAME} ament_cmake_ros_core::ament_ros_cxx_standard)
    set_target_properties(test_buffer_dso_plugin PROPERTIES
      CXX_VISIBILITY_PRESET hidden
      VISIBILITY_INLINES_HIDDEN ON)

    ament_add_gtest(test_buffer_dso test/test_buffer_dso.cpp)
    if(TARGET test_buffer_dso)
      target_link_libraries(test_buffer_dso
        ${PROJECT_NAME} ament_cmake_ros_core::ament_ros_cxx_standard ${CMAKE_DL_LIBS})
      target_compile_definitions(test_buffer_dso PRIVATE
        "BUFFER_DSO_PLUGIN_PATH=\"$<TARGET_FILE:test_buffer_dso_plugin>\"")
      add_dependencies(test_buffer_dso test_buffer_dso_plugin)
    endif()
  endif()

  ament_add_gtest(test_c_helpers test/test_c_helpers.cpp)
  if(TARGET test_c_helpers)
    target_link_libraries(test_c_helpers ${PROJECT_NAME} ament_cmake_ros_core::ament_ros_cxx_standard)
  endif()
endif()

ament_package()
