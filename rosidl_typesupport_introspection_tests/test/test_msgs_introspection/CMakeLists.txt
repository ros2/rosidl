find_package(ament_cmake_gtest REQUIRED)
find_package(test_msgs REQUIRED)

ament_add_gtest(
  test_typesupport_introspection_libraries
  test_typesupport_introspection_libraries.cpp
)
if(TARGET test_typesupport_introspection_libraries)
  target_include_directories(
    test_typesupport_introspection_libraries PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR})
  target_link_libraries(
    test_typesupport_introspection_libraries
    ${PROJECT_NAME})
  ament_target_dependencies(
    test_typesupport_introspection_libraries test_msgs)
endif()

ament_add_gtest(
  test_empty_message_introspection
  test_empty_message_introspection.cpp
)
if(TARGET test_empty_message_introspection)
  target_include_directories(
    test_empty_message_introspection PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR})
  target_link_libraries(
    test_empty_message_introspection ${PROJECT_NAME})
  ament_target_dependencies(
    test_empty_message_introspection test_msgs)
endif()

ament_add_gtest(
  test_arrays_message_introspection
  test_arrays_message_introspection.cpp
)
if(TARGET test_arrays_message_introspection)
  target_include_directories(
    test_arrays_message_introspection PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR})
  target_link_libraries(
    test_arrays_message_introspection ${PROJECT_NAME})
  ament_target_dependencies(
    test_arrays_message_introspection test_msgs)
endif()
