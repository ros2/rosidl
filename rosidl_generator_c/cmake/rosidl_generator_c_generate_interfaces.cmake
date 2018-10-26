# Copyright 2015 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

set(rosidl_generate_interfaces_c_IDL_FILES
  ${rosidl_generate_interfaces_IDL_FILES})
set(_output_path "${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_c/${PROJECT_NAME}")
set(_generated_msg_headers "")
set(_generated_msg_sources "")
set(_generated_srv_headers "")
set(_generated_srv_sources "")
set(_generated_action_headers "")
set(_generated_action_sources "")
foreach(_idl_file ${rosidl_generate_interfaces_c_IDL_FILES})
  get_filename_component(_parent_folder "${_idl_file}" DIRECTORY)
  get_filename_component(_parent_folder "${_parent_folder}" NAME)
  get_filename_component(_msg_name "${_idl_file}" NAME_WE)
  get_filename_component(_extension "${_idl_file}" EXT)
  string_camel_case_to_lower_case_underscore("${_msg_name}" _header_name)

  if(_parent_folder STREQUAL "msg")
    set(_generated_headers "_generated_msg_headers")
    set(_generated_sources "_generated_msg_sources")
  elseif(_parent_folder STREQUAL "srv")
    set(_generated_headers "_generated_srv_headers")
    set(_generated_sources "_generated_srv_sources")
  elseif(_parent_folder STREQUAL "action")
    set(_generated_headers "_generated_action_headers")
    set(_generated_sources "_generated_action_sources")
  else()
    message(FATAL_ERROR "Interface file with unknown parent folder: ${_idl_file}")
  endif()

  if(_extension STREQUAL ".msg")
    list(APPEND ${_generated_headers}
      "${_output_path}/${_parent_folder}/${_header_name}.h"
      "${_output_path}/${_parent_folder}/${_header_name}__functions.h"
      "${_output_path}/${_parent_folder}/${_header_name}__struct.h"
      "${_output_path}/${_parent_folder}/${_header_name}__type_support.h"
    )
    list(APPEND ${_generated_sources}
      "${_output_path}/${_parent_folder}/${_header_name}__functions.c"
    )
  elseif(_extension STREQUAL ".srv")
    list(APPEND ${_generated_headers}
      "${_output_path}/${_parent_folder}/${_header_name}.h"
    )
  else()
    list(REMOVE_ITEM rosidl_generate_interfaces_c_IDL_FILES ${_idl_file})
  endif()
endforeach()

set(_dependency_files "")
set(_dependencies "")
foreach(_pkg_name ${rosidl_generate_interfaces_DEPENDENCY_PACKAGE_NAMES})
  foreach(_idl_file ${${_pkg_name}_INTERFACE_FILES})
    get_filename_component(_idl_file_ext "${_idl_file}" EXT)
    if(${_idl_file_ext} STREQUAL ".msg")
      set(_abs_idl_file "${${_pkg_name}_DIR}/../${_idl_file}")
      normalize_path(_abs_idl_file "${_abs_idl_file}")
      list(APPEND _dependency_files "${_abs_idl_file}")
      list(APPEND _dependencies "${_pkg_name}:${_abs_idl_file}")
    endif()
  endforeach()
endforeach()

set(target_dependencies
  "${rosidl_generator_c_BIN}"
  ${rosidl_generator_c_GENERATOR_FILES}
  "${rosidl_generator_c_TEMPLATE_DIR}/msg.h.em"
  "${rosidl_generator_c_TEMPLATE_DIR}/msg__functions.c.em"
  "${rosidl_generator_c_TEMPLATE_DIR}/msg__functions.h.em"
  "${rosidl_generator_c_TEMPLATE_DIR}/msg__struct.h.em"
  "${rosidl_generator_c_TEMPLATE_DIR}/msg__type_support.h.em"
  "${rosidl_generator_c_TEMPLATE_DIR}/srv.h.em"
  ${rosidl_generate_interfaces_c_IDL_FILES}
  ${_dependency_files})
foreach(dep ${target_dependencies})
  if(NOT EXISTS "${dep}")
    get_property(is_generated SOURCE "${dep}" PROPERTY GENERATED)
    if(NOT ${_is_generated})
      message(FATAL_ERROR "Target dependency '${dep}' does not exist")
    endif()
  endif()
endforeach()

set(generator_arguments_file "${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_c__arguments.json")
rosidl_write_generator_arguments(
  "${generator_arguments_file}"
  PACKAGE_NAME "${PROJECT_NAME}"
  ROS_INTERFACE_FILES "${rosidl_generate_interfaces_c_IDL_FILES}"
  ROS_INTERFACE_DEPENDENCIES "${_dependencies}"
  OUTPUT_DIR "${_output_path}"
  TEMPLATE_DIR "${rosidl_generator_c_TEMPLATE_DIR}"
  TARGET_DEPENDENCIES ${target_dependencies}
)

add_custom_command(
  OUTPUT ${_generated_msg_headers} ${_generated_msg_sources} ${_generated_srv_headers} ${_generated_srv_sources} ${_generated_action_headers} ${_generated_action_sources}
  COMMAND ${PYTHON_EXECUTABLE} ${rosidl_generator_c_BIN}
  --generator-arguments-file "${generator_arguments_file}"
  DEPENDS ${target_dependencies}
  COMMENT "Generating C code for ROS interfaces"
  VERBATIM
)

# generate header to switch between export and import for a specific package
set(_visibility_control_file
  "${_output_path}/msg/rosidl_generator_c__visibility_control.h")
string(TOUPPER "${PROJECT_NAME}" PROJECT_NAME_UPPER)
configure_file(
  "${rosidl_generator_c_TEMPLATE_DIR}/rosidl_generator_c__visibility_control.h.in"
  "${_visibility_control_file}"
  @ONLY
)

list(APPEND _generated_msg_headers "${_visibility_control_file}")

set(_target_suffix "__rosidl_generator_c")

add_library(${rosidl_generate_interfaces_TARGET}${_target_suffix} ${rosidl_generator_c_LIBRARY_TYPE}
  ${_generated_msg_headers} ${_generated_msg_sources} ${_generated_srv_headers} ${_generated_srv_sources}
  ${_generated_action_headers} ${_generated_action_sources})
if(rosidl_generate_interfaces_LIBRARY_NAME)
  set_target_properties(${rosidl_generate_interfaces_TARGET}${_target_suffix}
    PROPERTIES OUTPUT_NAME "${rosidl_generate_interfaces_LIBRARY_NAME}${_target_suffix}")
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set_target_properties(${rosidl_generate_interfaces_TARGET}${_target_suffix} PROPERTIES
    C_STANDARD 11
    COMPILE_OPTIONS -Wall -Wextra -Wpedantic)
endif()
if(WIN32)
  target_compile_definitions(${rosidl_generate_interfaces_TARGET}${_target_suffix}
    PRIVATE "ROSIDL_GENERATOR_C_BUILDING_DLL_${PROJECT_NAME}")
endif()
target_include_directories(${rosidl_generate_interfaces_TARGET}${_target_suffix}
  PUBLIC
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_c
)
foreach(_pkg_name ${rosidl_generate_interfaces_DEPENDENCY_PACKAGE_NAMES})
  ament_target_dependencies(
    ${rosidl_generate_interfaces_TARGET}${_target_suffix}
    ${_pkg_name})
endforeach()
ament_target_dependencies(${rosidl_generate_interfaces_TARGET}${_target_suffix}
  "rosidl_generator_c"
  "rosidl_typesupport_interface")

add_dependencies(
  ${rosidl_generate_interfaces_TARGET}
  ${rosidl_generate_interfaces_TARGET}${_target_suffix}
)

if(NOT rosidl_generate_interfaces_SKIP_INSTALL)
  if(NOT _generated_msg_headers STREQUAL "")
    install(
      FILES ${_generated_msg_headers}
      DESTINATION "include/${PROJECT_NAME}/msg"
    )
  endif()
  if(NOT _generated_srv_headers STREQUAL "")
    install(
      FILES ${_generated_srv_headers}
      DESTINATION "include/${PROJECT_NAME}/srv"
    )
  endif()
  if(NOT _generated_action_headers STREQUAL "")
    install(
      FILES ${_generated_action_headers}
      DESTINATION "include/${PROJECT_NAME}/action"
    )
  endif()
  ament_export_libraries(${rosidl_generate_interfaces_TARGET}${_target_suffix})
  install(
    TARGETS ${rosidl_generate_interfaces_TARGET}${_target_suffix}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
  ament_export_include_directories(include)
endif()

if(BUILD_TESTING AND rosidl_generate_interfaces_ADD_LINTER_TESTS)
  if(
    NOT _generated_msg_headers STREQUAL "" OR
    NOT _generated_msg_sources STREQUAL "" OR
    NOT _generated_srv_headers STREQUAL "" OR
    NOT _generated_srv_sources STREQUAL "" OR
    NOT _generated_action_headers STREQUAL "" OR
    NOT _generated_action_sources STREQUAL ""
  )
    find_package(ament_cmake_cppcheck REQUIRED)
    ament_cppcheck(
      TESTNAME "cppcheck_rosidl_generated_c"
      "${_output_path}")

    find_package(ament_cmake_cpplint REQUIRED)
    get_filename_component(_cpplint_root "${_output_path}" DIRECTORY)
    ament_cpplint(
      TESTNAME "cpplint_rosidl_generated_c"
      # the generated code might contain longer lines for templated types
      MAX_LINE_LENGTH 999
      ROOT "${_cpplint_root}"
      "${_output_path}")

    find_package(ament_cmake_uncrustify REQUIRED)
    ament_uncrustify(
      TESTNAME "uncrustify_rosidl_generated_c"
      # the generated code might contain longer lines for templated types
      MAX_LINE_LENGTH 999
      "${_output_path}")
  endif()
endif()
