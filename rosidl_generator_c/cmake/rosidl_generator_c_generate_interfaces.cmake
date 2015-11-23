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
foreach(_idl_file ${rosidl_generate_interfaces_c_IDL_FILES})
  # TODO(wjwwood): Enable support for things others than .msg
  get_filename_component(_parent_folder "${_idl_file}" DIRECTORY)
  get_filename_component(_parent_folder "${_parent_folder}" NAME)
  get_filename_component(_msg_name "${_idl_file}" NAME_WE)
  string_camel_case_to_lower_case_underscore("${_msg_name}" _header_name)
  if("${_parent_folder} " STREQUAL "msg ")
    list(APPEND _generated_msg_headers
      "${_output_path}/${_parent_folder}/${_header_name}.h"
      "${_output_path}/${_parent_folder}/${_header_name}__functions.h"
      "${_output_path}/${_parent_folder}/${_header_name}__struct.h"
    )
    list(APPEND _generated_msg_sources
      "${_output_path}/${_parent_folder}/${_header_name}__functions.c"
    )
  else()
    list(REMOVE_ITEM rosidl_generate_interfaces_c_IDL_FILES ${_idl_file})
  endif()
endforeach()

set(_dependency_files "")
set(_dependencies "")
foreach(_pkg_name ${rosidl_generate_interfaces_DEPENDENCY_PACKAGE_NAMES})
  foreach(_idl_file ${${_pkg_name}_INTERFACE_FILES})
    # TODO(wjwwood): Enable support for things others than .msg
    # Conditionally process interface files if they end with .msg
    string(LENGTH ${_idl_file} _idl_file_len)
    math(EXPR _idl_file_ext_pos "${_idl_file_len} - 4")
    string(SUBSTRING ${_idl_file} ${_idl_file_ext_pos} -1 _idl_file_ext)
    if(${_idl_file_ext} STREQUAL ".msg")
      set(_abs_idl_file "${${_pkg_name}_DIR}/../${_idl_file}")
      normalize_path(_abs_idl_file "${_abs_idl_file}")
      list(APPEND _dependency_files "${_abs_idl_file}")
      list(APPEND _dependencies "${_pkg_name}:${_abs_idl_file}")
    endif()
  endforeach()
endforeach()

if(NOT "${_generated_msg_headers}${_generated_msg_sources} " STREQUAL " ")
  set(target_dependencies
    "${rosidl_generator_c_BIN}"
    ${rosidl_generator_c_GENERATOR_FILES}
    "${rosidl_generator_c_TEMPLATE_DIR}/msg.h.template"
    "${rosidl_generator_c_TEMPLATE_DIR}/msg__functions.c.template"
    "${rosidl_generator_c_TEMPLATE_DIR}/msg__functions.h.template"
    "${rosidl_generator_c_TEMPLATE_DIR}/msg__struct.h.template"
    ${rosidl_generate_interfaces_c_IDL_FILES}
    ${_dependency_files})
  foreach(dep ${target_dependencies})
    if(NOT EXISTS "${dep}")
      message(FATAL_ERROR "Target dependency '${dep}' does not exist")
    endif()
  endforeach()

  set(generator_arguments_file "${CMAKE_BINARY_DIR}/rosidl_generator_c__arguments.json")
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
    OUTPUT ${_generated_msg_headers} ${_generated_msg_sources}
    COMMAND ${PYTHON_EXECUTABLE} ${rosidl_generator_c_BIN}
    --generator-arguments-file "${generator_arguments_file}"
    DEPENDS ${target_dependencies}
    COMMENT "Generating C code for ROS interfaces"
    VERBATIM
  )

  # generate header to switch between export and import for a specific package
  set(_visibility_control_file
    "${_output_path}/msg/rosidl_generator_c__visibility_control.h")
  configure_file(
    "${rosidl_generator_c_TEMPLATE_DIR}/rosidl_generator_c__visibility_control.h.in"
    "${_visibility_control_file}"
    @ONLY
  )
  list(APPEND _generated_msg_headers "${_visibility_control_file}")

  set(_target_suffix "__rosidl_generator_c")

  add_library(${rosidl_generate_interfaces_TARGET}${_target_suffix} SHARED
    ${_generated_msg_headers} ${_generated_msg_sources})
  if(NOT WIN32)
    set_target_properties(${rosidl_generate_interfaces_TARGET}${_target_suffix} PROPERTIES
      COMPILE_FLAGS "-std=c11 -Wall -Wextra")
  endif()
  if(WIN32)
    target_compile_definitions(${rosidl_generate_interfaces_TARGET}${_target_suffix}
      PRIVATE "ROSIDL_BUILDING_DLL")
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
    "rosidl_generator_c")

  add_dependencies(
    ${rosidl_generate_interfaces_TARGET}
    ${rosidl_generate_interfaces_TARGET}${_target_suffix}
  )

  if(NOT rosidl_generate_interfaces_SKIP_INSTALL)
    install(
      FILES ${_generated_msg_headers}
      DESTINATION "include/${PROJECT_NAME}/msg"
    )
    ament_export_libraries(${rosidl_generate_interfaces_TARGET}${_target_suffix})
    install(
      TARGETS ${rosidl_generate_interfaces_TARGET}${_target_suffix}
      ARCHIVE DESTINATION lib
      LIBRARY DESTINATION lib
      RUNTIME DESTINATION bin
    )
  endif()

  ament_export_include_directories(include)
endif()
