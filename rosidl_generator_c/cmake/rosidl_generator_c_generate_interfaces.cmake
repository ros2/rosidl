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

set(rosidl_generate_interfaces_c_IDL_FILES ${rosidl_generate_interfaces_IDL_FILES})
message(" - rosidl_generator_c_generate_interfaces.cmake")
message("   - target: ${rosidl_generate_interfaces_TARGET}")
message("   - interface files: ${rosidl_generate_interfaces_c_IDL_FILES}")
message("   - dependency package names: ${rosidl_generate_interfaces_DEPENDENCY_PACKAGE_NAMES}")

set(_output_path "${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_c/${PROJECT_NAME}")
set(_generated_files "")
foreach(_idl_file ${rosidl_generate_interfaces_c_IDL_FILES})
  # TODO(wjwwood): Enable support for things others than .msg
  # Conditionally process interface files if they end with .msg
  string(LENGTH ${_idl_file} _idl_file_len)
  math(EXPR _idl_file_ext_pos "${_idl_file_len} - 4")
  string(SUBSTRING ${_idl_file} ${_idl_file_ext_pos} -1 _idl_file_ext)
  if (${_idl_file_ext} STREQUAL ".msg")
    get_filename_component(name "${_idl_file}" NAME_WE)
    list(APPEND _generated_files
      "${_output_path}/${name}-c.h"
      "${_output_path}/${name}_Struct-c.h"
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
    if (${_idl_file_ext} STREQUAL ".msg")
      set(_abs_idl_file "${${_pkg_name}_DIR}/../${_idl_file}")
      normalize_path(_abs_idl_file "${_abs_idl_file}")
      list(APPEND _dependency_files "${_abs_idl_file}")
      list(APPEND _dependencies "${_pkg_name}:${_abs_idl_file}")
    endif()
  endforeach()
endforeach()

message("   - generated files: ${_generated_files}")
message("   - dependencies: ${_dependencies}")

add_custom_command(
  OUTPUT ${_generated_files}
  COMMAND ${PYTHON_EXECUTABLE} ${rosidl_generator_c_BIN}
  --pkg-name ${PROJECT_NAME}
  --ros-interface-files ${rosidl_generate_interfaces_c_IDL_FILES}
  --deps ${_dependencies}
  --output-dir ${_output_path}
  --template-dir ${rosidl_generator_c_TEMPLATE_DIR}
  DEPENDS
  ${rosidl_generator_c_BIN}
  ${rosidl_generator_c_GENERATOR_FILES}
  ${rosidl_generator_c_TEMPLATE_DIR}/msg-c.h.template
  ${rosidl_generator_c_TEMPLATE_DIR}/msg_Struct-c.h.template
  ${rosidl_generate_interfaces_c_IDL_FILES}
  ${_dependency_files}
  COMMENT "Generating C code for ROS interfaces"
  VERBATIM
)

add_custom_target(
  ${rosidl_generate_interfaces_TARGET}__c
  DEPENDS
  ${_generated_files}
)
add_dependencies(
  ${rosidl_generate_interfaces_TARGET}
  ${rosidl_generate_interfaces_TARGET}__c
)

install(
  FILES ${_generated_files}
  DESTINATION "include/${PROJECT_NAME}"
)

ament_export_include_directories(include)
