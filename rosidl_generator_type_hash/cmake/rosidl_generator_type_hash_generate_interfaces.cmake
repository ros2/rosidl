# Copyright 2023 Open Source Robotics Foundation, Inc.
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

find_package(Python3 REQUIRED COMPONENTS Interpreter)

set(_output_path "${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_type_hash/${PROJECT_NAME}")
set(_generated_json_in "")
set(_generated_json "")
set(_generated_hash_tuples "")
set(_generated_hash_files "")

# Create lists of generated fiiles
foreach(_abs_idl_file ${rosidl_generate_interfaces_ABS_IDL_FILES})
  get_filename_component(_parent_folder "${_abs_idl_file}" DIRECTORY)
  get_filename_component(_parent_folder "${_parent_folder}" NAME)
  get_filename_component(_idl_stem "${_abs_idl_file}" NAME_WE)
  list(APPEND _generated_json_in
    "${_output_path}/${_parent_folder}/${_idl_stem}.in.json")
  list(APPEND _generated_json
    "${_output_path}/${_parent_folder}/${_idl_stem}.json")
  set(_hash_file "${_output_path}/${_parent_folder}/${_idl_stem}.sha256.json")
  list(APPEND _generated_hash_files ${_hash_file})
  list(APPEND _generated_hash_tuples "${_parent_folder}/${_idl_stem}:${_hash_file}")
endforeach()

# Find dependency packages' .in.json files
set(_dependency_files "")
set(_dependency_paths "")
foreach(_pkg_name ${rosidl_generate_interfaces_DEPENDENCY_PACKAGE_NAMES})
  set(_include_path "${${_pkg_name}_DIR}/..")
  normalize_path(_include_path "${_include_path}")
  list(APPEND _dependency_paths "${_pkg_name}:${_include_path}")
  foreach(_json_in_file ${${_pkg_name}_JSON_IN_FILES})
    set(_abs_json_in_file ${_include_path}/${_json_in_file})
    list(APPEND _dependency_files ${_abs_json_in_file})
  endforeach()
endforeach()

# Export __HASH_TUPLES variable for use by dependent generators
set(${rosidl_generate_interfaces_TARGET}__HASH_TUPLES ${_generated_hash_tuples})

# Validate that all dependencies exist
set(target_dependencies
  "${rosidl_generator_type_hash_BIN}"
  ${rosidl_generator_type_hash_GENERATOR_FILES}
  ${rosidl_generate_interfaces_ABS_IDL_FILES}
  ${_dependency_files})
foreach(dep ${target_dependencies})
  if(NOT EXISTS "${dep}")
    message(FATAL_ERROR "Target dependency '${dep}' does not exist")
  endif()
endforeach()

set(_generator_arguments_file "${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_type_hash__arguments.json")
rosidl_write_generator_arguments(
  "${_generator_arguments_file}"
  PACKAGE_NAME "${PROJECT_NAME}"
  IDL_TUPLES "${rosidl_generate_interfaces_IDL_TUPLES}"
  OUTPUT_DIR "${_output_path}"
  INCLUDE_PATHS "${_dependency_paths}"
)

# Create custom command and target to generate the hash output
set(_generated_files "${_generated_json};${_generated_json_in};${_generated_hash_files}")
add_custom_command(
  COMMAND Python3::Interpreter
  ARGS
  ${rosidl_generator_type_hash_BIN}
  --generator-arguments-file "${_generator_arguments_file}"
  OUTPUT ${_generated_files}
  DEPENDS ${target_dependencies}
  COMMENT "Generating type hashes for ROS interfaces"
  VERBATIM
)

set(_target "${rosidl_generate_interfaces_TARGET}__rosidl_generator_type_hash")
add_custom_target(${_target} DEPENDS ${_generated_files})

# Make top level generation target depend on this generated library
add_dependencies(${rosidl_generate_interfaces_TARGET} ${_target})

if(NOT rosidl_generate_interfaces_SKIP_INSTALL)
  foreach(_generated_file ${_generated_files})
    get_filename_component(_parent_folder "${_generated_file}" DIRECTORY)
    get_filename_component(_parent_folder "${_parent_folder}" NAME)
    install(
      FILES ${_generated_file}
      DESTINATION "share/${PROJECT_NAME}/${_parent_folder}")
  endforeach()
endif()