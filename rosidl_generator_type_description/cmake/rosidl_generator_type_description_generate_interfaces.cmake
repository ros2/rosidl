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

# By default, without the settings below, find_package(Python3) will attempt
# to find the newest python version it can, and additionally will find the
# most specific version.  For instance, on a system that has
# /usr/bin/python3.10, /usr/bin/python3.11, and /usr/bin/python3, it will find
# /usr/bin/python3.11, even if /usr/bin/python3 points to /usr/bin/python3.10.
# The behavior we want is to prefer the "system" installed version unless the
# user specifically tells us othewise through the Python3_EXECUTABLE hint.
# Setting CMP0094 to NEW means that the search will stop after the first
# python version is found.  Setting Python3_FIND_UNVERSIONED_NAMES means that
# the search will prefer /usr/bin/python3 over /usr/bin/python3.11.  And that
# latter functionality is only available in CMake 3.20 or later, so we need
# at least that version.
cmake_minimum_required(VERSION 3.20)
cmake_policy(SET CMP0094 NEW)
set(Python3_FIND_UNVERSIONED_NAMES FIRST)

find_package(Python3 REQUIRED COMPONENTS Interpreter)

set(_output_path "${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_type_description/${PROJECT_NAME}")
set(_generated_files "")
set(_generated_tuples "")

# Create list of generated files
foreach(_abs_idl_file ${rosidl_generate_interfaces_ABS_IDL_FILES})
  get_filename_component(_parent_folder "${_abs_idl_file}" DIRECTORY)
  get_filename_component(_parent_folder "${_parent_folder}" NAME)
  get_filename_component(_idl_name "${_abs_idl_file}" NAME)
  get_filename_component(_idl_stem "${_idl_name}" NAME_WE)
  set(_json_file "${_output_path}/${_parent_folder}/${_idl_stem}.json")
  list(APPEND _generated_files "${_json_file}")
  list(APPEND _generated_tuples "${_parent_folder}/${_idl_name}:${_json_file}")
endforeach()

# Find dependency packages' generated files
set(_dependency_files "")
set(_dependency_paths "")
foreach(_pkg_name ${rosidl_generate_interfaces_DEPENDENCY_PACKAGE_NAMES})
  set(_include_path "${${_pkg_name}_DIR}/..")
  normalize_path(_include_path "${_include_path}")
  list(APPEND _dependency_paths "${_pkg_name}:${_include_path}")
endforeach()

# Export __DESCRIPTION_TUPLES variable for use by dependent generators
set(${rosidl_generate_interfaces_TARGET}__DESCRIPTION_TUPLES ${_generated_tuples})

# Validate that all dependencies exist
set(target_dependencies
  "${rosidl_generator_type_description_BIN}"
  ${rosidl_generator_type_description_GENERATOR_FILES}
  ${rosidl_generate_interfaces_ABS_IDL_FILES}
  ${_dependency_files})
foreach(dep ${target_dependencies})
  if(NOT EXISTS "${dep}")
    message(FATAL_ERROR "Target dependency '${dep}' does not exist")
  endif()
endforeach()

set(_generator_arguments_file "${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_type_description__arguments.json")
rosidl_write_generator_arguments(
  "${_generator_arguments_file}"
  PACKAGE_NAME "${PROJECT_NAME}"
  IDL_TUPLES "${rosidl_generate_interfaces_IDL_TUPLES}"
  OUTPUT_DIR "${_output_path}"
  INCLUDE_PATHS "${_dependency_paths}"
)

# Create custom command and target to generate the hash output
add_custom_command(
  COMMAND Python3::Interpreter
  ARGS
  ${rosidl_generator_type_description_BIN}
  --generator-arguments-file "${_generator_arguments_file}"
  OUTPUT ${_generated_files}
  DEPENDS ${target_dependencies}
  COMMENT "Generating type hashes for ROS interfaces"
  VERBATIM
)

set(_target "${rosidl_generate_interfaces_TARGET}__rosidl_generator_type_description")
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
