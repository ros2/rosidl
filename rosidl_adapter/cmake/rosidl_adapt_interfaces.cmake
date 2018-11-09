# Copyright 2018 Open Source Robotics Foundation, Inc.
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

#
# Adapt interface input files to a coherent set of `.idl` files.
#
# The input file might already be an `.idl` file or any other format for which
# a plugin exist which converts the incoming file into an `.idl` file.
# E.g. `.msg` and `.srv` files are being supported for backward compatibility.
#
# Each output `.idl` file contains either a single message or a single service.
#
# :param target: the _name of the generation target,
#   specific generators might use the _name as a prefix for their own
#   generation step
# :type target: string
# :param ARGN: a list of interface files where each value might be either an
#   absolute path or path relative to the CMAKE_CURRENT_SOURCE_DIR.
# :type ARGN: list of strings
# :param DEPENDENCIES: the packages from which message types are
#   being used
# :type DEPENDENCIES: list of strings
# :param LIBRARY_NAME: the base name of the library, specific generators might
#   append their own suffix
# :type LIBRARY_NAME: string
# :param SKIP_INSTALL: if set skip installing the interface files
# :type SKIP_INSTALL: option
# :param SKIP_GROUP_MEMBERSHIP_CHECK: if set, skip enforcing the appartenance
#   to the rosidl_interface_packages group
# :type SKIP_GROUP_MEMBERSHIP_CHECK: option
# :param ADD_LINTER_TESTS: if set lint the interface files using
#   the ``ament_lint`` package
# :type ADD_LINTER_TESTS: option
#
# @public
#
function(rosidl_adapt_interfaces idl_var)
  cmake_parse_arguments(ARG "" "TARGET" ""
    ${ARGN})
  if(NOT ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "rosidl_adapt_interfaces() called without any "
      "interface files")
  endif()

  if(NOT rosidl_adapter_FOUND)
    message(FATAL_ERROR "rosidl_adapt_interfaces() called without "
      "find_package(rosidl_adapter) successfuly being called")
  endif()

  find_package(PythonInterp REQUIRED)
  if(NOT PYTHON_EXECUTABLE)
    message(FATAL_ERROR "Variable 'PYTHON_EXECUTABLE' must not be empty")
  endif()

  set(_idl_output "${CMAKE_CURRENT_BINARY_DIR}/rosidl_adapter/${ARG_TARGET}.idls")
  # TODO needs to use a JSON file for arguments otherwise the command might become too long
  set(cmd
    "${PYTHON_EXECUTABLE}" -m rosidl_adapter
    --package-name ${PROJECT_NAME}
    --interface-files ${ARG_UNPARSED_ARGUMENTS}
    --output-dir "${CMAKE_CURRENT_BINARY_DIR}/rosidl_adapter"
    --output-file "${_idl_output}")
  execute_process(
    COMMAND ${cmd}
    OUTPUT_QUIET
    ERROR_VARIABLE error
    RESULT_VARIABLE result
  )
  if(NOT result EQUAL 0)
    string(REPLACE ";" " " cmd_str "${cmd}")
    message(FATAL_ERROR
      "execute_process(${cmd_str}) returned error code ${result}:\n${error}")
  endif()

  file(STRINGS "${_idl_output}" idl_files)

  # split each absolute file into a tuple
  set(idl_tuples "")
  set(basepath "${CMAKE_CURRENT_BINARY_DIR}/rosidl_adapter/${PROJECT_NAME}")
  string(LENGTH "${basepath}" length)
  foreach(idl_file ${idl_files})
    string(SUBSTRING "${idl_file}" 0 ${length} prefix)
    if(NOT "${prefix}" STREQUAL "${basepath}")
      message(FATAL_ERROR "boom")
    endif()
    math(EXPR index "${length} + 1")
    string(SUBSTRING "${idl_file}" ${index} -1 rel_idl_file)
    list(APPEND idl_tuples "${basepath}:${rel_idl_file}")
  endforeach()

  set(${idl_var} ${idl_tuples} PARENT_SCOPE)
endfunction()
