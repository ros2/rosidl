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
function(rosidl_adapt_interfaces messages_var services_var)
  cmake_parse_arguments(ARG
    "" "TARGET" "MESSAGE_FILES;SERVICE_FILES;INTERFACE_FILES"
    ${ARGN})
  if(ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "rosidl_adapt_interfaces() called with unused "
      "arguments: ${ARG_UNPARSED_ARGUMENTS}")
  endif()
  if(NOT ARG_MESSAGE_FILES AND NOT ARG_SERVICE_FILES AND NOT ARG_INTERFACE_FILES)
    message(FATAL_ERROR "rosidl_adapt_interfaces() called without any "
      "interface files")
  endif()

  find_package(PythonInterp REQUIRED)
  if(NOT PYTHON_EXECUTABLE)
    message(FATAL_ERROR "Variable 'PYTHON_EXECUTABLE' must not be empty")
  endif()

  set(_messages_output "${CMAKE_CURRENT_BINARY_DIR}/rosidl_adapter/${ARG_TARGET}.messages")
  set(_services_output "${CMAKE_CURRENT_BINARY_DIR}/rosidl_adapter/${ARG_TARGET}.services")
  execute_process(
    COMMAND
    "${PYTHON_EXECUTABLE}" -m rosidl_adapter
    --package-dir "${CMAKE_SOURCE_DIR}"
    --package-name ${PROJECT_NAME}
    --message-files ${ARG_MESSAGE_FILES}
    --service-files ${ARG_SERVICE_FILES}
    --interface-files ${ARG_INTERFACE_FILES}
    --output-dir "${CMAKE_CURRENT_BINARY_DIR}/rosidl_adapter"
    --output-messages-file "${_messages_output}"
    --output-services-file "${_services_output}"
    OUTPUT_QUIET
  )

  file(STRINGS "${_messages_output}" message_files)
  file(STRINGS "${_services_output}" service_files)
  set(${messages_var} ${message_files} PARENT_SCOPE)
  set(${services_var} ${service_files} PARENT_SCOPE)
endfunction()
