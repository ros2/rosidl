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

# Determine which IDL files are describing action files.
#
# :param ARGN: Paths to files that may or may not be actions
# :type ARGN: strings
# :param OUTPUT_ACTION_VAR: Set to a list of files that are actions.
# :type OUTPUT_ACTION_VAR: list of strings
# :param OUTPUT_IDL_VAR: Set to a list of files that are not actions.
# :type OUTPUT_IDL_VAR: list of strings
#
# @public
#
function(rosidl_identify_action_idls)
  cmake_parse_arguments(_ARG
    ""
    "OUTPUT_ACTION_VAR;OUTPUT_IDL_VAR"
    ""
    ${ARGN})
  if(NOT _ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "rosidl_identify_action_idls() must be given IDL files")
  endif()
  if(NOT _ARG_OUTPUT_ACTION_VAR)
    message(FATAL_ERROR "rosidl_identify_action_idls() needs output variable for actions set")
  endif()
  if(NOT _ARG_OUTPUT_IDL_VAR)
    message(FATAL_ERROR "rosidl_identify_action_idls() needs output variable for non-actions set")
  endif()

  set(_action_files "")
  set(_non_action_files "")
  foreach(_idl_file ${_ARG_UNPARSED_ARGUMENTS})
    get_filename_component(_extension "${_idl_file}" EXT)
    if(_extension STREQUAL ".action")
      list(APPEND _action_files ${_idl_file})
    else()
      list(APPEND _non_action_files ${_idl_file})
    endif()
  endforeach()

  set(${_ARG_OUTPUT_ACTION_VAR} ${_action_files} PARENT_SCOPE)
  set(${_ARG_OUTPUT_IDL_VAR} ${_non_action_files} PARENT_SCOPE)
endfunction()
