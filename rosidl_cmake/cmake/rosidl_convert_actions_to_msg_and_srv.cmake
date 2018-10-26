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

# Convert action files from messages to services
#
# :param target: A target name to use to generate the .msg and .srv files
# :type target: string
# :param idl_files: A variable with a list of IDL files that may be actions.
# :type target: list of strings
# :param OUTPUT_IDL_VAR: Set to a list of message and service files
# :type OUTPUT_IDL_VAR: list of strings
#
# @public
#
function(rosidl_convert_actions_to_msg_and_srv target)
  cmake_parse_arguments(_ARG
    ""
    "OUTPUT_IDL_VAR"
    ""
    ${ARGN})
  if(NOT _ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "rosidl_convert_actions_to_msg_and_srv() must be given IDL files")
  endif()
  if(NOT _ARG_OUTPUT_IDL_VAR)
    message(FATAL_ERROR "rosidl_convert_actions_to_msg_and_srv() needs output variable for interfaces")
  endif()

  set(_action_files ${_ARG_UNPARSED_ARGUMENTS})

  set(_sub_target "${target}+_convert_actions_to_msg_and_srv")

  add_custom_target(
    ${_sub_target}
    DEPENDS
    ${_action_files}
    SOURCES
    ${_action_files}
  )

  # Call ament extension to do the generation
  # A target name that generators may want to use to prefix their own target names
  set(rosidl_convert_actions_to_msg_and_srv_TARGET ${_sub_target})
  # Give extensions a list of .action files to generate interfaces from
  set(rosidl_convert_actions_to_msg_and_srv_ACTION_FILES ${_action_files})
  # If true the extension should not install anything it generates
  set(rosidl_convert_actions_to_msg_and_srv_SKIP_INSTALL ${_ARG_SKIP_INSTALL})
  # If true the extension should create tests for language specific linters
  set(rosidl_convert_actions_to_msg_and_srv_ADD_LINTER_TESTS ${_ARG_ADD_LINTER_TESTS})
  # Extension should append to this variable
  set(rosidl_convert_actions_to_msg_and_srv_OUTPUT_IDL "")
  ament_execute_extensions("rosidl_convert_actions_to_msg_and_srv")

  set(${_ARG_OUTPUT_IDL_VAR} ${rosidl_convert_actions_to_msg_and_srv_OUTPUT_IDL} PARENT_SCOPE)
endfunction()
