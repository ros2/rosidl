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

# Convert action files from messages to services.
#
# This function declares what message and service files will be created given
# action files.
# It creates a custom target which generates messages and services when the
# .action files change.
#
# :param target: A target name to use to generate the .msg and .srv files
# :type target: string
# :param ARGN: Paths to .action files
# :type ARGN: strings
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

  # Make a list of files that will be generated
  set(_output_path "${CMAKE_CURRENT_BINARY_DIR}/rosidl_actions/${PROJECT_NAME}")
  set(_generated_action_idl_files "")
  foreach(_action_file ${_action_files})
    get_filename_component(_parent_folder "${_action_file}" DIRECTORY)
    get_filename_component(_parent_folder "${_parent_folder}" NAME)
    get_filename_component(_action_name "${_action_file}" NAME_WE)
    get_filename_component(_extension "${_action_file}" EXT)

    if(NOT _extension STREQUAL ".action")
      message(FATAL_ERROR "action files must end in .action")
    endif()

    list(APPEND _generated_action_idl_files
      "${_output_path}/${_parent_folder}/${_action_name}_Goal.srv")
    list(APPEND _generated_action_idl_files
      "${_output_path}/${_parent_folder}/${_action_name}_Goal_Request.msg")
    list(APPEND _generated_action_idl_files
      "${_output_path}/${_parent_folder}/${_action_name}_Goal_Response.msg")
    list(APPEND _generated_action_idl_files
      "${_output_path}/${_parent_folder}/${_action_name}_Result.srv")
    list(APPEND _generated_action_idl_files
      "${_output_path}/${_parent_folder}/${_action_name}_Result_Request.msg")
    list(APPEND _generated_action_idl_files
      "${_output_path}/${_parent_folder}/${_action_name}_Result_Response.msg")
    list(APPEND _generated_action_idl_files
      "${_output_path}/${_parent_folder}/${_action_name}_Feedback.msg")
  endforeach()

  # Write generator arguments
  set(generator_arguments_file "${CMAKE_CURRENT_BINARY_DIR}/rosidl_actions_convert_actions_to_msg_and_srv__arguments.json")
  rosidl_write_generator_arguments(
    "${generator_arguments_file}"
    PACKAGE_NAME "${PROJECT_NAME}"  # TODO(sloretz) why is this required?
    ROS_INTERFACE_FILES "${_action_files}"
    ROS_INTERFACE_DEPENDENCIES ""
    OUTPUT_DIR "${_output_path}"
    TEMPLATE_DIR "dontneedthisbutitisrequired"
  )

  find_package(rosidl_actions REQUIRED)

  # Cmake boilerplate to trigger generation
  add_custom_command(
    OUTPUT ${_generated_action_idl_files}
    COMMAND ${PYTHON_EXECUTABLE} ${rosidl_actions_BIN}
    --generator-arguments-file "${generator_arguments_file}"
    DEPENDS ${_action_files}
    COMMENT "Generating .msg and .srv for ROS .action interfaces"
    VERBATIM
  )

  add_custom_target(
    ${target}
    DEPENDS
    ${_generated_action_idl_files}
  )

  set(${_ARG_OUTPUT_IDL_VAR} ${_generated_action_idl_files} PARENT_SCOPE)
endfunction()
