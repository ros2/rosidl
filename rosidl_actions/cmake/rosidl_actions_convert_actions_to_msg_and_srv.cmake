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

# Implements extension `rosidl_generate_action_interfaces`
# This splits action files into .msg and .srv files

message(STATUS "rosidl_actions rosidl_actions_convert_actions_to_msg_and_srv.cmake")

# Make a list of files that will be generated
set(_output_path "${CMAKE_CURRENT_BINARY_DIR}/rosidl_actions/${PROJECT_NAME}")
set(_generated_action_idl_files "")
foreach(_action_file ${rosidl_convert_actions_to_msg_and_srv_ACTION_FILES})
  get_filename_component(_parent_folder "${_action_file}" DIRECTORY)
  get_filename_component(_parent_folder "${_parent_folder}" NAME)
  get_filename_component(_action_name "${_action_file}" NAME_WE)
  get_filename_component(_extension "${_action_file}" EXT)

  if(NOT _extension STREQUAL ".action")
    message(FATAL_ERROR "action files must end in .action")
  endif()

  list(APPEND _generated_action_idl_files
    "${_output_path}/${_parent_folder}/${_action_name}_Goal.srv"
    "${_output_path}/${_parent_folder}/${_action_name}_Result.srv"
    "${_output_path}/${_parent_folder}/${_action_name}_Feedback.msg"
  )
  message(STATUS "For ${action_file} will generate: ${_generated_action_idl_files}")
endforeach()

# Write generator arguments
set(generator_arguments_file "${CMAKE_CURRENT_BINARY_DIR}/rosidl_actions_convert_actions_to_msg_and_srv__arguments.json")
rosidl_write_generator_arguments(
  "${generator_arguments_file}"
  PACKAGE_NAME "${PROJECT_NAME}"  # TODO(sloretz) why is this required?
  ROS_INTERFACE_FILES "${rosidl_convert_actions_to_msg_and_srv_ACTION_FILES}"
  ROS_INTERFACE_DEPENDENCIES ""
  OUTPUT_DIR "${_output_path}"
  TEMPLATE_DIR "dontneedthisbutitisrequired"
)

# Cmake boilerplate to trigger generation
add_custom_command(
  OUTPUT ${_generated_action_idl_files}
  COMMAND ${PYTHON_EXECUTABLE} ${rosidl_actions_BIN}
  --generator-arguments-file "${generator_arguments_file}"
  DEPENDS ${rosidl_convert_actions_to_msg_and_srv_ACTION_FILES}
  COMMENT "Generating .msg and .srv for ROS .action interfaces"
  VERBATIM
)

set(_sub_target "${rosidl_convert_actions_to_msg_and_srv_TARGET}+_rosidl_actions")
add_custom_target(
  ${_sub_target} ALL
  DEPENDS
  ${_generated_action_idl_files}
)

# Make the high level target depend on the sub target
add_dependencies(
  ${rosidl_convert_actions_to_msg_and_srv_TARGET}
  ${_sub_target}
)

# Tell the extension caller what files were generated
foreach(_file ${_generated_action_idl_files})
  list(APPEND rosidl_convert_actions_to_msg_and_srv_OUTPUT_IDL ${_file})
endforeach()
