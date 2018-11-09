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

# Generate Interfaces for Actions
#
# This executes the extension point ``rosidl_generate_action_interfaces``.
# An extension of this type should expect a list of `.action` files and
# generate new files in response.
# Extensions should create a target that generates the files with the input
# `.action` file as a dependency.
#
# :param target: the _name of the generation target,
#   specific generators might use the _name as a prefix for their own
#   generation step
# :type target: string
# :param ARGN: a list of include directories where each value might
#   be either an absolute path or path relative to the
#   CMAKE_INSTALL_PREFIX.
# :type ARGN: list of strings
# :param TARGET_DEPENDENCIES: cmake targets or files the generated target
#   should depend on
# :type TARGET_DEPENDENCIES: list of strings
# :param LIBRARY_NAME: the base name of the library, specific generators might
#   append their own suffix
# :type LIBRARY_NAME: string
# :param SKIP_INSTALL: if set skip installing the interface files
# :type SKIP_INSTALL: option
# :param ADD_LINTER_TESTS: if set lint the interface files using
#   the ``ament_lint`` package
# :type ADD_LINTER_TESTS: option
#
# @public
#
function(rosidl_generate_action_interfaces target)
  cmake_parse_arguments(_ARG
    "ADD_LINTER_TESTS;SKIP_INSTALL"
    "LIBRARY_NAME" "TARGET_DEPENDENCIES"
    ${ARGN})
  if(NOT _ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "rosidl_generate_action_interfaces() called without any idl "
      "files")
  endif()

  set(_idl_files ${_ARG_UNPARSED_ARGUMENTS})

  # Create a custom target
  set(_sub_target "${target}+generate_action_interfaces")
  add_custom_target(
    ${_sub_target} ALL
    DEPENDS
    ${_idl_files}
    ${_ARG_TARGET_DEPENDENCIES}
    SOURCES
    ${_idl_files}
  )

  # A target name that generators may want to use to prefix their own target names
  set(rosidl_generate_action_interfaces_TARGET ${target})
  # Give extensions a list of .action files to generate interfaces from
  set(rosidl_generate_action_interfaces_IDL_FILES ${_idl_files})
  # TODO(sloretz) Where is LIBRARY_NAME used?
  set(rosidl_generate_action_interfaces_LIBRARY_NAME ${_ARG_LIBRARY_NAME})
  # If true the extension should not install anything it generates
  set(rosidl_generate_action_interfaces_SKIP_INSTALL ${_ARG_SKIP_INSTALL})
  # If true the extension should create tests for language specific linters
  set(rosidl_generate_action_interfaces_ADD_LINTER_TESTS ${_ARG_ADD_LINTER_TESTS})
  ament_execute_extensions("rosidl_generate_action_interfaces")

  add_dependencies(${target} ${_sub_target})
endfunction()
