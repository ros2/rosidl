# Copyright 2014-2015 Open Source Robotics Foundation, Inc.
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
# Generate code for ROS IDL files using all available generators.
#
# Execute the extension point ``rosidl_generate_interfaces``.
#
# :param target: the _name of the generation target,
#   specific generators might use the _name as a prefix for their own
#   generation step
# :type target: string
# :param ARGN: a list of include directories where each value might
#   be either an absolute path or path relative to the
#   CMAKE_INSTALL_PREFIX.
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
macro(rosidl_generate_interfaces target)
  cmake_parse_arguments(_ARG
    "ADD_LINTER_TESTS;SKIP_INSTALL;SKIP_GROUP_MEMBERSHIP_CHECK"
    "LIBRARY_NAME" "DEPENDENCIES"
    ${ARGN})
  if(NOT _ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "rosidl_generate_interfaces() called without any idl "
      "files")
  endif()

  if(_${PROJECT_NAME}_AMENT_PACKAGE)
    message(FATAL_ERROR
      "rosidl_generate_interfaces() must be called before ament_package()")
  endif()

  _rosidl_cmake_register_package_hook()
  ament_export_dependencies(${_ARG_DEPENDENCIES})

  # check all interface files
  set(_idl_files "")
  foreach(_idl_file ${_ARG_UNPARSED_ARGUMENTS})
    if(NOT IS_ABSOLUTE "${_idl_file}")
      set(_idl_file "${CMAKE_CURRENT_SOURCE_DIR}/${_idl_file}")
    endif()
    if(NOT EXISTS "${_idl_file}")
      message(FATAL_ERROR "rosidl_generate_interfaces() the passed idl file "
        "'${_idl_file}' does not exist")
    endif()
    list(APPEND _idl_files "${_idl_file}")
  endforeach()

  # collect all interface files from dependencies
  set(_dep_files)
  foreach(_dep ${_ARG_DEPENDENCIES})
    if(NOT ${_dep}_FOUND)
      message(FATAL_ERROR "rosidl_generate_interfaces() the passed dependency "
        "'${_dep}' has not been found before using find_package()")
    endif()
    foreach(_idl_file ${${_dep}_INTERFACE_FILES})
      set(_abs_idl_file "${${_dep}_DIR}/../${_idl_file}")
      normalize_path(_abs_idl_file "${_abs_idl_file}")
      list(APPEND _dep_files "${_abs_idl_file}")
    endforeach()
  endforeach()

  # Split .srv into two .msg files
  foreach(_idl_file ${_idl_files})
    get_filename_component(_extension "${_idl_file}" EXT)
    # generate request and response messages for services
    if(_extension STREQUAL ".srv")
      get_filename_component(_name "${_idl_file}" NAME_WE)
      get_filename_component(_parent_folder "${_idl_file}" DIRECTORY)
      get_filename_component(_parent_folder "${_parent_folder}" NAME)
      file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/${_parent_folder}")
      set(_request_file "${CMAKE_CURRENT_BINARY_DIR}/${_parent_folder}/${_name}_Request.msg")
      set(_response_file "${CMAKE_CURRENT_BINARY_DIR}/${_parent_folder}/${_name}_Response.msg")
      file(READ "${_idl_file}" _service_content)
      string(REGEX REPLACE "^((.*\r?\n)|)---(\r?\n.*)?$" "\\1" _request_content "${_service_content}")
      string(REGEX REPLACE "^((.*\r?\n)|)---(\r?\n(.*)|())$" "\\3" _response_content "${_service_content}")
      # only re-write the request/response messages if the content has changed
      # to avoid the last modified timestamp to be updated
      if(NOT EXISTS "${_request_file}")
        file(WRITE "${_request_file}" "${_request_content}")
      else()
        file(READ "${_request_file}" _existing_request_content)
        if(NOT "${_request_content}" STREQUAL "${_existing_request_content}")
          file(WRITE "${_request_file}" "${_request_content}")
        endif()
      endif()
      if(NOT EXISTS "${_response_file}")
        file(WRITE "${_response_file}" "${_response_content}")
      else()
        file(READ "${_response_file}" _existing_response_content)
        if(NOT "${_response_content}" STREQUAL "${_existing_response_content}")
          file(WRITE "${_response_file}" "${_response_content}")
        endif()
      endif()
      list(APPEND _idl_files "${_request_file}" "${_response_file}")
    endif()
  endforeach()

  # stamp all interface files
  foreach(_idl_file ${_idl_files})
    stamp("${_idl_file}")
  endforeach()

  # Separate action files from other interface files
  rosidl_identify_action_idls(${_idl_files}
    OUTPUT_ACTION_VAR _action_files
    OUTPUT_IDL_VAR _idl_files)

  # Convert action files into messages and services
  if(_action_files)
    set(_convert_actions_target "${target}+_convert_actions_to_msg_and_srv")
    rosidl_convert_actions_to_msg_and_srv(${_convert_actions_target} ${_action_files}
      OUTPUT_IDL_VAR _action_msg_and_srv_files)
  endif()

  add_custom_target(
    ${target} ALL
    DEPENDS
    ${_idl_files}
    ${_dep_files}
    ${_convert_actions_target}
    SOURCES
    ${_idl_files}
  )

  # Tell CMake in this directory scope that these files are generated
  foreach(_idl_file ${_action_msg_and_srv_files})
    list(APPEND _idl_files "${_idl_file}")
    set_property(SOURCE ${_idl_file} PROPERTY GENERATED 1)
  endforeach()

  if(NOT _ARG_SKIP_INSTALL)
    if(NOT _ARG_SKIP_GROUP_MEMBERSHIP_CHECK)
      set(_group_name "rosidl_interface_packages")
      if(NOT _AMENT_PACKAGE_NAME)
        ament_package_xml()
      endif()
      if(NOT _group_name IN_LIST ${_AMENT_PACKAGE_NAME}_MEMBER_OF_GROUPS)
        message(FATAL_ERROR
          "Packages installing interfaces must include \
          '<member_of_group>${_group_name}</member_of_group>' \
          in their package.xml"
        )
      endif()
    endif()
    # register interfaces with the ament index
    set(_idl_files_lines)
    foreach(_idl_file ${_ARG_UNPARSED_ARGUMENTS})
      get_filename_component(_interface_ns "${_idl_file}" DIRECTORY)
      get_filename_component(_interface_ns "${_interface_ns}" NAME)
      get_filename_component(_interface_name "${_idl_file}" NAME)
      list(APPEND _idl_files_lines "${_interface_ns}/${_interface_name}")
    endforeach()
    list(SORT _idl_files_lines)
    string(REPLACE ";" "\n" _idl_files_lines "${_idl_files_lines}")
    ament_index_register_resource("rosidl_interfaces" CONTENT "${_idl_files_lines}")
  endif()

  # collect package names of recursive dependencies which contain interface files
  set(_recursive_dependencies)
  foreach(_dep ${_ARG_DEPENDENCIES})
    if(DEFINED ${_dep}_INTERFACE_FILES)
      list_append_unique(_recursive_dependencies "${_dep}")
    endif()
    foreach(_dep2 ${${_dep}_RECURSIVE_DEPENDENCIES})
      if(DEFINED ${_dep2}_INTERFACE_FILES)
        list_append_unique(_recursive_dependencies "${_dep2}")
      endif()
    endforeach()
  endforeach()

  # generators must be executed in topological order
  # which is ensured by every generator finding its dependencies first
  # and then registering itself as an extension
  set(rosidl_generate_interfaces_TARGET ${target})
  set(rosidl_generate_interfaces_IDL_FILES ${_idl_files})
  set(rosidl_generate_interfaces_DEPENDENCY_PACKAGE_NAMES ${_recursive_dependencies})
  set(rosidl_generate_interfaces_LIBRARY_NAME ${_ARG_LIBRARY_NAME})
  set(rosidl_generate_interfaces_SKIP_INSTALL ${_ARG_SKIP_INSTALL})
  set(rosidl_generate_interfaces_ADD_LINTER_TESTS ${_ARG_ADD_LINTER_TESTS})
  ament_execute_extensions("rosidl_generate_interfaces")

  if(_action_files)
    # Invoke generation for `.action` files
    set(_skip_install "")
    if(_ARG_SKIP_INSTALL)
      set(_skip_install "SKIP_INSTALL")
    endif()
    set(_add_linter_tests "")
    if(_ARG_ADD_LINTER_TESTS)
      set(_add_linter_tests "ADD_LINTER_TESTS")
    endif()
    set(_library_name "")
    if(_ARG_LIBRARY_NAME)
      set(_library_name "LIBRARY ${_ARG_LIBRARY_NAME}")
    endif()
    set(_pkg_depends "")
    if(_recursive_dependencies)
      set(_pkg_depends "DEPENDENCY_PACKAGE_NAMES ${_recursive_dependencies}")
    endif()
    rosidl_generate_action_interfaces(${target}
      ${_skip_install}
      ${_add_linter_tests}
      ${_library_name}
      ${_action_files}
    )
  endif()

  if(NOT _ARG_SKIP_INSTALL)
    # install interface files to subfolders based on their extension
    foreach(_idl_file ${_idl_files})
      get_filename_component(_parent_folder "${_idl_file}" DIRECTORY)
      get_filename_component(_parent_folder "${_parent_folder}" NAME)
      install(
        FILES ${_idl_file}
        DESTINATION "share/${PROJECT_NAME}/${_parent_folder}"
      )
      get_filename_component(_name "${_idl_file}" NAME)
      list(APPEND _rosidl_cmake_INTERFACE_FILES "${_parent_folder}/${_name}")
    endforeach()
  endif()
endmacro()
