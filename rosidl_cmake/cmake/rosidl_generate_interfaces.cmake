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

# include CMake functions
include(CMakeParseArguments)

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
# :param SKIP_INSTALL: if set skip installing the interface files
# :type SKIP_INSTALL: option
#
# @public
#
macro(rosidl_generate_interfaces target)
  cmake_parse_arguments(_ARG "SKIP_INSTALL" "" "DEPENDENCIES" ${ARGN})
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

  file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/srv")

  foreach(_idl_file ${_idl_files})
    get_filename_component(_extension "${_idl_file}" EXT)
    # generate request and response messages for services
    if("${_extension}" STREQUAL ".srv")
      get_filename_component(_name "${_idl_file}" NAME_WE)
      set(_request_file "${CMAKE_CURRENT_BINARY_DIR}/srv/${_name}_Request.msg")
      set(_response_file "${CMAKE_CURRENT_BINARY_DIR}/srv/${_name}_Response.msg")
      file(READ "${_idl_file}" _service_content)
      string(REGEX REPLACE "^((.*\n)|)---(\n.*)?$" "\\1" _request_content "${_service_content}")
      string(REGEX REPLACE "^((.*\n)|)---(\n(.*)|())$" "\\4" _response_content "${_service_content}")
      file(WRITE "${_request_file}" "${_request_content}")
      file(WRITE "${_response_file}" "${_response_content}")
      list(APPEND _idl_files "${_request_file}" "${_response_file}")
    endif()
  endforeach()

  # stamp all interface files
  foreach(_idl_file ${_idl_files})
    stamp("${_idl_file}")
  endforeach()

  add_custom_target(
    ${target} ALL
    DEPENDS
    ${_idl_files}
    ${_dep_files}
    SOURCES
    ${_idl_files}
  )

  # generators must be executed in topological order
  # which is ensured by every generator finding its dependencies first
  # and then registering itself as an extension
  set(rosidl_generate_interfaces_TARGET ${target})
  set(rosidl_generate_interfaces_IDL_FILES ${_idl_files})
  set(rosidl_generate_interfaces_DEPENDENCY_PACKAGE_NAMES ${_ARG_DEPENDENCIES})
  ament_execute_extensions("rosidl_generate_interfaces")

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
