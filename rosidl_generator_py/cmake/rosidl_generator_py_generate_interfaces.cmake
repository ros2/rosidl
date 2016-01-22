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

set(_output_path
  "${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_py/${PROJECT_NAME}")
set(_generated_msg_files "")
set(_generated_srv_files "")
foreach(_idl_file ${rosidl_generate_interfaces_IDL_FILES})
  get_filename_component(_parent_folder "${_idl_file}" DIRECTORY)
  get_filename_component(_parent_folder "${_parent_folder}" NAME)
  get_filename_component(_msg_name "${_idl_file}" NAME_WE)
  string_camel_case_to_lower_case_underscore("${_msg_name}" _module_name)

  if("${_parent_folder} " STREQUAL "msg ")
    list(APPEND _generated_msg_files
      "${_output_path}/${_parent_folder}/_${_module_name}.py"
    )
  elseif("${_parent_folder} " STREQUAL "srv ")
    list(APPEND _generated_srv_files
      "${_output_path}/${_parent_folder}/_${_module_name}.py"
    )
  else()
    message(FATAL_ERROR "Interface file with unknown parent folder: ${_idl_file}")
  endif()
endforeach()

if(NOT "${_generated_msg_files} " STREQUAL " ")
  list(GET _generated_msg_files 0 _msg_file)
  get_filename_component(_parent_folder "${_msg_file}" DIRECTORY)
  list(APPEND _generated_msg_files
    "${_parent_folder}/__init__.py"
  )
endif()

if(NOT "${_generated_srv_files} " STREQUAL " ")
  list(GET _generated_srv_files 0 _srv_file)
  get_filename_component(_parent_folder "${_srv_file}" DIRECTORY)
  list(APPEND _generated_srv_files
    "${_parent_folder}/__init__.py"
  )
endif()

set(_dependency_files "")
set(_dependencies "")
foreach(_pkg_name ${rosidl_generate_interfaces_DEPENDENCY_PACKAGE_NAMES})
  foreach(_idl_file ${${_pkg_name}_INTERFACE_FILES})
    set(_abs_idl_file "${${_pkg_name}_DIR}/../${_idl_file}")
    normalize_path(_abs_idl_file "${_abs_idl_file}")
    list(APPEND _dependency_files "${_abs_idl_file}")
    list(APPEND _dependencies "${_pkg_name}:${_abs_idl_file}")
  endforeach()
endforeach()

set(target_dependencies
  "${rosidl_generator_py_BIN}"
  ${rosidl_generator_py_GENERATOR_FILES}
  "${rosidl_generator_py_TEMPLATE_DIR}/_msg.py.template"
  "${rosidl_generator_py_TEMPLATE_DIR}/_srv.py.template"
  ${rosidl_generate_interfaces_IDL_FILES}
  ${_dependency_files})
foreach(dep ${target_dependencies})
  if(NOT EXISTS "${dep}")
    message(FATAL_ERROR "Target dependency '${dep}' does not exist")
  endif()
endforeach()

set(generator_arguments_file "${CMAKE_BINARY_DIR}/rosidl_generator_py__arguments.json")
rosidl_write_generator_arguments(
  "${generator_arguments_file}"
  PACKAGE_NAME "${PROJECT_NAME}"
  ROS_INTERFACE_FILES "${rosidl_generate_interfaces_IDL_FILES}"
  ROS_INTERFACE_DEPENDENCIES "${_dependencies}"
  OUTPUT_DIR "${_output_path}"
  TEMPLATE_DIR "${rosidl_generator_py_TEMPLATE_DIR}"
  TARGET_DEPENDENCIES ${target_dependencies}
)

file(MAKE_DIRECTORY "${_output_path}")
file(WRITE "${_output_path}/__init__.py" "")
add_custom_command(
  OUTPUT ${_generated_msg_files} ${_generated_srv_files}
  COMMAND ${PYTHON_EXECUTABLE} ${rosidl_generator_py_BIN}
  --generator-arguments-file "${generator_arguments_file}"
  DEPENDS ${target_dependencies}
  COMMENT "Generating Python code for ROS interfaces"
  VERBATIM
)

if(TARGET ${rosidl_generate_interfaces_TARGET}__py)
  message(WARNING "Custom target ${rosidl_generate_interfaces_TARGET}__py already exists")
else()
  add_custom_target(
    ${rosidl_generate_interfaces_TARGET}__py
    DEPENDS
    ${_generated_msg_files} ${_generated_srv_files}
  )
endif()

add_dependencies(
  ${rosidl_generate_interfaces_TARGET}
  ${rosidl_generate_interfaces_TARGET}__py
)

if(NOT rosidl_generate_interfaces_SKIP_INSTALL)
  if(NOT "${_generated_msg_files} " STREQUAL " ")
    list(GET _generated_msg_files 0 _msg_file)
    get_filename_component(_msg_package_dir "${_msg_file}" DIRECTORY)
    get_filename_component(_msg_package_dir "${_msg_package_dir}" DIRECTORY)
    ament_python_install_package("${PROJECT_NAME}" PACKAGE_DIR "${_msg_package_dir}")
  endif()
endif()
