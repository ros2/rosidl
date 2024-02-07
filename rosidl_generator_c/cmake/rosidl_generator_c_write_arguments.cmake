# Copyright 2023 Open Source Robotics Foundation, Inc.
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

find_package(rcutils REQUIRED)
find_package(rosidl_cmake REQUIRED)
find_package(rosidl_runtime_c REQUIRED)
find_package(rosidl_typesupport_interface REQUIRED)

set(rosidl_generate_interfaces_c_IDL_TUPLES
  ${rosidl_generate_interfaces_IDL_TUPLES})
set(_output_path "${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_c/${PROJECT_NAME}")

set(_dependency_files "")
set(_dependencies "")
foreach(_pkg_name ${rosidl_generate_interfaces_DEPENDENCY_PACKAGE_NAMES})
  foreach(_idl_file ${${_pkg_name}_IDL_FILES})
    rosidl_find_package_idl(_abs_idl_file "${_pkg_name}" "${_idl_file}")
    list(APPEND _dependency_files "${_abs_idl_file}")
    list(APPEND _dependencies "${_pkg_name}:${_abs_idl_file}")
  endforeach()
endforeach()

set(target_dependencies
  ${rosidl_generator_c_GENERATOR_FILES}
  "${rosidl_generator_c_TEMPLATE_DIR}/action__type_support.h.em"
  "${rosidl_generator_c_TEMPLATE_DIR}/action__type_support.c.em"
  "${rosidl_generator_c_TEMPLATE_DIR}/empty__description.c.em"
  "${rosidl_generator_c_TEMPLATE_DIR}/full__description.c.em"
  "${rosidl_generator_c_TEMPLATE_DIR}/idl.h.em"
  "${rosidl_generator_c_TEMPLATE_DIR}/idl__description.c.em"
  "${rosidl_generator_c_TEMPLATE_DIR}/idl__functions.c.em"
  "${rosidl_generator_c_TEMPLATE_DIR}/idl__functions.h.em"
  "${rosidl_generator_c_TEMPLATE_DIR}/idl__struct.h.em"
  "${rosidl_generator_c_TEMPLATE_DIR}/idl__type_support.c.em"
  "${rosidl_generator_c_TEMPLATE_DIR}/idl__type_support.h.em"
  "${rosidl_generator_c_TEMPLATE_DIR}/msg__functions.c.em"
  "${rosidl_generator_c_TEMPLATE_DIR}/msg__functions.h.em"
  "${rosidl_generator_c_TEMPLATE_DIR}/msg__struct.h.em"
  "${rosidl_generator_c_TEMPLATE_DIR}/msg__type_support.h.em"
  "${rosidl_generator_c_TEMPLATE_DIR}/srv__type_support.c.em"
  "${rosidl_generator_c_TEMPLATE_DIR}/srv__type_support.h.em"
  ${rosidl_generate_interfaces_ABS_IDL_FILES}
  ${_dependency_files})
foreach(dep ${target_dependencies})
  if(NOT EXISTS "${dep}")
    message(FATAL_ERROR "Target dependency '${dep}' does not exist")
  endif()
endforeach()

set(additional_context_file "${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_c__additional_context.json")
set(disable_description_codegen_arg "false")
if(ROSIDL_GENERATOR_C_DISABLE_TYPE_DESCRIPTION_CODEGEN)
  set(disable_description_codegen_arg "true")  
endif()
rosidl_write_additional_context(
  "${additional_context_file}"
  DISABLE_DESCRIPTION_CODEGEN "${disable_description_codegen_arg}"
)

set(generator_arguments_file "${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_c__arguments.json")
rosidl_write_generator_arguments(
  "${generator_arguments_file}"
  PACKAGE_NAME "${PROJECT_NAME}"
  IDL_TUPLES "${rosidl_generate_interfaces_c_IDL_TUPLES}"
  ROS_INTERFACE_DEPENDENCIES "${_dependencies}"
  OUTPUT_DIR "${_output_path}"
  TEMPLATE_DIR "${rosidl_generator_c_TEMPLATE_DIR}"
  TARGET_DEPENDENCIES ${target_dependencies}
  TYPE_DESCRIPTION_TUPLES "${${rosidl_generate_interfaces_TARGET}__DESCRIPTION_TUPLES}"
  ROS_INTERFACE_FILES "${_target_sources}"
  GENERATOR_FILES "${rosidl_generator_c_GENERATOR_FILES}"
  ADDITIONAL_CONTEXT_FILE ${additional_context_file}
)

list(APPEND rosidl_generator_arguments_files ${generator_arguments_file})
