# Copyright 2014-2018 Open Source Robotics Foundation, Inc.
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
  "${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp/${PROJECT_NAME}")
set(_generated_headers "")
foreach(_abs_idl_file ${rosidl_generate_interfaces_ABS_IDL_FILES})
  get_filename_component(_parent_folder "${_abs_idl_file}" DIRECTORY)
  get_filename_component(_parent_folder "${_parent_folder}" NAME)
  get_filename_component(_idl_name "${_abs_idl_file}" NAME_WE)
  string_camel_case_to_lower_case_underscore("${_idl_name}" _header_name)

  list(APPEND _generated_headers
    "${_output_path}/${_parent_folder}/${_header_name}.hpp"
    "${_output_path}/${_parent_folder}/detail/${_header_name}__builder.hpp"
    "${_output_path}/${_parent_folder}/detail/${_header_name}__struct.hpp"
    "${_output_path}/${_parent_folder}/detail/${_header_name}__traits.hpp"
  )
endforeach()

set(_dependency_files "")
set(_dependencies "")
foreach(_pkg_name ${rosidl_generate_interfaces_DEPENDENCY_PACKAGE_NAMES})
  foreach(_idl_file ${${_pkg_name}_IDL_FILES})
    set(_abs_idl_file "${${_pkg_name}_DIR}/../${_idl_file}")
    normalize_path(_abs_idl_file "${_abs_idl_file}")
    list(APPEND _dependency_files "${_abs_idl_file}")
    list(APPEND _dependencies "${_pkg_name}:${_abs_idl_file}")
  endforeach()
endforeach()

set(target_dependencies
  "${rosidl_generator_cpp_BIN}"
  ${rosidl_generator_cpp_GENERATOR_FILES}
  "${rosidl_generator_cpp_TEMPLATE_DIR}/action__builder.hpp.em"
  "${rosidl_generator_cpp_TEMPLATE_DIR}/action__struct.hpp.em"
  "${rosidl_generator_cpp_TEMPLATE_DIR}/action__traits.hpp.em"
  "${rosidl_generator_cpp_TEMPLATE_DIR}/idl.hpp.em"
  "${rosidl_generator_cpp_TEMPLATE_DIR}/idl__builder.hpp.em"
  "${rosidl_generator_cpp_TEMPLATE_DIR}/idl__struct.hpp.em"
  "${rosidl_generator_cpp_TEMPLATE_DIR}/idl__traits.hpp.em"
  "${rosidl_generator_cpp_TEMPLATE_DIR}/msg__builder.hpp.em"
  "${rosidl_generator_cpp_TEMPLATE_DIR}/msg__struct.hpp.em"
  "${rosidl_generator_cpp_TEMPLATE_DIR}/msg__traits.hpp.em"
  "${rosidl_generator_cpp_TEMPLATE_DIR}/srv__builder.hpp.em"
  "${rosidl_generator_cpp_TEMPLATE_DIR}/srv__struct.hpp.em"
  "${rosidl_generator_cpp_TEMPLATE_DIR}/srv__traits.hpp.em"
  ${rosidl_generate_interfaces_ABS_IDL_FILES}
  ${_dependency_files})
foreach(dep ${target_dependencies})
  if(NOT EXISTS "${dep}")
    message(FATAL_ERROR "Target dependency '${dep}' does not exist")
  endif()
endforeach()

set(generator_arguments_file "${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp__arguments.json")
rosidl_write_generator_arguments(
  "${generator_arguments_file}"
  PACKAGE_NAME "${PROJECT_NAME}"
  IDL_TUPLES "${rosidl_generate_interfaces_IDL_TUPLES}"
  ROS_INTERFACE_DEPENDENCIES "${_dependencies}"
  OUTPUT_DIR "${_output_path}"
  TEMPLATE_DIR "${rosidl_generator_cpp_TEMPLATE_DIR}"
  TARGET_DEPENDENCIES ${target_dependencies}
)

add_custom_command(
  OUTPUT ${_generated_headers}
  COMMAND ${PYTHON_EXECUTABLE} ${rosidl_generator_cpp_BIN}
  --generator-arguments-file "${generator_arguments_file}"
  DEPENDS ${target_dependencies}
  COMMENT "Generating C++ code for ROS interfaces"
  VERBATIM
)

if(TARGET ${rosidl_generate_interfaces_TARGET}__cpp)
  message(WARNING "Custom target ${rosidl_generate_interfaces_TARGET}__cpp already exists")
else()
  add_custom_target(
    ${rosidl_generate_interfaces_TARGET}__cpp
    DEPENDS
    ${_generated_headers}
  )
endif()

add_dependencies(
  ${rosidl_generate_interfaces_TARGET}
  ${rosidl_generate_interfaces_TARGET}__cpp
)

set(_target_suffix "__rosidl_generator_cpp")
add_library(${rosidl_generate_interfaces_TARGET}${_target_suffix} INTERFACE)
target_include_directories(${rosidl_generate_interfaces_TARGET}${_target_suffix}
  INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>"
  "$<INSTALL_INTERFACE:include>"
)
foreach(_pkg_name ${rosidl_generate_interfaces_DEPENDENCY_PACKAGE_NAMES})
  target_link_libraries(
    ${rosidl_generate_interfaces_TARGET}${_target_suffix} INTERFACE
    ${${_pkg_name}_TARGETS})
endforeach()
target_link_libraries(
  ${rosidl_generate_interfaces_TARGET}${_target_suffix} INTERFACE
  ${rosidl_runtime_cpp_TARGETS})

if(NOT rosidl_generate_interfaces_SKIP_INSTALL)
  if(NOT _generated_headers STREQUAL "")
    install(
      DIRECTORY ${_output_path}/
      DESTINATION "include/${PROJECT_NAME}"
      PATTERN "*.hpp"
    )
  endif()
  ament_export_include_directories(include)

  install(
    TARGETS ${rosidl_generate_interfaces_TARGET}${_target_suffix}
    EXPORT ${rosidl_generate_interfaces_TARGET}${_target_suffix}
  )
  ament_export_targets(${rosidl_generate_interfaces_TARGET}${_target_suffix})
endif()

if(BUILD_TESTING AND rosidl_generate_interfaces_ADD_LINTER_TESTS)
  if(NOT _generated_headers STREQUAL "")
    find_package(ament_cmake_cppcheck REQUIRED)
    ament_cppcheck(
      TESTNAME "cppcheck_rosidl_generated_cpp"
      "${_output_path}")

    find_package(ament_cmake_cpplint REQUIRED)
    get_filename_component(_cpplint_root "${_output_path}" DIRECTORY)
    ament_cpplint(
      TESTNAME "cpplint_rosidl_generated_cpp"
      # the generated code might contain longer lines for templated types
      MAX_LINE_LENGTH 999
      ROOT "${_cpplint_root}"
      "${_output_path}")

    find_package(ament_cmake_uncrustify REQUIRED)
    ament_uncrustify(
      TESTNAME "uncrustify_rosidl_generated_cpp"
      # the generated code might contain longer lines for templated types
      # a value of zero tells uncrustify to ignore line length
      MAX_LINE_LENGTH 0
      "${_output_path}")
  endif()
endif()
