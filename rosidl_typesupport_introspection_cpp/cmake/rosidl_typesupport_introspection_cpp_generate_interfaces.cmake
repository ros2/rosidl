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
  "${CMAKE_CURRENT_BINARY_DIR}/rosidl_typesupport_introspection_cpp/${PROJECT_NAME}")
set(_generated_files "")
foreach(_idl_file ${rosidl_generate_interfaces_IDL_FILES})
  get_filename_component(_parent_folder "${_idl_file}" DIRECTORY)
  get_filename_component(_parent_folder "${_parent_folder}" NAME)
  if(_parent_folder STREQUAL "msg" OR _parent_folder STREQUAL "srv")
    get_filename_component(_msg_name "${_idl_file}" NAME_WE)
    string_camel_case_to_lower_case_underscore("${_msg_name}" _header_name)
    list(APPEND _generated_files
      "${_output_path}/${_parent_folder}/${_header_name}__type_support.cpp"
    )
  else()
    message(FATAL_ERROR "Interface file with unknown parent folder: ${_idl_file}")
  endif()
endforeach()

set(_dependency_files "")
set(_dependencies "")
foreach(_pkg_name ${rosidl_generate_interfaces_DEPENDENCY_PACKAGE_NAMES})
  foreach(_idl_file ${${_pkg_name}_INTERFACE_FILES})
  get_filename_component(_extension "${_idl_file}" EXT)
  if(_extension STREQUAL ".msg")
    set(_abs_idl_file "${${_pkg_name}_DIR}/../${_idl_file}")
    normalize_path(_abs_idl_file "${_abs_idl_file}")
    list(APPEND _dependency_files "${_abs_idl_file}")
    list(APPEND _dependencies "${_pkg_name}:${_abs_idl_file}")
  endif()
  endforeach()
endforeach()

set(target_dependencies
  "${rosidl_typesupport_introspection_cpp_BIN}"
  ${rosidl_typesupport_introspection_cpp_GENERATOR_FILES}
  "${rosidl_typesupport_introspection_cpp_TEMPLATE_DIR}/msg__type_support.cpp.em"
  "${rosidl_typesupport_introspection_cpp_TEMPLATE_DIR}/srv__type_support.cpp.em"
  ${rosidl_generate_interfaces_IDL_FILES}
  ${_dependency_files})
foreach(dep ${target_dependencies})
  if(NOT EXISTS "${dep}")
    message(FATAL_ERROR "Target dependency '${dep}' does not exist")
  endif()
endforeach()

set(generator_arguments_file "${CMAKE_BINARY_DIR}/rosidl_typesupport_introspection_cpp__arguments.json")
rosidl_write_generator_arguments(
  "${generator_arguments_file}"
  PACKAGE_NAME "${PROJECT_NAME}"
  ROS_INTERFACE_FILES "${rosidl_generate_interfaces_IDL_FILES}"
  ROS_INTERFACE_DEPENDENCIES "${_dependencies}"
  OUTPUT_DIR "${_output_path}"
  TEMPLATE_DIR "${rosidl_typesupport_introspection_cpp_TEMPLATE_DIR}"
  TARGET_DEPENDENCIES ${target_dependencies}
)

add_custom_command(
  OUTPUT ${_generated_files}
  COMMAND ${PYTHON_EXECUTABLE} ${rosidl_typesupport_introspection_cpp_BIN}
  --generator-arguments-file "${generator_arguments_file}"
  DEPENDS ${target_dependencies}
  COMMENT "Generating C++ introspection for ROS interfaces"
  VERBATIM
)

set(_target_suffix "__rosidl_typesupport_introspection_cpp")

add_library(${rosidl_generate_interfaces_TARGET}${_target_suffix} SHARED ${_generated_files})
if(rosidl_generate_interfaces_LIBRARY_NAME)
  set_target_properties(${rosidl_generate_interfaces_TARGET}${_target_suffix}
    PROPERTIES OUTPUT_NAME "${rosidl_generate_interfaces_LIBRARY_NAME}${_target_suffix}")
endif()
if(WIN32)
  target_compile_definitions(${rosidl_generate_interfaces_TARGET}${_target_suffix}
    PRIVATE "ROSIDL_BUILDING_DLL")
  target_compile_definitions(${rosidl_generate_interfaces_TARGET}${_target_suffix}
    PRIVATE "ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_BUILDING_DLL")
endif()
if(NOT WIN32)
  set_target_properties(${rosidl_generate_interfaces_TARGET}${_target_suffix}
    PROPERTIES COMPILE_FLAGS "-std=c++14 -Wall -Wextra")
endif()
target_include_directories(${rosidl_generate_interfaces_TARGET}${_target_suffix}
  PUBLIC
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)
ament_target_dependencies(${rosidl_generate_interfaces_TARGET}${_target_suffix}
  "rosidl_typesupport_interface"
  "rosidl_typesupport_introspection_cpp")
foreach(_pkg_name ${rosidl_generate_interfaces_DEPENDENCY_PACKAGE_NAMES})
  ament_target_dependencies(
    ${rosidl_generate_interfaces_TARGET}${_target_suffix}
    ${_pkg_name})
endforeach()

add_dependencies(
  ${rosidl_generate_interfaces_TARGET}
  ${rosidl_generate_interfaces_TARGET}${_target_suffix}
)
add_dependencies(
  ${rosidl_generate_interfaces_TARGET}${_target_suffix}
  ${rosidl_generate_interfaces_TARGET}__cpp
)

if(NOT rosidl_generate_interfaces_SKIP_INSTALL)
  install(
    TARGETS ${rosidl_generate_interfaces_TARGET}${_target_suffix}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
  ament_export_libraries(${rosidl_generate_interfaces_TARGET}${_target_suffix})
endif()

if(BUILD_TESTING AND rosidl_generate_interfaces_ADD_LINTER_TESTS)
  if(NOT _generated_files STREQUAL "")
    find_package(ament_cmake_cppcheck REQUIRED)
    ament_cppcheck(
      TESTNAME "cppcheck_rosidl_typesupport_introspection_cpp"
      "${_output_path}")

    find_package(ament_cmake_cpplint REQUIRED)
    get_filename_component(_cpplint_root "${_output_path}" DIRECTORY)
    ament_cpplint(
      TESTNAME "cpplint_rosidl_typesupport_introspection_cpp"
      # the generated code might contain longer lines for templated types
      MAX_LINE_LENGTH 999
      ROOT "${_cpplint_root}"
      "${_output_path}")

    find_package(ament_cmake_uncrustify REQUIRED)
    ament_uncrustify(
      TESTNAME "uncrustify_rosidl_typesupport_introspection_cpp"
      # the generated code might contain longer lines for templated types
      MAX_LINE_LENGTH 999
      "${_output_path}")
  endif()
endif()
