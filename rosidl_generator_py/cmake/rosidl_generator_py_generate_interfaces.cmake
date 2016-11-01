# Copyright 2014-2016 Open Source Robotics Foundation, Inc.
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

find_package(rosidl_generator_c REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)
find_package(rmw REQUIRED)

find_package(PythonInterp 3.5 REQUIRED)

find_package(python_cmake_module REQUIRED)
find_package(PythonExtra MODULE)

# Get a list of typesupport implementations from valid rmw implementations.
rosidl_generator_py_get_typesupports(_typesupport_impls)

if(_typesupport_impls STREQUAL "")
  message(WARNING "No valid typesupport for Python generator. Python messages will not be generated.")
  return()
endif()

set(_output_path
  "${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_py/${PROJECT_NAME}")
set(_generated_msg_py_files "")
set(_generated_msg_c_files "")
set(_generated_msg_c_common_files "")
set(_generated_srv_py_files "")
set(_generated_srv_c_files "")

foreach(_typesupport_impl ${_typesupport_impls})
  set(_generated_msg_c_ts_${_typesupport_impl}_files "")
endforeach()

foreach(_idl_file ${rosidl_generate_interfaces_IDL_FILES})
  get_filename_component(_parent_folder "${_idl_file}" DIRECTORY)
  get_filename_component(_parent_folder "${_parent_folder}" NAME)
  get_filename_component(_msg_name1 "${_idl_file}" NAME_WE)
  string_camel_case_to_lower_case_underscore("${_msg_name1}" _module_name)

  if(_parent_folder STREQUAL "msg")
    list(APPEND _generated_msg_py_files
      "${_output_path}/${_parent_folder}/_${_module_name}.py"
    )
    list(APPEND _generated_msg_c_files
      "${_output_path}/${_parent_folder}/_${_module_name}_s.c"
    )
    list(APPEND _generated_msg_c_common_files
      "${_output_path}/${_parent_folder}/_${_module_name}_s.c"
    )
    foreach(_typesupport_impl ${_typesupport_impls})
      list_append_unique(_generated_msg_c_files "${_output_path}/${_parent_folder}/_${PROJECT_NAME}_s.ep.${_typesupport_impl}.c")
      list_append_unique(_generated_msg_c_ts_${_typesupport_impl}_files "${_output_path}/${_parent_folder}/_${PROJECT_NAME}_s.ep.${_typesupport_impl}.c")
    endforeach()
  elseif(_parent_folder STREQUAL "srv")
    if("_${_module_name}_s.c" MATCHES "(.*)__response(.*)" OR "_${_module_name}_s.c" MATCHES "(.*)__request(.*)")
      list(APPEND _generated_srv_c_files
        "${_output_path}/${_parent_folder}/_${_module_name}_s.c"
      )
    endif()
    list(APPEND _generated_srv_py_files
      "${_output_path}/${_parent_folder}/_${_module_name}.py"
    )
  else()
    message(FATAL_ERROR "Interface file with unknown parent folder: ${_idl_file}")
  endif()
endforeach()

file(MAKE_DIRECTORY "${_output_path}")
file(WRITE "${_output_path}/__init__.py" "")

if(NOT _generated_msg_py_files STREQUAL "")
  list(GET _generated_msg_py_files 0 _msg_file)
  get_filename_component(_parent_folder "${_msg_file}" DIRECTORY)
  list(APPEND _generated_msg_py_files
    "${_parent_folder}/__init__.py"
  )
endif()

if(NOT _generated_srv_py_files STREQUAL "")
  list(GET _generated_srv_py_files 0 _srv_file)
  get_filename_component(_parent_folder "${_srv_file}" DIRECTORY)
  list(APPEND _generated_srv_py_files
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
  "${rosidl_generator_py_TEMPLATE_DIR}/_msg_support.c.em"
  "${rosidl_generator_py_TEMPLATE_DIR}/_msg_support.entry_point.c.em"
  "${rosidl_generator_py_TEMPLATE_DIR}/_msg.py.em"
  "${rosidl_generator_py_TEMPLATE_DIR}/_srv.py.em"
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


if(NOT _generated_msg_py_files STREQUAL "")
  list(GET _generated_msg_py_files 0 _msg_file)
  get_filename_component(_msg_package_dir1 "${_msg_file}" DIRECTORY)
  get_filename_component(_msg_package_dir2 "${_msg_package_dir1}" NAME)
endif()

if(NOT _generated_srv_py_files STREQUAL "")
  list(GET _generated_srv_py_files 0 _srv_file)
  get_filename_component(_srv_package_dir1 "${_srv_file}" DIRECTORY)
  get_filename_component(_srv_package_dir2 "${_srv_package_dir1}" NAME)
endif()

if(NOT rosidl_generate_interfaces_SKIP_INSTALL)
  ament_python_install_module("${_output_path}/__init__.py"
    DESTINATION_SUFFIX "${PROJECT_NAME}"
  )

  ament_python_install_module("${_output_path}/__init__.py"
    DESTINATION_SUFFIX "${PROJECT_NAME}/${_msg_package_dir2}"
  )

  # TODO(esteve): replace this with ament_python_install_module and allow a list
  # of modules to be passed instead of iterating over _generated_msg_py_files
  # See https://github.com/ros2/rosidl/issues/89
  if(NOT _msg_package_dir2 STREQUAL "")
    install(FILES ${_generated_msg_py_files}
      DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}/${_msg_package_dir2}"
    )
  endif()
  if(NOT _srv_package_dir2 STREQUAL "")
    install(FILES ${_generated_srv_py_files}
      DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}/${_srv_package_dir2}"
    )
  endif()
endif()

set(_target_suffix "__py")

# move custom command into a subdirectory to avoid multiple invocations on Windows
set(_subdir "${CMAKE_CURRENT_BINARY_DIR}/${rosidl_generate_interfaces_TARGET}${_target_suffix}")
file(MAKE_DIRECTORY "${_subdir}")
file(READ "${rosidl_generator_py_DIR}/custom_command.cmake" _custom_command)
file(WRITE "${_subdir}/CMakeLists.txt" "${_custom_command}")
add_subdirectory("${_subdir}" ${rosidl_generate_interfaces_TARGET}${_target_suffix})
set_property(
  SOURCE
  ${_generated_msg_py_files} ${_generated_msg_c_files} ${_generated_srv_py_files} ${_generated_srv_c_files}
  PROPERTY GENERATED 1)

macro(set_properties _build_type)
  set_target_properties(${_target_name} PROPERTIES
    COMPILE_FLAGS "${_extension_compile_flags}"
    PREFIX ""
    LIBRARY_OUTPUT_DIRECTORY${_build_type} ${_output_path}
    RUNTIME_OUTPUT_DIRECTORY${_build_type} ${_output_path}
    OUTPUT_NAME "${PROJECT_NAME}_s__${_typesupport_impl}${PythonExtra_EXTENSION_SUFFIX}"
    SUFFIX "${PythonExtra_EXTENSION_EXTENSION}")
endmacro()

foreach(_typesupport_impl ${_typesupport_impls})
  find_package(${_typesupport_impl} REQUIRED)

  set(_pyext_suffix "__pyext")
  set(_target_name "${PROJECT_NAME}__${_typesupport_impl}${_pyext_suffix}")

  add_library(${_target_name} SHARED
    ${_generated_msg_c_ts_${_typesupport_impl}_files}
    ${_generated_msg_c_common_files}
    ${_generated_srv_c_files}
  )

  add_dependencies(
    ${_target_name}
    ${rosidl_generate_interfaces_TARGET}${_target_suffix}
    ${rosidl_generate_interfaces_TARGET}__rosidl_generator_c
  )

  set(_extension_compile_flags "")
  set(_PYTHON_EXECUTABLE ${PYTHON_EXECUTABLE})
  if(NOT WIN32)
    set(_extension_compile_flags "-Wall -Wextra")
  elseif("${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
    set(PYTHON_EXECUTABLE ${PYTHON_EXECUTABLE_DEBUG})
  endif()
  set_properties("")
  if(WIN32)
    set_properties("_DEBUG")
    set_properties("_MINSIZEREL")
    set_properties("_RELEASE")
    set_properties("_RELWITHDEBINFO")
  endif()
  target_link_libraries(
    ${_target_name}
    ${PythonExtra_LIBRARIES}
    ${PROJECT_NAME}__${_typesupport_impl}
  )

  target_include_directories(${_target_name}
    PUBLIC
    ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_c
    ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_py
    ${PythonExtra_INCLUDE_DIRS}
  )

  ament_target_dependencies(${_target_name}
    "rosidl_generator_c"
    "${_typesupport_impl}"
  )
  foreach(_pkg_name ${rosidl_generate_interfaces_DEPENDENCY_PACKAGE_NAMES})
    ament_target_dependencies(${_target_name}
      ${_pkg_name}
    )
  endforeach()

  add_dependencies(${_target_name}
    ${rosidl_generate_interfaces_TARGET}__${_typesupport_impl}
  )
  ament_target_dependencies(${_target_name}
    "rosidl_generator_c"
    "rosidl_generator_py"
    "${PROJECT_NAME}__rosidl_generator_c"
  )
  set(PYTHON_EXECUTABLE ${_PYTHON_EXECUTABLE})

  if(NOT rosidl_generate_interfaces_SKIP_INSTALL)
    install(TARGETS ${_target_name}
      DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}")
  endif()
endforeach()

if(BUILD_TESTING AND rosidl_generate_interfaces_ADD_LINTER_TESTS)
  if(
    NOT _generated_msg_py_files STREQUAL "" OR
    NOT _generated_msg_c_files STREQUAL "" OR
    NOT _generated_msg_c_common_files STREQUAL "" OR
    NOT _generated_srv_py_files STREQUAL "" OR
    NOT _generated_srv_c_files STREQUAL ""
  )
    find_package(ament_cmake_cppcheck REQUIRED)
    ament_cppcheck(
      TESTNAME "cppcheck_rosidl_generated_py"
      "${_output_path}")

    find_package(ament_cmake_cpplint REQUIRED)
    get_filename_component(_cpplint_root "${_output_path}" DIRECTORY)
    ament_cpplint(
      TESTNAME "cpplint_rosidl_generated_py"
      # the generated code might contain longer lines for templated types
      MAX_LINE_LENGTH 999
      ROOT "${_cpplint_root}"
      "${_output_path}")

    find_package(ament_cmake_pep8 REQUIRED)
    ament_pep8(
      TESTNAME "pep8_rosidl_generated_py"
      # the generated code might contain longer lines for templated types
      MAX_LINE_LENGTH 999
      "${_output_path}")

    find_package(ament_cmake_pep257 REQUIRED)
    ament_pep257(
      TESTNAME "pep257_rosidl_generated_py"
      "${_output_path}")

    find_package(ament_cmake_pyflakes REQUIRED)
    ament_pyflakes(
      TESTNAME "pyflakes_rosidl_generated_py"
      "${_output_path}")

    find_package(ament_cmake_uncrustify REQUIRED)
    ament_uncrustify(
      TESTNAME "uncrustify_rosidl_generated_py"
      # the generated code might contain longer lines for templated types
      MAX_LINE_LENGTH 999
      "${_output_path}")
  endif()
endif()
