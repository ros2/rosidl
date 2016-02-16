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

find_package(rosidl_generator_c REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)
find_package(rmw REQUIRED)

# NOTE(esteve): required for CMake-2.8 in Ubuntu 14.04
set(Python_ADDITIONAL_VERSIONS 3.4)
find_package(PythonInterp 3.4 REQUIRED)

if(APPLE)
  find_program(PYTHON_CONFIG_EXECUTABLE NAMES "python3-config")
  if(NOT PYTHON_CONFIG_EXECUTABLE)
    message(FATAL_ERROR "Cannot find python3-config executable")
  endif()

  if(NOT DEFINED PYTHON_INCLUDE_DIRS)
    execute_process(
      COMMAND
      "${PYTHON_CONFIG_EXECUTABLE}"
      "--includes"
      OUTPUT_VARIABLE _output
      RESULT_VARIABLE _result
      OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    if(NOT _result EQUAL 0)
      message(FATAL_ERROR
        "execute_process(${PYTHON_CONFIG_EXECUTABLE} --includes) returned "
        "error code ${_result}")
    endif()

    string(REPLACE " " ";" _output_list ${_output})

    foreach(_includedir ${_output_list})
      string(SUBSTRING "${_includedir}" 2 -1 _includedir)
      list(APPEND PYTHON_INCLUDE_DIRS "${_includedir}")
    endforeach()
  endif()
  set(PYTHON_INCLUDE_DIRS
      ${PYTHON_INCLUDE_DIRS}
      CACHE INTERNAL
      "The paths to the Python include directories.")
  message(STATUS "Using PYTHON_INCLUDE_DIRS: ${PYTHON_INCLUDE_DIRS}")

  if(NOT DEFINED PYTHON_LIBRARIES)
    execute_process(
      COMMAND
      "${PYTHON_CONFIG_EXECUTABLE}"
      "--ldflags"
      OUTPUT_VARIABLE _output
      RESULT_VARIABLE _result
      OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    if(NOT _result EQUAL 0)
      message(FATAL_ERROR
        "execute_process(${PYTHON_CONFIG_EXECUTABLE} --ldflags) returned "
        "error code ${_result}")
    endif()

    string(REPLACE " " ";" _output_list "${_output}")
    set(PYTHON_LIBRARIES
      ""
      CACHE INTERNAL
      "The libraries that need to be linked against for Python extensions.")

    set(_library_paths "")
    foreach(_item ${_output_list})
      string(REGEX MATCH "-L(.*)" _regex_match ${_item})
      if(NOT "${_regex_match} " STREQUAL " ")
        string(SUBSTRING "${_regex_match}" 2 -1 _library_path)
        list(APPEND _library_paths "${_library_path}")
      endif()
    endforeach()

    set(_python_version_no_dots
      "${PYTHON_VERSION_MAJOR}${PYTHON_VERSION_MINOR}")
    set(_python_version
      "${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}")

    find_library(PYTHON_LIBRARY
      NAMES
      python${_python_version_no_dots}
      python${_python_version}mu
      python${_python_version}m
      python${_python_version}u
      python${_python_version}
      PATHS
      ${_library_paths}
      NO_SYSTEM_ENVIRONMENT_PATH
    )
  endif()

  set(PYTHON_LIBRARIES "${PYTHON_LIBRARY}")
  message(STATUS "Using PYTHON_LIBRARIES: ${PYTHON_LIBRARIES}")
else()
  find_package(PythonLibs 3.4 REQUIRED)
endif()

if(NOT DEFINED PYTHON_SOABI)
  set(_python_code
    "from sysconfig import get_config_var"
    "print(get_config_var('SOABI'))"
  )
  execute_process(
    COMMAND
    "${PYTHON_EXECUTABLE}"
    "-c"
    "${_python_code}"
    OUTPUT_VARIABLE _output
    RESULT_VARIABLE _result
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
  if(NOT _result EQUAL 0)
    message(FATAL_ERROR
      "execute_process(${PYTHON_EXECUTABLE} -c '${_python_code}') returned "
      "error code ${_result}")
  endif()

  set(PYTHON_SOABI
    "${_output}"
    CACHE INTERNAL
    "The SOABI suffix for Python native extensions. See PEP-3149: https://www.python.org/dev/peps/pep-3149/.")
endif()

if(NOT DEFINED PYTHON_MULTIARCH)
  set(_python_code
    "from sysconfig import get_config_var"
    "print(get_config_var('MULTIARCH'))"
  )
  execute_process(
    COMMAND
    "${PYTHON_EXECUTABLE}"
    "-c"
    "${_python_code}"
    OUTPUT_VARIABLE _output
    RESULT_VARIABLE _result
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
  if(NOT _result EQUAL 0)
    message(FATAL_ERROR
      "execute_process(${PYTHON_EXECUTABLE} -c '${_python_code}') returned "
      "error code ${_result}")
  endif()

  set(PYTHON_MULTIARCH
    "${_output}"
    CACHE INTERNAL
    "The MULTIARCH suffix for Python native extensions. See PEP-3149: https://www.python.org/dev/peps/pep-3149/.")
endif()

if("${PYTHON_SOABI} " STREQUAL " " OR "${PYTHON_SOABI} " STREQUAL "None ")
  set(PYTHON_EXTENSION_SUFFIX
    ""
    CACHE INTERNAL
    "The full suffix for Python native extensions. See PEP-3149: https://www.python.org/dev/peps/pep-3149/."
  )
else()
  if("${PYTHON_MULTIARCH} " STREQUAL " " OR "${PYTHON_SOABI} " STREQUAL "None ")
    set(PYTHON_EXTENSION_SUFFIX
      ".${PYTHON_SOABI}"
      CACHE INTERNAL
      "The full suffix for Python native extensions. See PEP-3149: https://www.python.org/dev/peps/pep-3149/."
    )
  else()
    set(PYTHON_EXTENSION_SUFFIX
      ".${PYTHON_SOABI}-${PYTHON_MULTIARCH}"
      CACHE INTERNAL
      "The full suffix for Python native extensions. See PEP-3149: https://www.python.org/dev/peps/pep-3149/."
    )
  endif()
endif()


# TODO(esteve): force opensplice and connext C type supports only, uncomment
# the following line when all typesupport implementations are ported to C
#get_rmw_typesupport(_typesupport_impls ${rmw_implementation})
set(_typesupport_impls "")
foreach(_extension IN LISTS AMENT_EXTENSIONS_rosidl_generate_interfaces)
  string(REPLACE ":" ";" _extension_list "${_extension}")
  list(LENGTH _extension_list _length)
  if(NOT _length EQUAL 2)
    message(FATAL_ERROR "ament_execute_extensions(${extension_point}) "
      "registered extension '${_extension}' can not be split into package "
      "name and cmake filename")
  endif()
  list(GET _extension_list 0 _pkg_name)
  list(GET _extension_list 1 _cmake_filename)
  if("${_pkg_name} " STREQUAL "rosidl_typesupport_opensplice_c ")
    list(APPEND _typesupport_impls "rosidl_typesupport_opensplice_c")
  endif()
  if("${_pkg_name} " STREQUAL "rosidl_typesupport_connext_c ")
    list(APPEND _typesupport_impls "rosidl_typesupport_connext_c")
  endif()
endforeach()

set(_output_path
  "${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_py/${PROJECT_NAME}")
set(_generated_msg_py_files "")
set(_generated_msg_c_files "")
set(_generated_msg_c_common_files "")
set(_generated_msg_c_ts_files "")
set(_generated_srv_files "")
foreach(_idl_file ${rosidl_generate_interfaces_IDL_FILES})
  get_filename_component(_parent_folder "${_idl_file}" DIRECTORY)
  get_filename_component(_parent_folder "${_parent_folder}" NAME)
  get_filename_component(_msg_name1 "${_idl_file}" NAME_WE)
  string_camel_case_to_lower_case_underscore("${_msg_name1}" _module_name)

  if("${_parent_folder} " STREQUAL "msg ")
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
      list(APPEND _generated_msg_c_files
        "${_output_path}/${_parent_folder}/_${_module_name}_s.ep.${_typesupport_impl}.c"
      )
      list(APPEND _generated_msg_c_ts_files
        "${_output_path}/${_parent_folder}/_${_module_name}_s.ep.${_typesupport_impl}.c"
      )
      list(APPEND _type_support_by_generated_msg_c_files "${_typesupport_impl}")
    endforeach()
  elseif("${_parent_folder} " STREQUAL "srv ")
    list(APPEND _generated_srv_files
      "${_output_path}/${_parent_folder}/_${_module_name}.py"
    )
  else()
    message(FATAL_ERROR "Interface file with unknown parent folder: ${_idl_file}")
  endif()
endforeach()

if(NOT "${_generated_msg_py_files} " STREQUAL " ")
  list(GET _generated_msg_py_files 0 _msg_file)
  get_filename_component(_parent_folder "${_msg_file}" DIRECTORY)
  list(APPEND _generated_msg_py_files
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
  "${rosidl_generator_py_TEMPLATE_DIR}/_msg_support.c.template"
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

set(_generated_extension_files "")
set(_extension_dependencies "")
set(_target_suffix "__py")

if(WIN32)
  set(PYTHON_EXTENSION_EXTENSION ".pyd")
else()
  # Also use .so for OSX, not dylib
  set(PYTHON_EXTENSION_EXTENSION ".so")
endif()

add_custom_command(
  # OUTPUT ${_generated_msg_py_files} ${_generated_msg_c__${_typesupport_impl}_files} ${_generated_srv_files}
  OUTPUT ${_generated_msg_py_files} ${_generated_msg_c_files} ${_generated_srv_files}
  COMMAND ${PYTHON_EXECUTABLE} ${rosidl_generator_py_BIN}
  --generator-arguments-file "${generator_arguments_file}"
  --typesupport-impl "${_typesupport_impl}"
  --typesupport-impls "${_typesupport_impls}"
  DEPENDS ${target_dependencies}
  COMMENT "Generating Python code for ROS interfaces"
  VERBATIM
)

foreach(_generated_msg_c_ts_file ${_generated_msg_c_ts_files})
  get_filename_component(_full_folder "${_generated_msg_c_ts_file}" DIRECTORY)
  get_filename_component(_parent_folder "${_full_folder}" NAME)
  get_filename_component(_base_msg_name "${_generated_msg_c_ts_file}" NAME_WE)
  get_filename_component(_full_extension_msg_name "${_generated_msg_c_ts_file}" EXT)

  set(_msg_name "${_base_msg_name}${_full_extension_msg_name}")

  # foreach(_typesupport_impl ${_typesupport_impls})
    list(FIND _generated_msg_c_ts_files ${_generated_msg_c_ts_file} _file_index)
    list(GET _type_support_by_generated_msg_c_files ${_file_index} _typesupport_impl)
    find_package(${_typesupport_impl} REQUIRED)
    set(_generated_msg_c_common_file "${_full_folder}/${_base_msg_name}.c")

    set(_pyext_suffix "__pyext")

    # TODO(esteve): Change the following code so that each file is compiled independently
    add_library(${_msg_name}${_pyext_suffix} SHARED
      "${_generated_msg_c_ts_file}"
      "${_generated_msg_c_common_file}"
    )

    set_target_properties(${_msg_name}${_pyext_suffix} PROPERTIES
      COMPILE_FLAGS "${_extension_compile_flags}" PREFIX ""
      LIBRARY_OUTPUT_DIRECTORY "${_output_path}/${_parent_folder}"
      RUNTIME_OUTPUT_DIRECTORY "${_output_path}/${_parent_folder}"
      OUTPUT_NAME "${_base_msg_name}__${_typesupport_impl}${PYTHON_EXTENSION_SUFFIX}"
      SUFFIX "${PYTHON_EXTENSION_EXTENSION}")

    set_target_properties(${_msg_name}${_pyext_suffix} PROPERTIES
      COMPILE_FLAGS "${_extension_compile_flags}" PREFIX ""
      LIBRARY_OUTPUT_DIRECTORY_DEBUG "${_output_path}/${_parent_folder}"
      RUNTIME_OUTPUT_DIRECTORY_DEBUG "${_output_path}/${_parent_folder}"
      OUTPUT_NAME "${_base_msg_name}__${_typesupport_impl}${PYTHON_EXTENSION_SUFFIX}"
      SUFFIX "${PYTHON_EXTENSION_EXTENSION}")

    set_target_properties(${_msg_name}${_pyext_suffix} PROPERTIES
      COMPILE_FLAGS "${_extension_compile_flags}" PREFIX ""
      LIBRARY_OUTPUT_DIRECTORY_RELEASE "${_output_path}/${_parent_folder}"
      RUNTIME_OUTPUT_DIRECTORY_RELEASE "${_output_path}/${_parent_folder}"
      OUTPUT_NAME "${_base_msg_name}__${_typesupport_impl}${PYTHON_EXTENSION_SUFFIX}"
      SUFFIX "${PYTHON_EXTENSION_EXTENSION}")

    set_target_properties(${_msg_name}${_pyext_suffix} PROPERTIES
      COMPILE_FLAGS "${_extension_compile_flags}" PREFIX ""
      LIBRARY_OUTPUT_DIRECTORY_RELWITHDEBINFO "${_output_path}/${_parent_folder}"
      RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO "${_output_path}/${_parent_folder}"
      OUTPUT_NAME "${_base_msg_name}__${_typesupport_impl}${PYTHON_EXTENSION_SUFFIX}"
      SUFFIX "${PYTHON_EXTENSION_EXTENSION}")

    set_target_properties(${_msg_name}${_pyext_suffix} PROPERTIES
      COMPILE_FLAGS "${_extension_compile_flags}" PREFIX ""
      LIBRARY_OUTPUT_DIRECTORY_MINSIZEREL "${_output_path}/${_parent_folder}"
      RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL "${_output_path}/${_parent_folder}"
      OUTPUT_NAME "${_base_msg_name}__${_typesupport_impl}${PYTHON_EXTENSION_SUFFIX}"
      SUFFIX "${PYTHON_EXTENSION_EXTENSION}")

    add_dependencies(
      ${_msg_name}${_pyext_suffix}
      ${rosidl_generate_interfaces_TARGET}__rosidl_generator_c
    )

    set(_extension_compile_flags "")
    if(NOT WIN32)
      set(_extension_compile_flags "-Wall -Wextra")
    endif()

    target_link_libraries(
      ${_msg_name}${_pyext_suffix}
      ${PYTHON_LIBRARIES}
      ${PROJECT_NAME}__${_typesupport_impl}
    )

    list(APPEND _generated_extension_files
      "$<TARGET_FILE:${_msg_name}${_pyext_suffix}>"
    )

    target_include_directories(${_msg_name}${_pyext_suffix}
      PUBLIC
      ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_c
      ${PYTHON_INCLUDE_DIRS}
    )

    foreach(_pkg_name ${rosidl_generate_interfaces_DEPENDENCY_PACKAGE_NAMES})
      ament_target_dependencies(
        ${_msg_name}${_pyext_suffix}
        ${_pkg_name}
      )
    endforeach()
    ament_target_dependencies(${_msg_name}${_pyext_suffix}
      "rosidl_generator_c"
      "${_typesupport_impl}"
    )

    list(APPEND _extension_dependencies ${_msg_name}${_pyext_suffix})

    ament_target_dependencies(${_msg_name}${_pyext_suffix}
      ${_typesupport_impl}
    )
    add_dependencies(${_msg_name}${_pyext_suffix}
      ${rosidl_generate_interfaces_TARGET}__${_typesupport_impl}
    )

  # endforeach()
  ament_target_dependencies(${_msg_name}${_pyext_suffix}
    "rosidl_generator_c"
    "rosidl_generator_py"
    "${PROJECT_NAME}__rosidl_generator_c"
  )
endforeach()

if(TARGET ${rosidl_generate_interfaces_TARGET}${_target_suffix})
  message(WARNING "Custom target ${rosidl_generate_interfaces_TARGET}${_target_suffix} already exists")
else()
  add_custom_target(
    ${rosidl_generate_interfaces_TARGET}${_target_suffix}
    DEPENDS
    ${_generated_msg_py_files}
    ${_generated_msg_c_files}
    ${_generated_srv_files}
    ${_generated_extension_files}
    ${_extension_dependencies}
    ${rosidl_generate_interfaces_TARGET}__rosidl_generator_c
  )
endif()

if(NOT rosidl_generate_interfaces_SKIP_INSTALL)
  if(NOT "${_generated_msg_py_files} " STREQUAL " ")
    list(GET _generated_msg_py_files 0 _msg_file)
    get_filename_component(_msg_package_dir "${_msg_file}" DIRECTORY)
    get_filename_component(_msg_package_dir "${_msg_package_dir}" DIRECTORY)
    ament_python_install_package("${PROJECT_NAME}" PACKAGE_DIR "${_msg_package_dir}")
  endif()
endif()
