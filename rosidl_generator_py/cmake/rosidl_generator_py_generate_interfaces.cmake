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
find_package(PythonLibs REQUIRED)

set(_output_path
  "${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_py/${PROJECT_NAME}")
set(_generated_msg_py_files "")
set(_generated_msg_c_files "")
set(_generated_srv_files "")
foreach(_idl_file ${rosidl_generate_interfaces_IDL_FILES})
  get_filename_component(_parent_folder "${_idl_file}" DIRECTORY)
  get_filename_component(_parent_folder "${_parent_folder}" NAME)
  get_filename_component(_msg_name "${_idl_file}" NAME_WE)
  string_camel_case_to_lower_case_underscore("${_msg_name}" _module_name)

  if("${_parent_folder} " STREQUAL "msg ")
    list(APPEND _generated_msg_py_files
      "${_output_path}/${_parent_folder}/_${_module_name}.py"
    )
    list(APPEND _generated_msg_c_files
      "${_output_path}/${_parent_folder}/_${_module_name}_support.c"
    )
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



set(_generated_extension_files "")
set(_extension_dependencies "")
set(_target_suffix "__py")


macro(target_interface)
  #get_rmw_typesupport(typesupport_impls ${rmw_implementation})
  set(typesupport_impls "rosidl_typesupport_introspection_c")
  foreach(typesupport_impl ${typesupport_impls})
    find_package(${typesupport_impl} REQUIRED)


    add_custom_command(
      OUTPUT ${_generated_msg_py_files} ${_generated_msg_c_files} ${_generated_srv_files}
      COMMAND ${PYTHON_EXECUTABLE} ${rosidl_generator_py_BIN}
      --generator-arguments-file "${generator_arguments_file}"
      --typesupport-impl "${typesupport_impl}"
      DEPENDS ${target_dependencies}
      COMMENT "Generating Python code for ROS interfaces"
      VERBATIM
    )

    set(_pyext_suffix "__pyext__${typesupport_impl}")

    if(TARGET ${_msg_name}${_pyext_suffix})
      message(STATUS "Whoops, we got an existing message support library")
    else()
    add_library(${_msg_name}${_pyext_suffix} SHARED
      ${_generated_msg_c_file})

    # TODO(esteve): obtain extension suffix (e.g. cpython-34m) via
    # sysconfig.get_config_var('EXT_SUFFIX') or sysconfig.get_config_var('SOABI')
    # See PEP-3149: https://www.python.org/dev/peps/pep-3149/

    set_target_properties(${_msg_name}${_pyext_suffix} PROPERTIES
      COMPILE_FLAGS "${_extension_compile_flags}" PREFIX ""
      OUTPUT_NAME "${_msg_name}__${typesupport_impl}.cpython-34m")

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
      ${PROJECT_NAME}__${typesupport_impl}
    )

    get_property(_extension_location TARGET "${_msg_name}${_pyext_suffix}" PROPERTY LOCATION)
    list(APPEND _generated_extension_files
      "${_extension_location}"
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
    )

    list(APPEND _extension_dependencies ${_msg_name}${_pyext_suffix})


    message(STATUS "Targeting dependency on ${typesupport_impl} for ${_msg_name}${_pyext_suffix}")
    ament_target_dependencies(${_msg_name}${_pyext_suffix}
      ${typesupport_impl}
    )
    add_dependencies(${_msg_name}${_pyext_suffix}
      ${rosidl_generate_interfaces_TARGET}__${typesupport_impl}
    )

    endif()
  endforeach()
  ament_target_dependencies(${_msg_name}${_pyext_suffix}
    "rosidl_generator_c"
    "rosidl_generator_py"
    "${PROJECT_NAME}__rosidl_generator_c"
  )
endmacro()

foreach(_generated_msg_c_file ${_generated_msg_c_files})
  get_filename_component(_parent_folder "${_generated_msg_c_file}" DIRECTORY)
  get_filename_component(_parent_folder "${_parent_folder}" NAME)
  get_filename_component(_msg_name "${_generated_msg_c_file}" NAME_WE)

  #call_for_each_rmw_implementation(target_interface)
  target_interface("")

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
    _ament_cmake_python_get_python_install_dir()

    list(GET _generated_msg_py_files 0 _msg_file)
    get_filename_component(_msg_package_dir "${_msg_file}" DIRECTORY)
    get_filename_component(_msg_package_dir "${_msg_package_dir}" DIRECTORY)

    install(
      FILES "${_msg_package_dir}/__init__.py"
      DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
    )

    install(
      FILES ${_generated_msg_py_files}
      DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}/msg"
    )

    install(
      FILES ${_generated_extension_files}
      DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}/msg"
    )

    if(NOT "${_generated_srv_files} " STREQUAL " ")
      install(
        FILES ${_generated_srv_files}
        DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}/srv"
      )
    endif()

# NOTE(esteve): this should work, but there must be something wrong with the dependencies
# because when this is invoked, it invariably fails to find the generated __init__.py file
# in builtin_interfaces. However, this doesn't fail when using install()
#    list(GET _generated_msg_py_files 0 _msg_file)
#    get_filename_component(_msg_package_dir "${_msg_file}" DIRECTORY)
#    get_filename_component(_msg_package_dir "${_msg_package_dir}" DIRECTORY)
#    ament_python_install_package("${PROJECT_NAME}" PACKAGE_DIR "${_msg_package_dir}")

  endif()
endif()
ament_export_include_directories(include)
