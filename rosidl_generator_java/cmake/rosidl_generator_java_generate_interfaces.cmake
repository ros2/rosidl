# Copyright 2016 Esteve Fernandez <esteve@apache.org>
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

find_package(ament_cmake_export_jars REQUIRED)
find_package(rosidl_generator_c REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)
find_package(rmw REQUIRED)

find_host_package(Java COMPONENTS Development)
include (UseJava)

if(NOT ANDROID)
  find_package(JNI REQUIRED)
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
  "${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_java/${PROJECT_NAME}")
set(_generated_msg_java_files "")
set(_generated_msg_cpp_files "")
set(_generated_msg_cpp_common_files "")
set(_generated_msg_cpp_ts_files "")
set(_generated_srv_files "")
foreach(_idl_file ${rosidl_generate_interfaces_IDL_FILES})
  get_filename_component(_parent_folder "${_idl_file}" DIRECTORY)
  get_filename_component(_parent_folder "${_parent_folder}" NAME)
  get_filename_component(_module_name "${_idl_file}" NAME_WE)

  if("${_parent_folder} " STREQUAL "msg ")
    list(APPEND _generated_msg_java_files
      "${_output_path}/${_parent_folder}/${_module_name}.java"
    )

    foreach(_typesupport_impl ${_typesupport_impls})
      list(APPEND _generated_msg_cpp_files
        "${_output_path}/${_parent_folder}/${_module_name}_s.ep.${_typesupport_impl}.cpp"
      )
      list(APPEND _generated_msg_cpp_ts_files
        "${_output_path}/${_parent_folder}/${_module_name}_s.ep.${_typesupport_impl}.cpp"
      )
      list(APPEND _type_support_by_generated_msg_cpp_files "${_typesupport_impl}")
    endforeach()
  elseif("${_parent_folder} " STREQUAL "srv ")
    list(APPEND _generated_srv_files
      "${_output_path}/${_parent_folder}/_${_module_name}.py"
    )
  else()
    message(FATAL_ERROR "Interface file with unknown parent folder: ${_idl_file}")
  endif()
endforeach()

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
  "${rosidl_generator_java_BIN}"
  ${rosidl_generator_java_GENERATOR_FILES}
  "${rosidl_generator_java_TEMPLATE_DIR}/msg_support.entry_point.cpp.template"
  "${rosidl_generator_java_TEMPLATE_DIR}/msg.java.template"
  "${rosidl_generator_java_TEMPLATE_DIR}/srv.java.template"
  ${rosidl_generate_interfaces_IDL_FILES}
  ${_dependency_files})
foreach(dep ${target_dependencies})
  if(NOT EXISTS "${dep}")
    message(FATAL_ERROR "Target dependency '${dep}' does not exist")
  endif()
endforeach()

set(generator_arguments_file "${CMAKE_BINARY_DIR}/rosidl_generator_java__arguments.json")
rosidl_write_generator_arguments(
  "${generator_arguments_file}"
  PACKAGE_NAME "${PROJECT_NAME}"
  ROS_INTERFACE_FILES "${rosidl_generate_interfaces_IDL_FILES}"
  ROS_INTERFACE_DEPENDENCIES "${_dependencies}"
  OUTPUT_DIR "${_output_path}"
  TEMPLATE_DIR "${rosidl_generator_java_TEMPLATE_DIR}"
  TARGET_DEPENDENCIES ${target_dependencies}
)

file(MAKE_DIRECTORY "${_output_path}")

set(_generated_extension_files "")
set(_extension_dependencies "")
set(_target_suffix "__java")

add_custom_command(
  OUTPUT ${_generated_msg_cpp_common_files} ${_generated_msg_java_files} ${_generated_msg_cpp_ts_files} ${_generated_msg_cpp_files} ${_generated_srv_files}
  COMMAND ${PYTHON_EXECUTABLE} ${rosidl_generator_java_BIN}
  --generator-arguments-file "${generator_arguments_file}"
  --typesupport-impl "${_typesupport_impl}"
  --typesupport-impls "${_typesupport_impls}"
  DEPENDS ${target_dependencies}
  COMMENT "Generating Java code for ROS interfaces"
  VERBATIM
)

foreach(_generated_msg_cpp_ts_file ${_generated_msg_cpp_ts_files})
  get_filename_component(_full_folder "${_generated_msg_cpp_ts_file}" DIRECTORY)
  get_filename_component(_parent_folder "${_full_folder}" NAME)
  get_filename_component(_base_msg_name "${_generated_msg_cpp_ts_file}" NAME_WE)
  get_filename_component(_full_extension_msg_name "${_generated_msg_cpp_ts_file}" EXT)

  set(_msg_name "${_base_msg_name}${_full_extension_msg_name}")

  list(FIND _generated_msg_cpp_ts_files ${_generated_msg_cpp_ts_file} _file_index)
  list(GET _type_support_by_generated_msg_cpp_files ${_file_index} _typesupport_impl)
  find_package(${_typesupport_impl} REQUIRED)
  set(_generated_msg_cpp_common_file "${_full_folder}/${_base_msg_name}.cpp")

  set(_javaext_suffix "__javaext")

  add_library(${_msg_name}${_javaext_suffix} SHARED
    "${_generated_msg_cpp_ts_file}"
  )

  set_target_properties(${_msg_name}${_javaext_suffix} PROPERTIES
    COMPILE_FLAGS "${_extension_compile_flags}"
    LIBRARY_OUTPUT_DIRECTORY "${_output_path}/${_parent_folder}"
    RUNTIME_OUTPUT_DIRECTORY "${_output_path}/${_parent_folder}"
    OUTPUT_NAME "${_base_msg_name}__${_typesupport_impl}"
  )

  set_target_properties(${_msg_name}${_javaext_suffix} PROPERTIES
    COMPILE_FLAGS "${_extension_compile_flags}"
    LIBRARY_OUTPUT_DIRECTORY_DEBUG "${_output_path}/${_parent_folder}"
    RUNTIME_OUTPUT_DIRECTORY_DEBUG "${_output_path}/${_parent_folder}"
    OUTPUT_NAME "${_base_msg_name}__${_typesupport_impl}"
  )

  set_target_properties(${_msg_name}${_javaext_suffix} PROPERTIES
    COMPILE_FLAGS "${_extension_compile_flags}"
    LIBRARY_OUTPUT_DIRECTORY_RELEASE "${_output_path}/${_parent_folder}"
    RUNTIME_OUTPUT_DIRECTORY_RELEASE "${_output_path}/${_parent_folder}"
    OUTPUT_NAME "${_base_msg_name}__${_typesupport_impl}"
  )

  set_target_properties(${_msg_name}${_javaext_suffix} PROPERTIES
    COMPILE_FLAGS "${_extension_compile_flags}"
    LIBRARY_OUTPUT_DIRECTORY_RELWITHDEBINFO "${_output_path}/${_parent_folder}"
    RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO "${_output_path}/${_parent_folder}"
    OUTPUT_NAME "${_base_msg_name}__${_typesupport_impl}"
  )

  set_target_properties(${_msg_name}${_javaext_suffix} PROPERTIES
    COMPILE_FLAGS "${_extension_compile_flags}"
    LIBRARY_OUTPUT_DIRECTORY_MINSIZEREL "${_output_path}/${_parent_folder}"
    RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL "${_output_path}/${_parent_folder}"
    OUTPUT_NAME "${_base_msg_name}__${_typesupport_impl}"
  )

  add_dependencies(
    ${_msg_name}${_javaext_suffix}
    ${rosidl_generate_interfaces_TARGET}__rosidl_generator_c
  )

  set(_extension_compile_flags "")
  if(NOT WIN32)
    set(_extension_compile_flags "-Wall -Wextra")
  endif()

  target_link_libraries(
    ${_msg_name}${_javaext_suffix}
    ${PROJECT_NAME}__${_typesupport_impl}
  )

  target_include_directories(${_msg_name}${_javaext_suffix}
    PUBLIC
    ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_c
    ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_java
    ${JNI_INCLUDE_DIRS}
  )

  foreach(_pkg_name ${rosidl_generate_interfaces_DEPENDENCY_PACKAGE_NAMES})
    ament_target_dependencies(
      ${_msg_name}${_javaext_suffix}
      ${_pkg_name}
    )
  endforeach()

  ament_target_dependencies(${_msg_name}${_javaext_suffix}
    "rosidl_generator_c"
    "${_typesupport_impl}"
  )

  list(APPEND _extension_dependencies ${_msg_name}${_javaext_suffix})

  ament_target_dependencies(${_msg_name}${_javaext_suffix}
    ${_typesupport_impl}
  )
  add_dependencies(${_msg_name}${_javaext_suffix}
    ${rosidl_generate_interfaces_TARGET}__${_typesupport_impl}
  )

  if(NOT rosidl_generate_interfaces_SKIP_INSTALL)
    ament_export_libraries("${_base_msg_name}__${_typesupport_impl}")

    install(TARGETS ${_msg_name}${_javaext_suffix}
      DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}/${_msg_package_dir2}"
      ARCHIVE DESTINATION lib
      LIBRARY DESTINATION lib
    )
  endif()

  ament_target_dependencies(${_msg_name}${_javaext_suffix}
    "rosidl_generator_c"
    "rosidl_generator_java"
    "${PROJECT_NAME}__rosidl_generator_c"
  )

endforeach()

set(_jar_deps "")
foreach(_pkg_name ${rosidl_generate_interfaces_DEPENDENCY_PACKAGE_NAMES})
  find_package(${_pkg_name} REQUIRED)
  foreach(_jar_dep ${${_pkg_name}_JARS})
    list(APPEND _jar_deps "${_jar_dep}")
  endforeach()
endforeach()

add_jar("${PROJECT_NAME}_jar"
  "${_generated_msg_java_files}"
  OUTPUT_NAME
  "${PROJECT_NAME}"
  INCLUDE_JARS
  "${_jar_deps}"
)

get_property(_jar_file TARGET "${PROJECT_NAME}_jar" PROPERTY "JAR_FILE")

if(TARGET ${rosidl_generate_interfaces_TARGET}${_target_suffix})
  message(WARNING "Custom target ${rosidl_generate_interfaces_TARGET}${_target_suffix} already exists")
else()
  add_custom_target(
    ${rosidl_generate_interfaces_TARGET}${_target_suffix}
    ALL DEPENDS
    ${_generated_msg_java_files}
    ${_generated_msg_cpp_files}
    ${_generated_srv_files}
    ${_generated_extension_files}
    ${_extension_dependencies}
    ${rosidl_generate_interfaces_TARGET}__rosidl_generator_c
  )
endif()

if(NOT rosidl_generate_interfaces_SKIP_INSTALL)
  set(_install_jar_dir "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}")
  if(NOT "${_generated_msg_java_files} " STREQUAL " ")
    list(GET _generated_msg_java_files 0 _msg_file)
    get_filename_component(_msg_package_dir "${_msg_file}" DIRECTORY)
    get_filename_component(_msg_package_dir "${_msg_package_dir}" DIRECTORY)

    install_jar("${PROJECT_NAME}_jar" "share/java")
    ament_export_jars("share/java/${PROJECT_NAME}.jar")
  endif()
endif()
