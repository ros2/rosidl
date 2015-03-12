message(" - rosidl_typesupport_introspection_cpp_generate_interfaces.cmake")
message("   - target: ${rosidl_generate_interfaces_TARGET}")
message("   - interface files: ${rosidl_generate_interfaces_IDL_FILES}")
message("   - dependency package names: ${rosidl_generate_interfaces_DEPENDENCY_PACKAGE_NAMES}")

set(_output_path "${CMAKE_CURRENT_BINARY_DIR}/rosidl_typesupport_introspection_cpp/${PROJECT_NAME}")
set(_generated_files "")
foreach(_idl_file ${rosidl_generate_interfaces_IDL_FILES})
  get_filename_component(_extension "${_idl_file}" EXT)
  if("${_extension}" STREQUAL ".msg")
    get_filename_component(name "${_idl_file}" NAME_WE)
    list(APPEND _generated_files
      "${_output_path}/${name}_TypeSupport_Introspection.cpp"
    )
  elseif("${_extension}" STREQUAL ".srv")
    get_filename_component(name "${_idl_file}" NAME_WE)
    list(APPEND _generated_files
      "${_output_path}/${name}_ServiceTypeSupport_Introspection.cpp"
    )
  endif()
endforeach()

set(_dependency_files "")
set(_dependencies "")
foreach(_pkg_name ${rosidl_generate_interfaces_DEPENDENCY_PACKAGE_NAMES})
  foreach(_idl_file ${${_pkg_name}_INTERFACE_FILES})
  get_filename_component(_extension "${_idl_file}" EXT)
  if("${_extension}" STREQUAL ".msg")
    set(_abs_idl_file "${${_pkg_name}_DIR}/../${_idl_file}")
    normalize_path(_abs_idl_file "${_abs_idl_file}")
    list(APPEND _dependency_files "${_abs_idl_file}")
    list(APPEND _dependencies "${_pkg_name}:${_abs_idl_file}")
  endif()
  endforeach()
endforeach()

message("   - generated files: ${_generated_files}")
message("   - dependencies: ${_dependencies}")

add_custom_command(
  OUTPUT ${_generated_files}
  COMMAND ${PYTHON_EXECUTABLE} ${rosidl_typesupport_introspection_cpp_BIN}
  --pkg-name ${PROJECT_NAME}
  --ros-interface-files ${rosidl_generate_interfaces_IDL_FILES}
  --deps ${_dependencies}
  --output-dir ${_output_path}
  --template-dir ${rosidl_typesupport_introspection_cpp_TEMPLATE_DIR}
  DEPENDS
  ${rosidl_typesupport_introspection_cpp_BIN}
  ${rosidl_typesupport_introspection_cpp_GENERATOR_FILES}
  ${rosidl_typesupport_introspection_cpp_TEMPLATE_DIR}/msg_TypeSupport_Introspection.cpp.template
  ${rosidl_typesupport_introspection_cpp_TEMPLATE_DIR}/srv_ServiceTypeSupport_Introspection.cpp.template
  ${rosidl_generate_interfaces_IDL_FILES}
  ${_dependency_files}
  COMMENT "Generating C++ introspection for ROS interfaces"
  VERBATIM
)

set(_target_suffix "__typesupport_introspection_cpp")

add_library(${rosidl_generate_interfaces_TARGET}${_target_suffix} SHARED ${_generated_files})
if(WIN32)
  target_compile_definitions(${rosidl_generate_interfaces_TARGET}${_target_suffix}
    PRIVATE "ROSIDL_TSI_CPP_BUILDING_DLL")
endif()
target_include_directories(${rosidl_generate_interfaces_TARGET}${_target_suffix}
  PUBLIC
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)
foreach(_pkg_name ${rosidl_generate_interfaces_DEPENDENCY_PACKAGE_NAMES})
  ament_target_dependencies(
    ${rosidl_generate_interfaces_TARGET}${_target_suffix}
    ${_pkg_name})
endforeach()
ament_target_dependencies(${rosidl_generate_interfaces_TARGET}${_target_suffix}
  "rosidl_generator_cpp"
  "rosidl_typesupport_introspection_cpp")

add_dependencies(
  ${rosidl_generate_interfaces_TARGET}
  ${rosidl_generate_interfaces_TARGET}${_target_suffix}
)
add_dependencies(
  ${rosidl_generate_interfaces_TARGET}${_target_suffix}
  ${rosidl_generate_interfaces_TARGET}__cpp
)

install(
  TARGETS ${rosidl_generate_interfaces_TARGET}${_target_suffix}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_libraries(${rosidl_generate_interfaces_TARGET}${_target_suffix})
