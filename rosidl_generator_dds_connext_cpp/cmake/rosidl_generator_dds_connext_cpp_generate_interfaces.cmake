message(" - rosidl_generator_dds_connext_cpp_generate_interfaces.cmake")
message("   - target: ${rosidl_generate_interfaces_TARGET}")
message("   - interface files: ${rosidl_generate_interfaces_IDL_FILES}")
message("   - dependency package names: ${rosidl_generate_interfaces_DEPENDENCY_PACKAGE_NAMES}")

set(_dds_idl_files "")
set(_dds_idl_path "${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_dds_idl/${PROJECT_NAME}")
foreach(_idl_file ${rosidl_generate_interfaces_IDL_FILES})
  get_filename_component(name "${_idl_file}" NAME_WE)
  list(APPEND _dds_idl_files "${_dds_idl_path}/${name}_.idl")
endforeach()

set(_output_path "${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_dds_connext_cpp/${PROJECT_NAME}/dds_idl")
set(_generated_files "")
foreach(_idl_file ${rosidl_generate_interfaces_IDL_FILES})
  get_filename_component(name "${_idl_file}" NAME_WE)
  list(APPEND _generated_files "${_output_path}/${name}_.h")
  list(APPEND _generated_files "${_output_path}/${name}_.cxx")
  list(APPEND _generated_files "${_output_path}/${name}_Plugin.h")
  list(APPEND _generated_files "${_output_path}/${name}_Plugin.cxx")
  list(APPEND _generated_files "${_output_path}/${name}_Support.h")
  list(APPEND _generated_files "${_output_path}/${name}_Support.cxx")
  list(APPEND _generated_files "${_output_path}/${name}_TypeSupport.cpp")
endforeach()

set(_dependency_files "")
set(_dependencies "")
foreach(_pkg_name ${rosidl_generate_interfaces_DEPENDENCY_PACKAGE_NAMES})
  foreach(_idl_file ${${_pkg_name}_INTERFACE_FILES})
    get_filename_component(name "${_idl_file}" NAME_WE)
    set(_abs_idl_file "${${_pkg_name}_DIR}/../dds_idl/${name}_.idl")
    list(APPEND _dependency_files "${_abs_idl_file}")
    list(APPEND _dependencies "${_pkg_name}:${_abs_idl_file}")
  endforeach()
endforeach()

message("   - generated files: ${_generated_files}")
message("   - dependencies: ${_dependencies}")

add_custom_command(
  OUTPUT ${_generated_files}
  COMMAND ${PYTHON_EXECUTABLE} ${rosidl_generator_dds_connext_cpp_BIN}
  --pkg-name ${PROJECT_NAME}
  --ros-interface-files ${rosidl_generate_interfaces_IDL_FILES}
  --interface-files ${_dds_idl_files}
  --deps ${_dependencies}
  --output-dir "${_output_path}"
  --idl-pp "${CONNEXT_DDSGEN2}"
  --template-dir ${rosidl_generator_dds_connext_cpp_TEMPLATE_DIR}
  DEPENDS
  ${rosidl_generator_dds_connext_cpp_BIN}
  ${rosidl_generator_dds_connext_cpp_DIR}/../../../${PYTHON_INSTALL_DIR}/rosidl_generator_dds_connext_cpp/__init__.py
  ${rosidl_generator_dds_connext_cpp_TEMPLATE_DIR}/msg_TypeSupport.cpp.template
  ${_dds_idl_files}
  ${_dependency_files}
  COMMENT "Generating C++ interfaces for RTI Connext"
  VERBATIM
)

set(_target_suffix "__dds_connext_cpp")

set(CMAKE_CXX_FLAGS "-std=c++0x")

add_library(${rosidl_generate_interfaces_TARGET}${_target_suffix} SHARED ${_generated_files})
target_include_directories(${rosidl_generate_interfaces_TARGET}${_target_suffix}
  PUBLIC
  ${CONNEXT_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_dds_connext_cpp
  ${rosidl_generator_cpp_INCLUDE_DIRS}
)
target_compile_definitions(${rosidl_generate_interfaces_TARGET}${_target_suffix}
  PUBLIC ${CONNEXT_DEFINITIONS})
foreach(_pkg_name ${rosidl_generate_interfaces_DEPENDENCY_PACKAGE_NAMES})
  target_include_directories(${rosidl_generate_interfaces_TARGET}${_target_suffix}
    PUBLIC
    ${${_pkg_name}_INCLUDE_DIRS}
    ${${_pkg_name}_DIR}/../../../include/${_pkg_name}/dds_idl
  )
  target_link_libraries(${rosidl_generate_interfaces_TARGET}${_target_suffix}
    ${${_pkg_name}_LIBRARIES})
endforeach()
target_link_libraries(${rosidl_generate_interfaces_TARGET}${_target_suffix} ${CONNEXT_LIBRARIES})

add_dependencies(
  ${rosidl_generate_interfaces_TARGET}
  ${rosidl_generate_interfaces_TARGET}${_target_suffix}
)
add_dependencies(
  ${rosidl_generate_interfaces_TARGET}${_target_suffix}
  ${rosidl_generate_interfaces_TARGET}_cpp
)
add_dependencies(
  ${rosidl_generate_interfaces_TARGET}_dds_idl
  ${rosidl_generate_interfaces_TARGET}${_target_suffix}
)

install(
  FILES ${_generated_files}
  DESTINATION "include/${PROJECT_NAME}/dds_idl"
)
install(
  TARGETS ${rosidl_generate_interfaces_TARGET}${_target_suffix}
  DESTINATION "lib"
)

ament_export_include_directories(include)
ament_export_libraries(${rosidl_generate_interfaces_TARGET}${_target_suffix} ${CONNEXT_LIBRARIES})
