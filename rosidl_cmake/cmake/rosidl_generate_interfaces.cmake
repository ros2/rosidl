# include CMake functions
include(CMakeParseArguments)

#
# Generate code for ROS IDL files using all available generators.
#
# Execute the extension point ``rosidl_generate_interfaces``.
#
# :param target: the name of the generation target,
#   specific generators might use the name as a prefix for their own
#   generation step
# :type target: string
# :param ARGN: a list of include directories where each value might
#   be either an absolute path or path relative to the
#   CMAKE_INSTALL_PREFIX.
# :type ARGN: list of strings
# :param DEPENDENCIES: the packages from which message types are
#   being used
# :type DEPENDENCIES: list of strings
# :param SKIP_INSTALL: if set skip installing the interface files
# :type SKIP_INSTALL: option
#
# @public
#
macro(rosidl_generate_interfaces target)
  #message(" - rosidl_generate_interfaces(${target} ${ARGN})")

  cmake_parse_arguments(_ARG "SKIP_INSTALL" "" "DEPENDENCIES" ${ARGN})
  if(NOT _ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "rosidl_generate_interfaces() called without any idl "
      "files")
  endif()

  if(_${PROJECT_NAME}_AMENT_PACKAGE)
    message(FATAL_ERROR
      "rosidl_generate_interfaces() must be called before ament_package()")
  endif()

  _rosidl_cmake_register_package_hook()
  ament_export_dependencies(${_ARG_DEPENDENCIES})

  _rosidl_generate_interfaces(${target} ${ARGN})
endmacro()

function(_rosidl_generate_interfaces target)
  cmake_parse_arguments(ARG "SKIP_INSTALL" "" "DEPENDENCIES" ${ARGN})

  # check all interface files
  set(idl_files "")
  foreach(idl_file ${ARG_UNPARSED_ARGUMENTS})
    if(NOT IS_ABSOLUTE "${idl_file}")
      set(idl_file "${CMAKE_CURRENT_SOURCE_DIR}/${idl_file}")
    endif()
    if(NOT EXISTS "${idl_file}")
      message(FATAL_ERROR "rosidl_generate_interfaces() the passed idl file "
        "'${idl_file}' does not exist")
    endif()
    #message("   - generate interface for: ${idl_file}")
    list(APPEND idl_files "${idl_file}")
  endforeach()

  # collect all interface files from dependencies
  set(dep_files)
  foreach(dep ${ARG_DEPENDENCIES})
    if(NOT ${dep}_FOUND)
      message(FATAL_ERROR "rosidl_generate_interfaces() the passed dependency "
        "'${dep}' has not been found before using find_package()")
    endif()
    foreach(idl_file ${${dep}_INTERFACE_FILES})
      list(APPEND dep_files "${${dep}_DIR}/../${idl_file}")
    endforeach()
  endforeach()

  # stamp all interface files
  foreach(idl_file ${idl_files})
    stamp("${idl_file}")
  endforeach()

  add_custom_target(
    ${target} ALL
    DEPENDS
    ${idl_files}
    ${dep_files}
    SOURCES
    ${idl_files}
  )

  # generators must be executed in topological order
  # which is ensured by every generator finding its dependencies first
  # and then registering itself as an extension
  set(rosidl_generate_interfaces_TARGET ${target})
  set(rosidl_generate_interfaces_IDL_FILES ${idl_files})
  set(rosidl_generate_interfaces_DEPENDENCY_PACKAGE_NAMES ${ARG_DEPENDENCIES})
  ament_execute_extensions("rosidl_generate_interfaces")

  if(NOT SKIP_INSTALL)
    # generate derived interface files
    # foreach(idl_file ${idl_files})
    #   get_filename_component(extension "${idl_file}" EXT)
    #   # generate services and feedback messages for actions
    #   if("${extension}" STREQUAL ".action")
    #     file(READ "${idl_file}" action_content)
    #     string(REGEX REPLACE "(.*\n--\n.*\n)--\n.*" "\\1" service_content "${action_content}")
    #     string(REGEX REPLACE ".*\n--\n.*\n--\n(.*)" "\\1" feedback_content "${action_content}")
    #     get_filename_component(name "${idl_file}" NAME_WE)
    #     set(service_file "${CMAKE_CURRENT_BINARY_DIR}/srv/${name}.srv")
    #     set(feedback_file "${CMAKE_CURRENT_BINARY_DIR}/msg/${name}_Feedback.msg")
    #     set(feedback_with_header_file "${CMAKE_CURRENT_BINARY_DIR}/msg/${name}_FeedbackWithHeader.msg")
    #     file(WRITE "${service_file}" "${service_content}")
    #     file(WRITE "${feedback_file}" "${feedback_content}")
    #     file(WRITE "${feedback_with_header_file}" "rosidl_cmake/FeedbackHeader header\n${name}_Feedback feedback\n")
    #     list(APPEND idl_files "${service_file}" "${feedback_file}" "${feedback_with_header_file}")
    #   endif()
    # endforeach()
    # foreach(idl_file ${idl_files})
    #   get_filename_component(extension "${idl_file}" EXT)
    #   # generate request and response messages for services
    #   if("${extension}" STREQUAL ".srv")
    #     file(READ "${idl_file}" service_content)
    #     string(REGEX REPLACE "(.*\n)--\n.*" "\\1" request_content "${service_content}")
    #     string(REGEX REPLACE ".*\n--\n(.*)" "\\1" response_content "${service_content}")
    #     get_filename_component(name "${idl_file}" NAME_WE)
    #     set(request_file "${CMAKE_CURRENT_BINARY_DIR}/msg/${name}_Request.msg")
    #     set(request_with_header_file "${CMAKE_CURRENT_BINARY_DIR}/msg/${name}_RequestWithHeader.msg")
    #     set(response_file "${CMAKE_CURRENT_BINARY_DIR}/msg/${name}_Response.msg")
    #     set(response_with_header_file "${CMAKE_CURRENT_BINARY_DIR}/msg/${name}_ResponseWithHeader.msg")
    #     file(WRITE "${request_file}" "${request_content}")
    #     file(WRITE "${request_with_header_file}" "rosidl_cmake/RequestHeader header\n${name}_Request request\n")
    #     file(WRITE "${response_file}" "${response_content}")
    #     file(WRITE "${response_with_header_file}" "rosidl_cmake/ResponseHeader header\n${name}_Response reponse\n")
    #     list(APPEND idl_files "${request_file}" "${response_file}" "${request_with_header_file}" "${response_with_header_file}")
    #   endif()
    # endforeach()

    # install interface files to subfolders based on their extension
    foreach(idl_file ${idl_files})
      get_filename_component(extension "${idl_file}" EXT)
      string(SUBSTRING "${extension}" 1 -1 extension)
      install(
        FILES ${idl_file}
        DESTINATION "share/${PROJECT_NAME}/${extension}"
      )
      get_filename_component(name "${idl_file}" NAME)
      list(APPEND _rosidl_cmake_INTERFACE_FILES "${extension}/${name}")
    endforeach()
    set(_rosidl_cmake_INTERFACE_FILES ${_rosidl_cmake_INTERFACE_FILES} PARENT_SCOPE)
  endif()
endfunction()
