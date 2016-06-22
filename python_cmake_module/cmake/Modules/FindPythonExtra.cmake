# Copyright 2016 Open Source Robotics Foundation, Inc.
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


###############################################################################
#
# CMake module for providing extra information about the Python interpreter.
#
# Output variables:
#
# - PythonExtra_EXTENSION_SUFFIX: The suffix for a Python extension, according
#    to PEP-3149: https://www.python.org/dev/peps/pep-3149/
# - PythonExtra_EXTENSION_EXTENSION: The extension for a Python extension. On
#    Linux and Mac OSX equals to ".so", on Windows to ".pyd"
# - PythonExtra_INCLUDE_DIRS: The paths to the directories where the Python
#    headers are installed.
# - PythonExtra_LIBRARIES: The paths to the Python libraries.
#
# Example usage:
#
#   find_package(python_cmake_module REQUIRED)
#   find_package(PythonExtra MODULE)
#   # use PythonExtra_* variables
#
###############################################################################

# lint_cmake: -convention/filename, -package/stdargs

set(PythonExtra_FOUND FALSE)

find_package(PythonInterp 3.5 REQUIRED)

if(PYTHONINTERP_FOUND)
  if(APPLE)
    find_program(PYTHON_CONFIG_EXECUTABLE NAMES "python3-config")
    if(NOT PYTHON_CONFIG_EXECUTABLE)
      message(FATAL_ERROR "Cannot find python3-config executable")
    endif()

    if(NOT DEFINED PythonExtra_INCLUDE_DIRS)
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
        list(APPEND PythonExtra_INCLUDE_DIRS "${_includedir}")
      endforeach()
    endif()
    set(PythonExtra_INCLUDE_DIRS
        ${PythonExtra_INCLUDE_DIRS}
        CACHE INTERNAL
        "The paths to the Python include directories.")
    message(STATUS "Using PythonExtra_INCLUDE_DIRS: ${PythonExtra_INCLUDE_DIRS}")

    if(NOT DEFINED PythonExtra_LIBRARIES)
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
      set(PythonExtra_LIBRARIES
        ""
        CACHE INTERNAL
        "The libraries that need to be linked against for Python extensions.")

      set(_library_paths "")
      foreach(_item ${_output_list})
        string(REGEX MATCH "-L(.*)" _regex_match ${_item})
        if(NOT _regex_match STREQUAL "")
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

    set(PythonExtra_LIBRARIES "${PYTHON_LIBRARY}")
    message(STATUS "Using PythonExtra_LIBRARIES: ${PythonExtra_LIBRARIES}")
  else()
    find_package(PythonLibs 3.5 REQUIRED)
    if(WIN32 AND CMAKE_BUILD_TYPE STREQUAL "Debug")
      get_filename_component(_python_executable_dir "${PYTHON_EXECUTABLE}" DIRECTORY)
      get_filename_component(_python_executable_name "${PYTHON_EXECUTABLE}" NAME_WE)
      get_filename_component(_python_executable_ext "${PYTHON_EXECUTABLE}" EXT)
      set(_python_executable_debug "${_python_executable_dir}/${_python_executable_name}_d${_python_executable_ext}")
      if(EXISTS "${_python_executable_debug}")
        set(PYTHON_EXECUTABLE_DEBUG "${_python_executable_debug}")
      else()
        message(FATAL_ERROR "${_python_executable_debug} doesn't exist")
      endif()
    endif()
    message(STATUS "Using PYTHON_EXECUTABLE: ${PYTHON_EXECUTABLE}")
    message(STATUS "Using PYTHON_INCLUDE_DIRS: ${PYTHON_INCLUDE_DIRS}")
    message(STATUS "Using PYTHON_LIBRARIES: ${PYTHON_LIBRARIES}")
    set(PythonExtra_INCLUDE_DIRS "${PYTHON_INCLUDE_DIRS}")
    set(PythonExtra_LIBRARIES "${PYTHON_LIBRARIES}")
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

  if(PYTHON_SOABI STREQUAL "" OR PYTHON_SOABI STREQUAL "None")
    set(PythonExtra_EXTENSION_SUFFIX
      ""
      CACHE INTERNAL
      "The full suffix for Python native extensions. See PEP-3149: https://www.python.org/dev/peps/pep-3149/."
    )
  else()
    set(PythonExtra_EXTENSION_SUFFIX
      ".${PYTHON_SOABI}"
      CACHE INTERNAL
      "The full suffix for Python native extensions. See PEP-3149: https://www.python.org/dev/peps/pep-3149/."
    )
  endif()

  if(WIN32)
    if(CMAKE_BUILD_TYPE STREQUAL "Debug")
      set(PythonExtra_EXTENSION_EXTENSION "_d.pyd")
    else()
      set(PythonExtra_EXTENSION_EXTENSION ".pyd")
    endif()
  else()
    # Also use .so for OSX, not dylib
    set(PythonExtra_EXTENSION_EXTENSION ".so")
  endif()

  set(PythonExtra_FOUND TRUE)
endif()

include(FindPackageHandleStandardArgs)
set(_required_vars
  PythonExtra_EXTENSION_EXTENSION
  PythonExtra_INCLUDE_DIRS
  PythonExtra_LIBRARIES)
if(NOT WIN32)
  list(APPEND _required_vars PythonExtra_EXTENSION_SUFFIX)
elseif("${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
  list(APPEND _required_vars PYTHON_EXECUTABLE_DEBUG)
endif()
find_package_handle_standard_args(PythonExtra
  FOUND_VAR PythonExtra_FOUND
  REQUIRED_VARS ${_required_vars}
)
