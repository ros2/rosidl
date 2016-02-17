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

# NOTE(esteve): required for CMake-2.8 in Ubuntu 14.04
set(Python_ADDITIONAL_VERSIONS 3.4)
find_package(PythonInterp 3.4 REQUIRED)

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

    set(PythonExtra_LIBRARIES "${PYTHON_LIBRARY}")
    message(STATUS "Using PythonExtra_LIBRARIES: ${PythonExtra_LIBRARIES}")
  else()
    find_package(PythonLibs 3.4 REQUIRED)
    message("PYTHON LIBS: ${PYTHON_LIBRARIES}")
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
    set(PythonExtra_EXTENSION_SUFFIX
      ""
      CACHE INTERNAL
      "The full suffix for Python native extensions. See PEP-3149: https://www.python.org/dev/peps/pep-3149/."
    )
  else()
    if("${PYTHON_MULTIARCH} " STREQUAL " " OR "${PYTHON_SOABI} " STREQUAL "None ")
      set(PythonExtra_EXTENSION_SUFFIX
        ".${PYTHON_SOABI}"
        CACHE INTERNAL
        "The full suffix for Python native extensions. See PEP-3149: https://www.python.org/dev/peps/pep-3149/."
      )
    else()
      set(PythonExtra_EXTENSION_SUFFIX
        ".${PYTHON_SOABI}-${PYTHON_MULTIARCH}"
        CACHE INTERNAL
        "The full suffix for Python native extensions. See PEP-3149: https://www.python.org/dev/peps/pep-3149/."
      )
    endif()
  endif()

  if(WIN32)
    set(PythonExtra_EXTENSION_EXTENSION ".pyd")
  else()
    # Also use .so for OSX, not dylib
    set(PythonExtra_EXTENSION_EXTENSION ".so")
  endif()

  set(PythonExtra_FOUND TRUE)
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(PythonExtra
  FOUND_VAR PythonExtra_FOUND
  REQUIRED_VARS
    PythonExtra_EXTENSION_EXTENSION
    PythonExtra_EXTENSION_SUFFIX
    PythonExtra_INCLUDE_DIRS
    PythonExtra_LIBRARIES
)
