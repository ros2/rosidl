# Copyright 2023 Open Source Robotics Foundation, Inc.
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

#
# Generate a JSON / YAML file containing additional context data for expanding
# IDL templates
#
#
# @public
#
function(rosidl_write_additional_context output_file)
  
  set(OPTIONAL_ONE_VALUE_KEYWORDS
    "DISABLE_DESCRIPTION_CODEGEN")
  set(OPTIONAL_MULTI_VALUE_KEYWORDS
    "TYPE_SUPPORTS")

  cmake_parse_arguments(
    ARG
    ""
    "${OPTIONAL_ONE_VALUE_KEYWORDS}"
    "${OPTIONAL_MULTI_VALUE_KEYWORDS}"
    ${ARGN})
  if(ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "rosidl_write_additional_context() called with unused "
      "arguments: ${ARG_UNPARSED_ARGUMENTS}")
  endif()

  # create folder
  get_filename_component(output_path "${output_file}" PATH)
  file(MAKE_DIRECTORY "${output_path}")

  # open object
  file(WRITE "${output_file}"
    "{")

  set(first_element TRUE)

  # write string values
  foreach(one_value_argument ${OPTIONAL_ONE_VALUE_KEYWORDS})
    if(DEFINED ARG_${one_value_argument})
      # write conditional comma and mandatory newline
      if(NOT first_element)
        file(APPEND "${output_file}" ",")
      else()
        set(first_element FALSE)
      endif()
      file(APPEND "${output_file}" "\n")

      string(TOLOWER "${one_value_argument}" key)
      string(REPLACE "\\" "\\\\" value "${ARG_${one_value_argument}}")
      file(APPEND "${output_file}"
        "  \"${key}\": \"${value}\"")
    endif()
  endforeach()

  # write array values
  foreach(multi_value_argument ${OPTIONAL_MULTI_VALUE_KEYWORDS})
    if(ARG_${multi_value_argument})
      # write conditional comma and mandatory newline and indentation
      if(NOT first_element)
        file(APPEND "${output_file}" ",")
      else()
        set(first_element FALSE)
      endif()
      file(APPEND "${output_file}" "\n")

      # write key, open array
      string(TOLOWER "${multi_value_argument}" key)
      file(APPEND "${output_file}"
        "  \"${key}\": [\n")

      # write array values, last without trailing colon
      list(GET ARG_${multi_value_argument} -1 last_value)
      list(REMOVE_AT ARG_${multi_value_argument} -1)
      foreach(value ${ARG_${multi_value_argument}})
        string(REPLACE "\\" "\\\\" value "${value}")
        file(APPEND "${output_file}"
          "    \"${value}\",\n")
      endforeach()
      string(REPLACE "\\" "\\\\" last_value "${last_value}")
      file(APPEND "${output_file}"
        "    \"${last_value}\"\n")

      # close array
      file(APPEND "${output_file}"
        "  ]")
    endif()
  endforeach()

  # close object
  file(APPEND "${output_file}"
    "\n}\n")
endfunction()
