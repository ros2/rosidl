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

macro(rosidl_generator_py_get_typesupports TYPESUPPORT_IMPLS)
  set(${TYPESUPPORT_IMPLS} "")
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
      list(APPEND ${TYPESUPPORT_IMPLS} "rosidl_typesupport_opensplice_c")
    endif()
    if("${_pkg_name} " STREQUAL "rosidl_typesupport_connext_c ")
      list(APPEND ${TYPESUPPORT_IMPLS} "rosidl_typesupport_connext_c")
    endif()
  endforeach()
endmacro()
