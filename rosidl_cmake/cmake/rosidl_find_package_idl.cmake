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

# Get the absolute path of an IDL file stored in a package's share
# directory, checking if the package's find module set a GNUInstallDirs
# pkgname_DATADIR variable pointing to a separate location from the
# default pkgname_DIR.
#
# :param var: A name of a variable to store the idl file absolute path.
# :param pkg_name: Name of the package, assumed to have already been
#   find_package'd.
# :type pkg_name: string
# :param idl_name: The filename of the idl file to look for.
# :type idl_name: string
#
# @public
#
function(rosidl_find_package_idl var pkg_name idl_file)
  set(_candidates
    "${${pkg_name}_DATADIR}/${idl_file}"
    "${${pkg_name}_DIR}/../${idl_file}"
  )
  foreach(_try ${_candidates})
    if(EXISTS "${_try}")
      normalize_path(_norm "${_try}")
      set("${var}" "${_norm}" PARENT_SCOPE)
      return()
    endif()
  endforeach()
  message(FATAL_ERROR "Unable to find ${idl_file} in any of ${_candidates} for ${pkg_name}.")
  set("${var}" "-NOTFOUND" PARENT_SCOPE)
endfunction()
