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

import importlib

type_support_map = {
    'rmw_connext_cpp': 'rosidl_typesupport_connext_c',
    'rmw_opensplice_cpp': 'rosidl_typesupport_opensplice_c',
}


class UnsupportedTypeSupport(Exception):
    """Raised when no supported type support can be found for a given rmw implementation."""

    def __init__(self, rmw_implementation):
        message = "No supported type support for '{0}'".format(rmw_implementation)
        super(UnsupportedTypeSupport, self).__init__(message)
        self.rmw_implementation = rmw_implementation


def import_type_support(pkg_name, subfolder, rosidl_name, rmw_implementation):
    """Import the appropriate type support module for a given rosidl and rmw implementation.

    This function will determine the correct type support package to import
    from based on the rmw implementation given.
    For example, giving `opensplice_static` as the rmw implementation would
    result in the `rosidl_typesupport_opensplice_c` type support package being
    used when importing.

    :param pkg_name str: name of the package which contains the rosidl
    :param subfolder str: name of the rosidl containing folder, e.g. `msg`, `srv`, etc.
    :param rosidl_name str: name of the rosidl
    :param rmw_implementation str: name of the rmw implementation
    :returns: the type support Python module for this specific rosidl and rmw implementation pair
    """
    if rmw_implementation not in type_support_map.keys():
        raise UnsupportedTypeSupport(rmw_implementation)
    type_support_name = type_support_map[rmw_implementation]
    import_package = '{pkg_name}.{subfolder}'.format(
        pkg_name=pkg_name,
        subfolder=subfolder,
    )
    module_name = '._{rosidl_name}_s__{type_support_name}'.format(
        rosidl_name=rosidl_name,
        type_support_name=type_support_name,
    )
    return importlib.import_module(module_name, package=import_package)
