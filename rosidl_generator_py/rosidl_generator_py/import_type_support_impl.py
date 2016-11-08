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

import ament_index_python
import importlib


class UnsupportedTypeSupport(Exception):
    """Raised when no supported type support can be found for a given rmw implementation."""

    def __init__(self, rmw_implementation):
        message = "No supported type support for '{0}'".format(rmw_implementation)
        super(UnsupportedTypeSupport, self).__init__(message)
        self.rmw_implementation = rmw_implementation


def import_type_support(pkg_name, rmw_implementation):
    """
    Import the appropriate type support module of a package for a given rmw implementation.

    This function will determine the correct type support module to import based on the rmw
    implementation given.
    This module will provide the c type support of the given rmw implementation for the rosidl
    files in the specified package, such that the ROS message structures in the package can be
    converted to and from message structures used by the rmw implementation.

    :param pkg_name str: name of the package
    :param rmw_implementation str: name of the rmw implementation
    :returns: the type support Python module for the specified package and rmw implementation pair
    """
    if not ament_index_python.has_resource('rmw_typesupport_c', rmw_implementation):
        raise UnsupportedTypeSupport(rmw_implementation)

    type_support_name, _ = ament_index_python.get_resource('rmw_typesupport_c', rmw_implementation)
    import_package = '{pkg_name}'.format(
        pkg_name=pkg_name,
    )
    module_name = '.{pkg_name}_s__{type_support_name}'.format(
        pkg_name=pkg_name,
        type_support_name=type_support_name,
    )
    return importlib.import_module(module_name, package=import_package)
