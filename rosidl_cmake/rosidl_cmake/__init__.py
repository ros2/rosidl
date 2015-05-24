# Copyright 2015 Open Source Robotics Foundation, Inc.
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

import os

from rosidl_parser import BaseType
from rosidl_parser import PACKAGE_NAME_MESSAGE_TYPE_SEPARATOR


def extract_message_types(pkg_name, ros_interface_files, deps):
    types = []

    for ros_interface_file in ros_interface_files:
        base_type = _get_base_type(pkg_name, ros_interface_file)
        if base_type:
            types.append(base_type)

    for dep in deps:
        # only take the first : for separation, as Windows follows with a C:\
        dep_parts = dep.split(':', 1)
        assert(len(dep_parts) == 2)
        pkg_name = dep_parts[0]
        base_type = _get_base_type(pkg_name, dep_parts[1])
        if base_type:
            types.append(base_type)

    return types


def _get_base_type(pkg_name, idl_path):
    idl_filename = os.path.basename(idl_path)
    msg_name, extension = os.path.splitext(idl_filename)
    if extension != '.msg':
        return None
    return BaseType(pkg_name + PACKAGE_NAME_MESSAGE_TYPE_SEPARATOR + msg_name)
