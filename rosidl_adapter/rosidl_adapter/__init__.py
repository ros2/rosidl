# Copyright 2018 Open Source Robotics Foundation, Inc.
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

from enum import Enum


class InterfaceType(Enum):
    UNKNOWN = 0
    MESSAGE = 1
    SERVICE = 2


def convert_to_idl(
    package_dir, package_name, interface_file, output_dir,
    interface_type=InterfaceType.UNKNOWN
):
    print('convert_to_idl', interface_file)
    # TODO use plugin approach rather than hard coded alternatives
    if interface_file.suffix == '.idl':
        assert interface_type != InterfaceType.UNKNOWN, \
            '.idl files must be passed explicitly as a message or service file'
        # just pass through for now
        # TODO support e.g. multiple entities in a single input file?
        return (interface_type, interface_file)

    if interface_file.suffix == '.msg':
        assert interface_type in (InterfaceType.UNKNOWN, InterfaceType.MESSAGE)
        from rosidl_adapter.msg import convert_msg_to_idl
        return (InterfaceType.MESSAGE, convert_msg_to_idl(
            package_dir, package_name, interface_file,
            output_dir / package_name / 'msg'))

    if interface_file.suffix == '.srv':
        assert interface_type in (InterfaceType.UNKNOWN, InterfaceType.SERVICE)
        from rosidl_adapter.srv import convert_srv_to_idl
        return (InterfaceType.SERVICE, convert_srv_to_idl(
            package_dir, package_name, interface_file,
            output_dir / package_name / 'srv'))

    # unknown / unhandled for now
    return (InterfaceType.UNKNOWN, interface_file)
