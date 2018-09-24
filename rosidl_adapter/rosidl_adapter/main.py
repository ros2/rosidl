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

import argparse
import os
import pathlib
import sys


from rosidl_adapter import convert_to_idl
from rosidl_adapter import InterfaceType


def main(argv=sys.argv[1:]):
    parser = argparse.ArgumentParser(
        description='Convert interface files to .idl')
    parser.add_argument(
        '--package-dir', required=True,
        help='The base directory of the package')
    parser.add_argument(
        '--package-name', required=True,
        help='The name of the package')
    parser.add_argument(
        '--message-files', nargs='*',
        help='The message files to convert to .idl')
    parser.add_argument(
        '--service-files', nargs='*',
        help='The service files to convert to .idl')
    parser.add_argument(
        '--interface-files', nargs='*',
        help='The interface files to convert to .idl')
    parser.add_argument(
        '--output-dir', required=True,
        help='The base directory to create .idl files in')
    parser.add_argument(
        '--output-messages-file', required=True,
        help='The output file containing the absolute paths to the message '
             '.idl files')
    parser.add_argument(
        '--output-services-file', required=True,
        help='The output file containing the absolute paths to the service '
             '.idl files')
    args = parser.parse_args(argv)
    output_dir = pathlib.Path(args.output_dir)

    message_files = []
    service_files = []

    for message_file in args.message_files:
        message_file = pathlib.Path(message_file)
        interface_type, idl_file = convert_to_idl(
            args.package_dir, args.package_name, message_file, output_dir,
            interface_type = InterfaceType.MESSAGE)
        if interface_type != InterfaceType.MESSAGE:
            raise ValueError("Expected message but was something else")
        message_files.append(idl_file)

    for service_file in args.service_files:
        service_file = pathlib.Path(message_file)
        interface_type, idl_file = convert_to_idl(
            args.package_dir, args.package_name, service_file, output_dir,
            interface_type = InterfaceType.SERVICE)
        if interface_type != InterfaceType.SERVICE:
            raise ValueError("Expected service but was something else")
        service_files.append(idl_file)

    for interface_file in args.interface_files:
        interface_file = pathlib.Path(interface_file)

        interface_type, idl_file = convert_to_idl(
            args.package_dir, args.package_name, interface_file,
            output_dir)
        if interface_type == InterfaceType.MESSAGE:
            message_files.append(idl_file)
        elif interface_type == InterfaceType.SERVICE:
            service_files.append(idl_file)
        elif interface_type == InterfaceType.UNKNOWN:
            raise RuntimeError(
                'Unsupported interface file format: {interface_file}'
                .format_map(locals()))
        else:
            assert False

    os.makedirs(os.path.dirname(args.output_messages_file), exist_ok=True)
    with open(args.output_messages_file, 'w') as h:
        for message_file in message_files:
            h.write('{message_file}\n'.format_map(locals()))
    os.makedirs(os.path.dirname(args.output_services_file), exist_ok=True)
    with open(args.output_services_file, 'w') as h:
        for service_file in service_files:
            h.write('{service_file}\n'.format_map(locals()))
