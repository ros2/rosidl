# Copyright 2014-2015 Open Source Robotics Foundation, Inc.
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

from rosidl_adapter.msg.parser import BaseType
from rosidl_adapter.msg.parser import InvalidSpecification
from rosidl_adapter.msg.parser import parse_message_string
from rosidl_adapter.msg.parser import UnknownMessageType

SERVICE_REQUEST_RESPONSE_SEPARATOR = '---'
SERVICE_REQUEST_MESSAGE_SUFFIX = '_Request'
SERVICE_RESPONSE_MESSAGE_SUFFIX = '_Response'


class InvalidServiceSpecification(InvalidSpecification):
    pass


class ServiceSpecification:

    def __init__(self, pkg_name, srv_name, request_message, response_message):
        self.pkg_name = pkg_name
        self.srv_name = srv_name
        self.request = request_message
        self.response = response_message


def parse_service_file(pkg_name, interface_filename):
    basename = os.path.basename(interface_filename)
    srv_name = os.path.splitext(basename)[0]
    with open(interface_filename, 'r') as h:
        return parse_service_string(
            pkg_name, srv_name, h.read())


def parse_service_string(pkg_name, srv_name, message_string):
    lines = message_string.splitlines()
    separator_indices = [
        index for index, line in enumerate(lines) if line == SERVICE_REQUEST_RESPONSE_SEPARATOR]
    if not separator_indices:
        raise InvalidServiceSpecification(
            "Could not find separator '%s' between request and response" %
            SERVICE_REQUEST_RESPONSE_SEPARATOR)
    if len(separator_indices) != 1:
        raise InvalidServiceSpecification(
            "Could not find unique separator '%s' between request and response" %
            SERVICE_REQUEST_RESPONSE_SEPARATOR)

    request_message_string = '\n'.join(lines[:separator_indices[0]])
    request_message = parse_message_string(
        pkg_name, srv_name + SERVICE_REQUEST_MESSAGE_SUFFIX, request_message_string)

    response_message_string = '\n'.join(lines[separator_indices[0] + 1:])
    response_message = parse_message_string(
        pkg_name, srv_name + SERVICE_RESPONSE_MESSAGE_SUFFIX, response_message_string)

    return ServiceSpecification(pkg_name, srv_name, request_message, response_message)


def validate_srv_field_types(spec, known_msg_types):
    for service_part in ('request', 'response'):
        msg_spec = getattr(spec, service_part)
        for field in msg_spec.fields:
            if field.type.is_primitive_type():
                continue
            base_type = BaseType(BaseType.__str__(field.type))
            if base_type not in known_msg_types:
                raise UnknownMessageType(
                    "Service {service_part} interface '{msg_spec.base_type}' "
                    '"contains an unknown field type: {field}'
                    .format_map(locals()))
