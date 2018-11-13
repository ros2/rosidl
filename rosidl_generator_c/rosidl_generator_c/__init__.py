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

from rosidl_cmake import convert_camel_case_to_lower_case_underscore
from rosidl_cmake import expand_template
from rosidl_cmake import get_newest_modification_time
from rosidl_cmake import read_generator_arguments
from rosidl_parser import parse_action_file
from rosidl_parser import parse_message_file
from rosidl_parser import parse_service_file


def generate_c(generator_arguments_file):
    args = read_generator_arguments(generator_arguments_file)

    template_dir = args['template_dir']
    mapping_msgs = {
        os.path.join(template_dir, 'msg.h.em'): '%s.h',
        os.path.join(template_dir, 'msg__functions.c.em'): '%s__functions.c',
        os.path.join(template_dir, 'msg__functions.h.em'): '%s__functions.h',
        os.path.join(template_dir, 'msg__struct.h.em'): '%s__struct.h',
        os.path.join(template_dir, 'msg__type_support.h.em'): '%s__type_support.h',
    }
    mapping_srvs = {
        os.path.join(template_dir, 'srv.h.em'): '%s.h',
    }
    mapping_action = {
        os.path.join(template_dir, 'action.h.em'): '%s.h',
        os.path.join(template_dir, 'action__type_support.h.em'): '%s__type_support.h',
    }
    for template_file in list(mapping_msgs.keys()) + list(mapping_srvs.keys()):
        assert os.path.exists(template_file), 'Could not find template: ' + template_file

    functions = {
        'get_header_filename_from_msg_name': convert_camel_case_to_lower_case_underscore,
    }
    latest_target_timestamp = get_newest_modification_time(args['target_dependencies'])

    for ros_interface_file in args['ros_interface_files']:
        extension = os.path.splitext(ros_interface_file)[1]
        subfolder = os.path.basename(os.path.dirname(ros_interface_file))
        if extension == '.msg':
            spec = parse_message_file(args['package_name'], ros_interface_file)
            for template_file, generated_filename in mapping_msgs.items():
                generated_file = os.path.join(
                    args['output_dir'], subfolder, generated_filename %
                    convert_camel_case_to_lower_case_underscore(spec.base_type.type))
                data = {
                    'spec': spec,
                    'pkg': spec.base_type.pkg_name,
                    'msg': spec.msg_name,
                    'type': spec.base_type.type,
                    'subfolder': subfolder,
                }
                data.update(functions)
                expand_template(
                    template_file, data, generated_file,
                    minimum_timestamp=latest_target_timestamp)
        elif extension == '.srv':
            spec = parse_service_file(args['package_name'], ros_interface_file)
            for template_file, generated_filename in mapping_srvs.items():
                data = {'spec': spec, 'subfolder': subfolder}
                data.update(functions)
                generated_file = os.path.join(
                    args['output_dir'], subfolder, generated_filename %
                    convert_camel_case_to_lower_case_underscore(spec.srv_name))
                expand_template(
                    template_file, data, generated_file,
                    minimum_timestamp=latest_target_timestamp)
        elif extension == '.action':
            spec = parse_action_file(args['package_name'], ros_interface_file)
            for template_file, generated_filename in mapping_action.items():
                data = {'spec': spec, 'subfolder': subfolder}
                data.update(functions)
                generated_file = os.path.join(
                    args['output_dir'], subfolder, generated_filename %
                    convert_camel_case_to_lower_case_underscore(spec.action_name))
                expand_template(
                    template_file, data, generated_file,
                    minimum_timestamp=latest_target_timestamp)
    return 0


MSG_TYPE_TO_C = {
    'bool': 'bool',
    'byte': 'uint8_t',
    'char': 'signed char',
    'float32': 'float',
    'float64': 'double',
    'uint8': 'uint8_t',
    'int8': 'int8_t',
    'uint16': 'uint16_t',
    'int16': 'int16_t',
    'uint32': 'uint32_t',
    'int32': 'int32_t',
    'uint64': 'uint64_t',
    'int64': 'int64_t',
    'string': 'rosidl_generator_c__String',
}


def get_typename_of_base_type(type_):
    if not type_.is_primitive_type():
        return '%s__%s__%s' % (type_.pkg_name, 'msg', type_.type)
    suffix = type_.type
    if suffix == 'string':
        suffix = 'String'
    return 'rosidl_generator_c__' + suffix


def primitive_msg_type_to_c(type_):
    return MSG_TYPE_TO_C[type_]


def msg_type_to_c(type_, name_):
    """
    Convert a message type into the C declaration.

    Example input: uint32, std_msgs/String
    Example output: uint32_t, char *

    @param type_: The message type
    @type type_: rosidl_parser.Type
    @param type_: The field name
    @type type_: str
    """
    c_type = None
    if type_.is_primitive_type():
        c_type = MSG_TYPE_TO_C[type_.type]
    else:
        c_type = '%s__msg__%s' % (type_.pkg_name, type_.type)

    if type_.is_array:
        if type_.array_size is None or type_.is_upper_bound:
            # Dynamic sized array
            if type_.is_primitive_type() and type_.type != 'string':
                c_type = 'rosidl_generator_c__%s' % type_.type
            return '%s__Sequence %s' % (c_type, name_)
        else:
            # Static sized array (field specific)
            return '%s %s[%d]' % \
                (c_type, name_, type_.array_size)
    else:
        return '%s %s' % (c_type, name_)


def value_to_c(type_, value):
    assert type_.is_primitive_type()
    assert value is not None

    if not type_.is_array:
        return primitive_value_to_c(type_.type, value)

    c_values = []
    for single_value in value:
        c_value = primitive_value_to_c(type_.type, single_value)
        c_values.append(c_value)
    c_value = '{%s}' % ', '.join(c_values)
    if len(c_values) > 1:
        # Only wrap in a second set of {} if the array length is > 1.
        # This avoids "warning: braces around scalar initializer"
        c_value = '{%s}' % c_value
    return c_value


def primitive_value_to_c(type_, value):
    assert value is not None

    if type_ == 'bool':
        return 'true' if value else 'false'

    if type_ in ['byte', 'char', 'int8', 'int16', 'int32', 'int64']:
        return str(value)

    if type_ in ['uint8', 'uint16', 'uint32', 'uint64']:
        return str(value) + 'u'

    if type_ in ['float32']:
        return '%sf' % value

    if type_ in ['float64']:
        return '%sl' % value

    if type_ == 'string':
        return '"%s"' % escape_string(value)

    assert False, "unknown primitive type '%s'" % type_


def escape_string(s):
    s = s.replace('\\', '\\\\')
    s = s.replace('"', '\\"')
    return s
