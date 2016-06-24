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

from rosidl_cmake import convert_camel_case_to_lower_case_underscore
from rosidl_cmake import expand_template
from rosidl_cmake import get_newest_modification_time
from rosidl_cmake import read_generator_arguments
from rosidl_parser import parse_message_file
from rosidl_parser import parse_service_file


def generate_cpp(generator_arguments_file):
    args = read_generator_arguments(generator_arguments_file)

    template_dir = args['template_dir']
    mapping_msgs = {
        os.path.join(template_dir, 'msg.hpp.em'): '%s.hpp',
        os.path.join(template_dir, 'msg__struct.hpp.em'): '%s__struct.hpp',
        os.path.join(template_dir, 'msg__traits.hpp.em'): '%s__traits.hpp',
    }

    mapping_srvs = {
        os.path.join(template_dir, 'srv.hpp.em'): '%s.hpp',
        os.path.join(template_dir, 'srv__struct.hpp.em'): '%s__struct.hpp',
        os.path.join(template_dir, 'srv__traits.hpp.em'): '%s__traits.hpp',
    }
    for template_file in mapping_msgs.keys():
        assert os.path.exists(template_file), 'Could not find template: ' + template_file
    for template_file in mapping_srvs.keys():
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
                data = {'spec': spec, 'subfolder': subfolder}
                data.update(functions)
                generated_file = os.path.join(
                    args['output_dir'], subfolder, generated_filename %
                    convert_camel_case_to_lower_case_underscore(spec.base_type.type))
                expand_template(
                    template_file, data, generated_file,
                    minimum_timestamp=latest_target_timestamp)

        elif extension == '.srv':
            spec = parse_service_file(args['package_name'], ros_interface_file)
            for template_file, generated_filename in mapping_srvs.items():
                data = {'spec': spec}
                data.update(functions)
                generated_file = os.path.join(
                    args['output_dir'], subfolder, generated_filename %
                    convert_camel_case_to_lower_case_underscore(spec.srv_name))
                expand_template(
                    template_file, data, generated_file,
                    minimum_timestamp=latest_target_timestamp)

    return 0


MSG_TYPE_TO_CPP = {
    'bool': 'bool',
    'byte': 'uint8_t',
    'char': 'char',
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
    'string': 'std::basic_string<char, std::char_traits<char>, ' +
              'typename ContainerAllocator::template rebind<char>::other>',
}


def msg_type_to_cpp(type_):
    """
    Convert a message type into the C++ declaration.

    Example input: uint32, std_msgs/String
    Example output: uint32_t, std_msgs::String_<ContainerAllocator>

    @param type_: The message type
    @type type_: rosidl_parser.Type
    """
    cpp_type = None
    if type_.is_primitive_type():
        cpp_type = MSG_TYPE_TO_CPP[type_.type]
    else:
        cpp_type = '%s::msg::%s_<ContainerAllocator>' % \
            (type_.pkg_name, type_.type)

    if type_.is_array:
        if not type_.array_size:
            return \
                ('std::vector<%s, typename ContainerAllocator::template ' +
                 'rebind<%s>::other>') % (cpp_type, cpp_type)
        elif type_.is_upper_bound:
            return \
                ('rosidl_generator_cpp::BoundedVector<%s, %u, typename ContainerAllocator::' +
                 'template rebind<%s>::other>') % (cpp_type, type_.array_size, cpp_type)
        else:
            return 'std::array<%s, %u>' % (cpp_type, type_.array_size)
    else:
        return cpp_type


def value_to_cpp(type_, value):
    assert type_.is_primitive_type()
    assert value is not None

    if not type_.is_array:
        return primitive_value_to_cpp(type_, value)

    cpp_values = []
    for single_value in value:
        cpp_value = primitive_value_to_cpp(type_, single_value)
        cpp_values.append(cpp_value)
    cpp_value = '{%s}' % ', '.join(cpp_values)
    if len(cpp_values) > 1:
        # Only wrap in a second set of {} if the array length is > 1.
        # This avoids "warning: braces around scalar initializer"
        cpp_value = '{%s}' % cpp_value
    return cpp_value


def primitive_value_to_cpp(type_, value):
    assert type_.is_primitive_type()
    assert value is not None

    if type_.type == 'bool':
        return 'true' if value else 'false'

    if type_.type in [
        'byte',
        'char',
        'int8', 'uint8',
        'int16', 'uint16',
        'int32', 'uint32',
        'int64', 'uint64',
    ]:
        return str(value)

    if type_.type in ['float32']:
        return '%sf' % value

    if type_.type in ['float64']:
        return '%sl' % value

    if type_.type == 'string':
        return '"%s"' % escape_string(value)

    assert False, "unknown primitive type '%s'" % type_.type


def escape_string(s):
    s = s.replace('\\', '\\\\')
    s = s.replace('"', '\\"')
    return s
