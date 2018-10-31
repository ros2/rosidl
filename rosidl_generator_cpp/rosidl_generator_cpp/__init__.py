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
from rosidl_parser import parse_action_file
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

    mapping_actions = {
        os.path.join(template_dir, 'action.hpp.em'): '%s.hpp',
        os.path.join(template_dir, 'action__struct.hpp.em'): '%s__struct.hpp',
    }

    for template_file in mapping_msgs.keys():
        assert os.path.exists(template_file), 'Could not find template: ' + template_file
    for template_file in mapping_srvs.keys():
        assert os.path.exists(template_file), 'Could not find template: ' + template_file
    for template_file in mapping_actions.keys():
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
            for template_file, generated_filename in mapping_actions.items():
                data = {'spec': spec, 'subfolder': subfolder}
                data.update(functions)
                generated_file = os.path.join(
                    args['output_dir'], subfolder, generated_filename %
                    convert_camel_case_to_lower_case_underscore(spec.action_name))
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


def msg_type_only_to_cpp(type_):
    """
    Convert a message type into the C++ declaration, ignoring array types.

    Example input: uint32, std_msgs/String
    Example output: uint32_t, std_msgs::String_<ContainerAllocator>

    @param type_: The message type
    @type type_: rosidl_parser.Type
    """
    if type_.is_primitive_type():
        cpp_type = MSG_TYPE_TO_CPP[type_.type]
    else:
        cpp_type = '%s::msg::%s_<ContainerAllocator>' % \
            (type_.pkg_name, type_.type)

    return cpp_type


def msg_type_to_cpp(type_):
    """
    Convert a message type into the C++ declaration, along with the array type.

    Example input: uint32, std_msgs/String, std_msgs/String[3]
    Example output: uint32_t, std_msgs::String_<ContainerAllocator>,
                    std::array<std_msgs::String_<ContainerAllocator>, 3>

    @param type_: The message type
    @type type_: rosidl_parser.Type
    """
    cpp_type = msg_type_only_to_cpp(type_)

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
    """
    Convert a python value into a string representing that value in C++.

    This is equivalent to primitive_value_to_cpp but can process arrays values as well

    Warning this still processes only primitive types
    @param type_: a ROS IDL type
    @type type_: builtin.str
    @param value: the value to convert
    @type value: python builtin (bool, int, float, str or list)
    @returns: a string containing the C++ representation of the value
    """
    assert type_.is_primitive_type(), "Could not convert non-primitive type '%s' to CPP" % (type_)
    assert value is not None, "Value for type '%s' must not be None" % (type_)

    if not type_.is_array:
        return primitive_value_to_cpp(type_, value)

    cpp_values = []
    is_string_array = type_.__str__().startswith('string')
    for single_value in value:
        cpp_value = primitive_value_to_cpp(type_, single_value)
        if is_string_array:
            tmp_cpp_value = '{%s}' % cpp_value
        else:
            tmp_cpp_value = cpp_value
        cpp_values.append(tmp_cpp_value)
    cpp_value = '{%s}' % ', '.join(cpp_values)
    if len(cpp_values) > 1 and not is_string_array:
        # Only wrap in a second set of {} if the array length is > 1.
        # This avoids "warning: braces around scalar initializer"
        cpp_value = '{%s}' % cpp_value
    return cpp_value


def primitive_value_to_cpp(type_, value):
    """
    Convert a python value into a string representing that value in C++.

    Warning: The value has to be a primitive and not a list
      (aka this function doesn't work for arrays)
    @param type_: a ROS IDL type
    @type type_: builtin.str
    @param value: the value to convert
    @type value: python builtin (bool, int, float or str)
    @returns: a string containing the C++ representation of the value
    """
    assert type_.is_primitive_type(), "Could not convert non-primitive type '%s' to CPP" % (type_)
    assert value is not None, "Value for type '%s' must not be None" % (type_)

    if type_.type == 'bool':
        return 'true' if value else 'false'

    if type_.type in [
        'byte',
        'char',
        'int8', 'uint8',
        'int16', 'uint16',
        'float64'
    ]:
        return str(value)

    if type_.type == 'int32':
        return '%sl' % value

    if type_.type == 'uint32':
        return '%sul' % value

    if type_.type == 'int64':
        return '%sll' % value

    if type_.type == 'uint64':
        return '%sull' % value

    if type_.type == 'float32':
        return '%sf' % value

    if type_.type == 'string':
        return '"%s"' % escape_string(value)

    assert False, "unknown primitive type '%s'" % type_.type


def default_value_from_type(type_):
    if type_ == 'string':
        return ''
    elif type_ in ['float32', 'float64']:
        return 0.0
    elif type_ == 'bool':
        return False
    return 0


def escape_string(s):
    s = s.replace('\\', '\\\\')
    s = s.replace('"', '\\"')
    return s


def create_init_alloc_and_member_lists(spec):
    # A Member object represents the information we need to know to initialize
    # a single member of the class.
    class Member:

        def __init__(self, name):
            self.name = name
            self.default_value = None
            self.zero_value = None
            self.zero_need_array_override = False
            self.type = None
            self.num_prealloc = 0

        def same_default_and_zero_value(self, other):
            return self.default_value == other.default_value and \
                self.zero_value == other.zero_value

    # A CommonMemberSet is a set of adjacent members that share the same set of
    # initialization semantics.  Here, initialization semantics mean that all
    # members of the set have a default value (or do not have a default value),
    # and all members of the set have a zero value (or do not have a zero
    # value).
    class CommonMemberSet:

        def __init__(self):
            self.members = []

        def add_member(self, member):
            if not self.members or self.members[-1].same_default_and_zero_value(member):
                self.members.append(member)
                return True
            return False

    # The loop below is used to generate three different lists:
    #   init_list - The list of member variables that we will initialize using member
    #               initialization in the default constructor
    #   alloc_list - The list of member variables that we will initialize using member
    #                initializion in the allocator constructor
    #   member_list - The list of members that we will generate initialization code
    #                 for in the body of the constructors
    init_list = []
    alloc_list = []
    member_list = []
    for field in spec.fields:
        member = Member(field.name)
        member.type = field.type
        if field.type.is_array:
            if field.type.is_fixed_size_array():
                alloc_list.append(field.name + '(_alloc)')
                if field.type.is_primitive_type():
                    default = default_value_from_type(field.type.type)
                    single = primitive_value_to_cpp(field.type, default)
                    member.zero_value = [single] * field.type.array_size
                    if field.default_value is not None:
                        member.default_value = []
                        for val in field.default_value:
                            member.default_value.append(primitive_value_to_cpp(field.type, val))
                else:
                    member.zero_value = []
                    member.zero_need_array_override = True
            else:
                if field.default_value is not None:
                    member.default_value = value_to_cpp(field.type, field.default_value)
                    member.num_prealloc = len(field.default_value)
        else:
            if field.type.is_primitive_type():
                if field.type.type == 'string':
                    alloc_list.append(field.name + '(_alloc)')
                default = default_value_from_type(field.type.type)
                member.zero_value = primitive_value_to_cpp(field.type, default)
                if field.default_value is not None:
                    member.default_value = primitive_value_to_cpp(field.type, field.default_value)
            else:
                init_list.append(field.name + '(_init)')
                alloc_list.append(field.name + '(_alloc, _init)')

        if member.default_value is not None or member.zero_value is not None:
            if not member_list or not member_list[-1].add_member(member):
                commonset = CommonMemberSet()
                commonset.add_member(member)
                member_list.append(commonset)

    return init_list, alloc_list, member_list
