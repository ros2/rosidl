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

from rosidl_adapter.msg.parser import parse_message_string
from rosidl_adapter.resource import expand_template


def convert_msg_to_idl(package_dir, package_name, input_file, output_dir):
    assert input_file.suffix == '.msg'

    print('Reading input file: {input_file}'.format_map(locals()))
    content = input_file.read_text(encoding='utf-8')
    msg = parse_message_string(package_name, input_file.stem, content)

    output_file = output_dir / input_file.with_suffix('.idl').name
    print('Writing output file: {output_file}'.format_map(locals()))
    data = {
        'pkg_name': package_name,
        'relative_input_file': input_file.absolute().relative_to(package_dir),
        'msg': msg,
        'get_idl_type': get_idl_type,
        'get_include_file': get_include_file,
        'to_literal': to_literal,
    }

    print('\n'.join([c.name for c in msg.constants]))
    print('\n'.join([f.name for f in msg.fields]))

    expand_template('msg.idl.em', data, output_file)
    return output_file


MSG_TYPE_TO_IDL = {
    'bool': 'boolean',
    'byte': 'octet',
    'char': 'int8',
    'int8': 'int8',
    'uint8': 'uint8',
    'int16': 'int16',
    'uint16': 'uint16',
    'int32': 'int32',
    'uint32': 'uint32',
    'int64': 'int64',
    'uint64': 'uint64',
    'float32': 'float',
    'float64': 'double',
    'string': 'string',
}


def escape_string(string):
    """Escape string according to IDL 4.2 section 7.2.6.2.2 ."""
    estr = string.encode().decode('unicode_escape')
    # Escape quotes too
    estr = estr.replace('"', '\\"')
    estr = estr.replace("'", "\\'")
    estr = estr.replace('?', '\\?')
    return estr


def contains_unicode(string):
    """Return true if string has unicode code points."""
    try:
        string.encode().decode('ascii')
    except UnicodeDecodeError:
        return True
    return False


def to_literal(idl_type, value):
    if 'string' == idl_type:
        return to_string_literal(value)
    elif 'char' == idl_type:
        return to_character_literal(value)
    return value


def to_character_literal(value):
    """Convert string to character literal as described in IDL 4.2 section  7.2.6.2.1 .
    Value is a value from a .msg char.
    The return value text defining a character literal to put into idl."""
    if isinstance(value, int):
        # msg char is [-128, 127]
        # Make negative numbers instead be in the range [128, 255] and convert to char.
        if value < 0:
            value *= -1
            value += 127
        value = chr(value)
    estr = escape_string(value)
    if contains_unicode(value):
        return "L'{0}'".format(estr)
    return "'{0}'".format(estr)


def to_string_literal(string):
    """Convert string to character literal as described in IDL 4.2 section  7.2.6.3 ."""
    estr = escape_string(string)
    if contains_unicode(string):
        return 'L"{0}"'.format(estr)
    return '"{0}"'.format(estr)


def get_include_file(base_type):
    if base_type.is_primitive_type():
        return None
    return '{base_type.pkg_name}/msg/{base_type.type}.idl'.format_map(locals())


def get_idl_type(type_):
    if isinstance(type_, str):
        identifier = MSG_TYPE_TO_IDL[type_]
    elif type_.is_primitive_type():
        identifier = MSG_TYPE_TO_IDL[type_.type]
    else:
        identifier = '{type_.pkg_name}::msg::{type_.type}' \
            .format_map(locals())

    if isinstance(type_, str) or not type_.is_array:
        return identifier

    if type_.is_fixed_size_array():
        return '{identifier}[{type_.array_size}]'.format_map(locals())

    if not type_.is_upper_bound:
        return 'sequence<{identifier}>'.format_map(locals())

    return 'sequence<{identifier}, {type_.array_size}>'.format_map(locals())
