# Copyright 2023 Open Source Robotics Foundation, Inc.
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

import hashlib
import json

from rosidl_parser import definition

# This mapping must match the constants defined in type_description_interfaces/msgs/FieldType.msg
# TODO There is no FieldType.msg definition for the following rosidl_parser.definition types
# * SIGNED_NONEXPLICIT_INTEGER_TYPES = short, long, long long
# * UNSIGNED_NONEXPLICIT_INTEGER_TYPES = unsigned short, unsigned long, unsigned long long
# *
FIELD_TYPES = {
    'nested_type': 0,
    'int8': 1,
    'uint8': 2,
    'int16': 3,
    'uint16': 4,
    'int32': 5,
    'uint32': 6,
    'int64': 7,
    'uint64': 8,
    'float': 9,
    'double': 10,
    'long double': 11,
    'char': 12,
    'wchar': 13,
    'boolean': 14,
    'octet': 15,  # byte
    definition.UnboundedString: 16,
    definition.UnboundedWString: 17,
    # TODO there is no rosidl_parser.definition type for fixed strings (there is array of char, though?)
    # FIXED_STRING = 18
    # FIXED_WSTRING = 19
    definition.BoundedString: 20,
    definition.BoundedWString: 21,
}

NESTED_FIELD_TYPE_OFFSETS = {
    definition.Array: 32,
    definition.BoundedSequence: 64,
    definition.UnboundedSequence: 96,
}

def translate_type_id(value_type, offset, result):
    if isinstance(value_type, definition.BasicType):
        result['type_id'] = FIELD_TYPES[value_type.typename] + offset
    elif isinstance(value_type, definition.AbstractGenericString):
        result['type_id'] = FIELD_TYPES[type(value_type)] + offset
        if value_type.has_maximum_size():
            result['string_length'] = value_type.maximum_size
    elif isinstance(value_type, definition.NamespacedType):
        result['type_id'] = offset
        result['nested_type_name'] = '/'.join(value_type.namespaced_name())
    elif isinstance(value_type, definition.NamedType):
        result['type_id'] = offset
        result['nested_type_name'] = value_type.name
    else:
        raise TypeError('Unknown value type ', value_type)


def serialize_field_type(ftype: definition.AbstractType):
    result = {
        'type_id': 0,
        'length': 0,
        'string_length': 0,
        'nested_type_name': '',
    }

    if isinstance(ftype, definition.AbstractNestableType):
        translate_type_id(ftype, 0, result)
    elif isinstance(ftype, definition.AbstractNestedType):
        type_id_offset = NESTED_FIELD_TYPE_OFFSETS[type(ftype)]
        translate_type_id(ftype.value_type, type_id_offset, result)
        if ftype.has_maximum_size():
            try:
                result['length'] = ftype.maximum_size
            except AttributeError:
                result['length'] = ftype.size
    else:
        print(ftype)
        raise Exception('Unable to translate field type', ftype)

    return result


def serialize_field(member: definition.Member):
    return {
        'name': member.name,
        'type': serialize_field_type(member.type),
        # skipping default_value
    }


def serialize_individual_type_description(msg: definition.Message):
    fields = [serialize_field(member) for member in msg.structure.members]
    # referenced_types = [f['type']['nested_type_name'] for f in fields]
    # referenced_types = [f for f in referenced_types if f != '']
    return {
        'type_name': '/'.join(msg.structure.namespaced_type.namespaced_name()),
        'fields': fields
    }


def generate_type_version_hash(file_key, idl_files):
    idl = idl_files[file_key]

    includes = []
    referenced_type_descriptions = {}
    serialization_data = {
        'type_description': None,
        'referenced_type_descriptions': [],
    }

    for el in idl.content.elements:
        if isinstance(el, definition.Include):
            includes.append(el.locator)
            # print(f'  Include: {el.locator}')
        elif isinstance(el, definition.Message):
            # print(f'  Message: {el.structure.namespaced_type.namespaces} / {el.structure.namespaced_type.name}')
            serialization_data['type_description'] = serialize_individual_type_description(el)
        elif isinstance(el, definition.Service):
            serialization_data['type_description'] = {
                'request_message': serialize_individual_type_description(el.request_message),
                'response_message': serialize_individual_type_description(el.response_message),
            }
            # print(f'  Service: {el.namespaced_type.name}')
            pass
        elif isinstance(el, definition.Action):
            # print(f'  Action: {el}')
            pass
        else:
            raise Exception(f'Do not know how to hash {el}')

    while includes:
        locator = includes.pop()
        if locator not in referenced_type_descriptions:
            included_file = idl_files[locator]
            for el in included_file.content.elements:
                if isinstance(el, definition.Include):
                    includes.append(el.locator)
                    # print(f'  Include: {el.locator}')
                elif isinstance(el, definition.Message):
                    referenced_type_descriptions[locator] = serialize_individual_type_description(el)

    referenced_type_descriptions
    serialization_data['referenced_type_descriptions'] = sorted(
        referenced_type_descriptions.values(), key=lambda td: td['type_name'])
    serialized_type_description = json.dumps(serialization_data)
    # print(serialized_type_description)
    m = hashlib.sha256()
    m.update(serialized_type_description.encode('utf-8'))
    return m.digest()
