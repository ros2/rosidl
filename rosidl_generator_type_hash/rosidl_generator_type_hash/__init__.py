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

import hashlib
import json
from pathlib import Path
import re
import sys

from rosidl_parser import definition
from rosidl_parser.parser import parse_idl_file


def convert_camel_case_to_lower_case_underscore(value):
    # insert an underscore before any upper case letter
    # which is followed by a lower case letter
    value = re.sub('(.)([A-Z][a-z]+)', r'\1_\2', value)
    # insert an underscore before any upper case letter
    # which is preseded by a lower case letter or number
    value = re.sub('([a-z0-9])([A-Z])', r'\1_\2', value)
    return value.lower()


def generate_type_hash(generator_arguments_file):
    with open(generator_arguments_file, mode='r', encoding='utf-8') as h:
        args = json.load(h)
    print(args)

    idl_files = {}
    idl_files_to_generate = []
    package_name = args['package_name']
    output_dir = args['output_dir']

    for idl_tuple in args.get('idl_tuples', []):
        idl_parts = idl_tuple.rsplit(':', 1)
        assert len(idl_parts) == 2
        namespaced_idl_path = str(Path(package_name) / idl_parts[1])
        locator = definition.IdlLocator(*idl_parts)
        try:
            idl_files[namespaced_idl_path] = parse_idl_file(locator)
            idl_files_to_generate.append(namespaced_idl_path)
        except Exception as e:
            print('Error  processing idl file: ' +
                  str(locator.get_absolute_path()), file=sys.stderr)
            raise(e)

    generated_files = []
    for file_key in idl_files_to_generate:
        idl_rel_path = Path(file_key)
        idl_rel_path = idl_rel_path.relative_to(idl_rel_path.parts[0])
        idl_stem = idl_rel_path.stem
        idl_stem = convert_camel_case_to_lower_case_underscore(idl_stem)
        json_individual_repr = json.dumps(serialize_individual_idl(idl_files[file_key]))
        # json_repr = idl_to_hashable_json(file_key, idl_files)

        generate_to_dir = Path(output_dir) / idl_rel_path.parent
        generate_to_dir.mkdir(parents=True, exist_ok=True)

        json_path = generate_to_dir / f'{idl_stem}.individual.json'
        with json_path.open('w', encoding='utf-8') as json_file:
            json_file.write(json_individual_repr)
        generated_files.append(str(json_path))

        json_nested_repr = json.dumps(serialize_nested_idl(file_key, idl_files))
        json_path = generate_to_dir / f'{idl_stem}.json'
        with json_path.open('w', encoding='utf-8') as json_file:
            json_file.write(json_nested_repr)
        generated_files.append(str(json_path))

        sha = hashlib.sha256()
        sha.update(json_nested_repr.encode('utf-8'))
        type_hash = sha.hexdigest()
        hash_path = generate_to_dir / f'{idl_stem}.sha256'
        with hash_path.open('w', encoding='utf-8') as hash_file:
            hash_file.write(type_hash)
        generated_files.append(str(hash_path))

    return generated_files


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
        raise Exception('Unable to translate field type', ftype)

    return result


def serialize_field(member: definition.Member):
    return {
        'name': member.name,
        'type': serialize_field_type(member.type),
        # skipping default_value
    }


def serialize_individual_type_description(msg: definition.Message):
    return {
        'type_name': '/'.join(msg.structure.namespaced_type.namespaced_name()),
        'fields': [serialize_field(member) for member in msg.structure.members]
    }


def serialize_individual_idl(idl: definition.IdlFile):
    for el in idl.content.elements:
        if isinstance(el, definition.Message):
            return serialize_individual_type_description(el)
        elif isinstance(el, definition.Service):
            return {
                'request': serialize_individual_type_description(el.request_message),
                'response': serialize_individual_type_description(el.response_message),
            }
        elif isinstance(el, definition.Action):
            # TODO(emersonknapp)
            pass
    raise Exception("Didn't find something to serialize...")


def serialize_nested_idl(file_key, idl_files):
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
        elif isinstance(el, definition.Message):
            serialization_data['type_description'] = serialize_individual_type_description(el)
        elif isinstance(el, definition.Service):
            serialization_data['type_description'] = {
                'request_message': serialize_individual_type_description(el.request_message),
                'response_message': serialize_individual_type_description(el.response_message),
            }
            pass
        elif isinstance(el, definition.Action):
            # TODO
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
                elif isinstance(el, definition.Message):
                    referenced_type_descriptions[locator] = serialize_individual_type_description(el)

    referenced_type_descriptions
    serialization_data['referenced_type_descriptions'] = sorted(
        referenced_type_descriptions.values(), key=lambda td: td['type_name'])
    return serialization_data


def generate_type_version_hash(file_key, idl_files):
    serialized_type_description = idl_to_hashable_json(file_key, idl_files)
    # print(json.dumps(serialization_data, indent=2))
    m = hashlib.sha256()
    m.update(serialized_type_description.encode('utf-8'))
    return m.digest()
