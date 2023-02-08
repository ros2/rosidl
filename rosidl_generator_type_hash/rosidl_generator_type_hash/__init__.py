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
import sys
from typing import List

from rosidl_parser import definition
from rosidl_parser.parser import parse_idl_file


def generate_type_hash(generator_arguments_file: str):
    with open(generator_arguments_file, 'r') as f:
        args = json.load(f)
    package_name = args['package_name']
    output_dir = args['output_dir']
    idl_tuples = args['idl_tuples']
    include_paths = args.get('include_paths', [])

    include_map = {
        package_name: Path(output_dir)
    }
    for include_tuple in include_paths:
        include_parts = include_tuple.rsplit(':', 1)
        assert len(include_parts) == 2
        include_package_name, include_base_path = include_parts
        include_map[include_package_name] = Path(include_base_path)

    generated_files = []
    json_contents = []
    for idl_tuple in idl_tuples:
        idl_parts = idl_tuple.rsplit(':', 1)
        assert len(idl_parts) == 2
        locator = definition.IdlLocator(*idl_parts)
        try:
            idl_file = parse_idl_file(locator)
        except Exception as e:
            print('Error processing idl file: ' +
                  str(locator.get_absolute_path()), file=sys.stderr)
            raise(e)

        idl_rel_path = Path(idl_parts[1])
        idl_stem = idl_rel_path.stem
        generate_to_dir = (Path(output_dir) / idl_rel_path).parent
        generate_to_dir.mkdir(parents=True, exist_ok=True)
        stem_path = generate_to_dir / f'{idl_stem}'

        json_in = generate_json_in(idl_file)
        # Need to generate all .json.in before expanding .json final
        json_contents.append((stem_path, json_in))

        json_path = stem_path.with_suffix('.json.in')
        with json_path.open('w', encoding='utf-8') as json_file:
            json_file.write(json.dumps(json_in, indent=2))
            generated_files.append(str(json_path))

    for stem_path, json_in in json_contents:
        json_out = generate_json_out(json_in, include_map)
        json_out_repr = json.dumps(json_out)

        json_path = stem_path.with_suffix('.json')
        with json_path.open('w', encoding='utf-8') as json_file:
            json_file.write(json_out_repr)
            generated_files.append(str(json_path))

        sha = hashlib.sha256()
        sha.update(json_out_repr.encode('utf-8'))
        type_hash = sha.hexdigest()
        hash_path = stem_path.with_suffix('.sha256')
        with hash_path.open('w', encoding='utf-8') as hash_file:
            hash_file.write(type_hash)
        generated_files.append(str(hash_path))

    return generated_files


# This mapping must match the constants defined in type_description_interfaces/msgs/FieldType.msg
# TODO There is no FieldType.msg definition for the following rosidl_parser.definition types
# * SIGNED_NONEXPLICIT_INTEGER_TYPES = short, long, long long
# * UNSIGNED_NONEXPLICIT_INTEGER_TYPES = unsigned short, unsigned long, unsigned long long
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
    # TODO there is no rosidl_parser.definition type for fixed strings (caveat: array of char)
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


def generate_json_in(idl: definition.IdlFile):
    type_description = None
    includes = []
    # TODO(emersonknapp): do the individual message types get their own type hash?
    for el in idl.content.elements:
        if isinstance(el, definition.Include):
            includes.append(el.locator)
        elif isinstance(el, definition.Message):
            type_description = serialize_individual_type_description(el)
        elif isinstance(el, definition.Service):
            type_description = {
                'request_message': serialize_individual_type_description(el.request_message),
                'response_message': serialize_individual_type_description(el.response_message),
            }
        elif isinstance(el, definition.Action):
            type_description = {
                'send_goal_service': {
                    'request_message': serialize_individual_type_description(
                        el.send_goal_service.request_message),
                    'response_message': serialize_individual_type_description(
                        el.send_goal_service.response_message),
                },
                'get_result_service': {
                    'request_message': serialize_individual_type_description(
                        el.get_result_service.request_message),
                    'response_message': serialize_individual_type_description(
                        el.get_result_service.response_message),
                },
                'feedback_message': serialize_individual_type_description(el.feedback_message),
            }
        else:
            raise Exception(f'Do not know how to hash {el}')
    if type_description is None:
        raise Exception('Did not find an interface to serialize in IDL file')

    includes = [
        str(Path(include).with_suffix('.json.in')) for include in includes
    ]
    return {
        'type_description': type_description,
        'includes': includes,
    }


def generate_json_out(json_in, includes_map):
    pending_includes = json_in['includes']
    loaded_includes = {}
    while pending_includes:
        process_include = pending_includes.pop()
        if process_include in loaded_includes:
            continue
        p_path = Path(process_include)
        assert(not p_path.is_absolute())
        include_package = p_path.parts[0]
        include_file = includes_map[include_package] / p_path.relative_to(include_package)

        with include_file.open('r') as include_file:
            include_json = json.load(include_file)

        loaded_includes[process_include] = include_json['type_description']
        pending_includes.extend(include_json['includes'])

    return {
        'type_description': json_in['type_description'],
        'referenced_type_descriptions': sorted(
            loaded_includes.values(), key=lambda td: td['type_name'])
    }
