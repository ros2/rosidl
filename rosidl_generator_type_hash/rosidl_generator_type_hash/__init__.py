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
from pathlib import Path
import sys
from typing import List

from rosidl_parser import definition
from rosidl_parser.parser import parse_idl_file


def generate_type_hash(generator_arguments_file: str) -> List[str]:
    with open(generator_arguments_file, 'r') as f:
        args = json.load(f)
    package_name = args['package_name']
    output_dir = Path(args['output_dir'])
    idl_tuples = args['idl_tuples']
    include_paths = args.get('include_paths', [])

    # Lookup for directory containing pregenerated .json.in files
    include_map = {
        package_name: output_dir
    }
    for include_tuple in include_paths:
        include_parts = include_tuple.rsplit(':', 1)
        assert len(include_parts) == 2
        include_package_name, include_base_path = include_parts
        include_map[include_package_name] = Path(include_base_path)

    generated_files = []
    hashers = []

    # First generate all .json.in files (so referenced types can be used in expansion)
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
        generate_to_dir = (output_dir / idl_rel_path).parent
        generate_to_dir.mkdir(parents=True, exist_ok=True)

        hasher = InterfaceHasher.from_idl(idl_file)
        generated_files.extend(
            hasher.write_json_in(output_dir))
        hashers.append(hasher)

    # Expand .json.in and generate .sha256 hash files
    for hasher in hashers:
        generated_files.extend(
            hasher.write_json_out(output_dir, include_map))
        generated_files.extend(
            hasher.write_hash(output_dir))

    return generated_files


# This mapping must match the constants defined in type_description_interfaces/msgs/FieldType.msg
# TODO(emersonknapp)
# There is no FieldType.msg definition for the following rosidl_parser.definition types
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
    # TODO(emersonknapp)
    # there is no rosidl_parser.definition type FixedString
    # FIXED_STRING = 18
    # FIXED_WSTRING = 19
    definition.BoundedString: 20,
    definition.BoundedWString: 21,
}

NESTED_FIELD_TYPE_OFFSETS = {
    definition.Array: 32,
    definition.BoundedSequence: 32 * 2,
    definition.UnboundedSequence: 32 * 3,
}


def serialize_field_type(ftype: definition.AbstractType) -> dict:
    result = {
        'type_id': 0,
        'length': 0,
        'string_length': 0,
        'nested_type_name': '',
    }

    # Determine value type, if this is a nested type
    type_id_offset = 0
    if isinstance(ftype, definition.AbstractNestableType):
        value_type = ftype
    elif isinstance(ftype, definition.AbstractNestedType):
        type_id_offset = NESTED_FIELD_TYPE_OFFSETS[type(ftype)]
        value_type = ftype.value_type
    else:
        raise Exception('Unable to translate field type', ftype)

    # Translate value type to FieldType.msg const value
    if isinstance(value_type, definition.BasicType):
        result['type_id'] = FIELD_TYPES[value_type.typename] + type_id_offset
    elif isinstance(value_type, definition.AbstractGenericString):
        result['type_id'] = FIELD_TYPES[type(value_type)] + type_id_offset
        if value_type.has_maximum_size():
            try:
                result['length'] = value_type.maximum_size
            except AttributeError:
                result['length'] = value_type.size
    elif isinstance(value_type, definition.NamespacedType):
        result['type_id'] = type_id_offset
        result['nested_type_name'] = '/'.join(value_type.namespaced_name())
    elif isinstance(value_type, definition.NamedType):
        result['type_id'] = type_id_offset
        result['nested_type_name'] = value_type.name
    else:
        raise TypeError('Unknown value type ', value_type)

    return result


def serialize_field(member: definition.Member) -> dict:
    return {
        'name': member.name,
        'type': serialize_field_type(member.type),
        # skipping default_value
    }


def serialize_individual_type_description(
    namespaced_type: definition.NamespacedType, members: List[definition.Member]
) -> dict:
    return {
        'type_name': '/'.join(namespaced_type.namespaced_name()),
        'fields': [serialize_field(member) for member in members]
    }


class InterfaceHasher:

    @classmethod
    def from_idl(cls, idl: definition.IdlFile):
        for el in idl.content.elements:
            if isinstance(el, definition.Message):
                return InterfaceHasher(el)
            elif isinstance(el, definition.Service):
                return InterfaceHasher(el)
            elif isinstance(el, definition.Action):
                return InterfaceHasher(el)
        raise Exception('No interface found in IDL')

    def __init__(self, interface):
        self.interface = interface
        self.subinterfaces = {}

        # Determine top level interface, and member fields based on that
        if isinstance(interface, definition.Message):
            self.namespaced_type = interface.structure.namespaced_type
            self.interface_type = 'message'
            self.members = interface.structure.members
        elif isinstance(interface, definition.Service):
            self.namespaced_type = interface.namespaced_type
            self.interface_type = 'service'
            self.subinterfaces = {
                'request_message': InterfaceHasher(interface.request_message),
                'response_message': InterfaceHasher(interface.response_message),
                'event_message': InterfaceHasher(interface.event_message),
            }
            self.members = [
                definition.Member(hasher.namespaced_type, field_name)
                for field_name, hasher in self.subinterfaces.items()
            ]
        elif isinstance(interface, definition.Action):
            self.namespaced_type = interface.namespaced_type
            self.interface_type = 'action'
            self.subinterfaces = {
                'goal': InterfaceHasher(interface.goal),
                'result': InterfaceHasher(interface.result),
                'feedback': InterfaceHasher(interface.feedback),
                'send_goal_service': InterfaceHasher(interface.send_goal_service),
                'get_result_service': InterfaceHasher(interface.get_result_service),
                'feedback_message': InterfaceHasher(interface.feedback_message),
            }
            self.members = [
                definition.Member(hasher.namespaced_type, field_name)
                for field_name, hasher in self.subinterfaces.items()
            ]

        self.individual_type_description = serialize_individual_type_description(
            self.namespaced_type, self.members)

        # Determine needed includes from member fields
        included_types = []
        for member in self.members:
            if isinstance(member.type, definition.NamespacedType):
                included_types.append(member.type)
            elif (
                isinstance(member.type, definition.AbstractNestedType) and
                isinstance(member.type.value_type, definition.NamespacedType)
            ):
                included_types.append(member.type.value_type)

        self.includes = [
            str(Path(*t.namespaced_name()).with_suffix('.json.in'))
            for t in included_types
        ]

        self.rel_path = Path(*self.namespaced_type.namespaced_name()[1:])
        self.include_path = Path(*self.namespaced_type.namespaced_name())

        self.json_in = {
            'type_description': self.individual_type_description,
            'includes': self.includes,
        }

    def write_json_in(self, output_dir) -> List[str]:
        generated_files = []
        for key, val in self.subinterfaces.items():
            generated_files += val.write_json_in(output_dir)

        json_path = output_dir / self.rel_path.with_suffix('.json.in')
        json_path.parent.mkdir(parents=True, exist_ok=True)
        with json_path.open('w', encoding='utf-8') as json_file:
            json_file.write(json.dumps(self.json_in, indent=2))
        return generated_files + [str(json_path)]

    def write_json_out(self, output_dir: Path, includes_map: dict) -> List[str]:
        generated_files = []
        for key, val in self.subinterfaces.items():
            generated_files += val.write_json_out(output_dir, includes_map)

        # Recursively load includes from all included type descriptions
        pending_includes = self.includes[:]
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

        self.json_out = {
            'type_description': self.json_in['type_description'],
            'referenced_type_descriptions': sorted(
                loaded_includes.values(), key=lambda td: td['type_name'])
        }

        json_path = output_dir / self.rel_path.with_suffix('.json')
        with json_path.open('w', encoding='utf-8') as json_file:
            json_file.write(json.dumps(self.json_out))
        return generated_files + [str(json_path)]

    def calculate_hash(self) -> dict:
        json_out_repr = json.dumps(self.json_out)
        sha = hashlib.sha256()
        sha.update(json_out_repr.encode('utf-8'))
        type_hash = sha.hexdigest()

        type_hash_infos = {
            self.interface_type: type_hash,
        }
        for key, val in self.subinterfaces.items():
            type_hash_infos[key] = val.calculate_hash()

        return type_hash_infos

    def write_hash(self, output_dir: Path) -> List[str]:
        type_hash = self.calculate_hash()
        hash_path = output_dir / self.rel_path.with_suffix('.sha256')
        with hash_path.open('w', encoding='utf-8') as hash_file:
            hash_file.write(json.dumps(type_hash, indent=2))
        return [str(hash_path)]
