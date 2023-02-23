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

from copy import deepcopy
import hashlib
import json
from pathlib import Path
import re
import sys
from typing import List, Tuple

from rosidl_parser import definition
from rosidl_parser.parser import parse_idl_file

# RIHS: ROS Interface Hashing Standard, per REP-2011
# NOTE: These values and implementations must be updated if
# - type_description_interfaces messsages change, or
# - the hashing algorithm for type descriptions changes
# Both changes require an increment of the RIHS version
RIHS01_PREFIX = 'RIHS01_'
RIHS01_HASH_VALUE_SIZE = 32
RIHS01_PATTERN = re.compile(r'RIHS([0-9a-f]{2})_([0-9a-f]{64})')

# Used by code generators to create type hash variable names
TYPE_HASH_VAR = 'TYPE_HASH'


def generate_type_hash(generator_arguments_file: str) -> List[str]:
    with open(generator_arguments_file, 'r') as f:
        args = json.load(f)
    package_name = args['package_name']
    output_dir = Path(args['output_dir'])
    idl_tuples = args['idl_tuples']
    include_paths = args.get('include_paths', [])

    # Lookup for directory containing dependency .json files
    include_map = {
        package_name: output_dir
    }
    for include_tuple in include_paths:
        include_parts = include_tuple.split(':', 1)
        assert len(include_parts) == 2
        include_package_name, include_base_path = include_parts
        include_map[include_package_name] = Path(include_base_path)

    generated_files = []
    hashers = {}

    # Initialize all local types first so they can be referenced by other local types
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
        hashers[hasher.namespaced_type.namespaced_name()] = hasher

    # Generate output files
    for hasher in hashers.values():
        generated_files += hasher.write_unified_json(output_dir, hashers, include_map)

    return generated_files


def parse_rihs_string(rihs_str: str) -> Tuple[int, str]:
    """Parse RIHS string, return (version, value) tuple."""
    match = RIHS01_PATTERN.match(rihs_str)
    if not match:
        raise ValueError(f'Type hash string {rihs_str} does not match expected RIHS format.')
    version, value = match.group(1, 2)
    return (int(version), value)


# This mapping must match the constants defined in type_description_interfaces/msgs/FieldType.msg
# NOTE: Nonexplicit integer types are not defined in FieldType (short, long, long long).
# If a ROS IDL uses these, this generator will throw a KeyError.
FIELD_VALUE_TYPE_NAMES = {
    None: 'FIELD_TYPE_NOT_SET',
    'nested_type': 'FIELD_TYPE_NESTED_TYPE',
    'int8': 'FIELD_TYPE_INT8',
    'uint8': 'FIELD_TYPE_UINT8',
    'int16': 'FIELD_TYPE_INT16',
    'uint16': 'FIELD_TYPE_UINT16',
    'int32': 'FIELD_TYPE_INT32',
    'uint32': 'FIELD_TYPE_UINT32',
    'int64': 'FIELD_TYPE_INT64',
    'uint64': 'FIELD_TYPE_UINT64',
    'float': 'FIELD_TYPE_FLOAT',
    'double': 'FIELD_TYPE_DOUBLE',
    'long': 'LONG_DOUBLE',
    'char': 'FIELD_TYPE_CHAR',
    'wchar': 'FIELD_TYPE_WCHAR',
    'boolean': 'FIELD_TYPE_BOOLEAN',
    'octet': 'FIELD_TYPE_BYTE',
    definition.UnboundedString: 'FIELD_TYPE_STRING',
    definition.UnboundedWString: 'FIELD_TYPE_WSTRING',
    # NOTE: rosidl_parser does not define fixed string types
    definition.BoundedString: 'FIELD_TYPE_BOUNDED_STRING',
    definition.BoundedWString: 'FIELD_TYPE_BOUNDED_WSTRING',
}

NESTED_FIELD_TYPE_SUFFIXES = {
    definition.Array: '_ARRAY',
    definition.BoundedSequence: '_BOUNDED_SEQUENCE',
    definition.UnboundedSequence: '_UNBOUNDED_SEQUENCE',
}

# Copied directly from FieldType.msg, with simple string manipulation to create a dict
FIELD_TYPE_IDS = {
    'FIELD_TYPE_NOT_SET': 0,

    # Nested type defined in other .msg/.idl files.
    'FIELD_TYPE_NESTED_TYPE': 1,

    # Basic Types
    # Integer Types
    'FIELD_TYPE_INT8': 2,
    'FIELD_TYPE_UINT8': 3,
    'FIELD_TYPE_INT16': 4,
    'FIELD_TYPE_UINT16': 5,
    'FIELD_TYPE_INT32': 6,
    'FIELD_TYPE_UINT32': 7,
    'FIELD_TYPE_INT64': 8,
    'FIELD_TYPE_UINT64': 9,

    # Floating-Point Types
    'FIELD_TYPE_FLOAT': 10,
    'FIELD_TYPE_DOUBLE': 11,
    'FIELD_TYPE_LONG_DOUBLE': 12,

    # Char and WChar Types
    'FIELD_TYPE_CHAR': 13,
    'FIELD_TYPE_WCHAR': 14,

    # Boolean Type
    'FIELD_TYPE_BOOLEAN': 15,

    # Byte/Octet Type
    'FIELD_TYPE_BYTE': 16,

    # String Types
    'FIELD_TYPE_STRING': 17,
    'FIELD_TYPE_WSTRING': 18,

    # Fixed String Types
    'FIELD_TYPE_FIXED_STRING': 19,
    'FIELD_TYPE_FIXED_WSTRING': 20,

    # Bounded String Types
    'FIELD_TYPE_BOUNDED_STRING': 21,
    'FIELD_TYPE_BOUNDED_WSTRING': 22,

    # Fixed Sized Array Types
    'FIELD_TYPE_NESTED_TYPE_ARRAY': 49,
    'FIELD_TYPE_INT8_ARRAY': 50,
    'FIELD_TYPE_UINT8_ARRAY': 51,
    'FIELD_TYPE_INT16_ARRAY': 52,
    'FIELD_TYPE_UINT16_ARRAY': 53,
    'FIELD_TYPE_INT32_ARRAY': 54,
    'FIELD_TYPE_UINT32_ARRAY': 55,
    'FIELD_TYPE_INT64_ARRAY': 56,
    'FIELD_TYPE_UINT64_ARRAY': 57,
    'FIELD_TYPE_FLOAT_ARRAY': 58,
    'FIELD_TYPE_DOUBLE_ARRAY': 59,
    'FIELD_TYPE_LONG_DOUBLE_ARRAY': 60,
    'FIELD_TYPE_CHAR_ARRAY': 61,
    'FIELD_TYPE_WCHAR_ARRAY': 62,
    'FIELD_TYPE_BOOLEAN_ARRAY': 63,
    'FIELD_TYPE_BYTE_ARRAY': 64,
    'FIELD_TYPE_STRING_ARRAY': 65,
    'FIELD_TYPE_WSTRING_ARRAY': 66,
    'FIELD_TYPE_FIXED_STRING_ARRAY': 67,
    'FIELD_TYPE_FIXED_WSTRING_ARRAY': 68,
    'FIELD_TYPE_BOUNDED_STRING_ARRAY': 69,
    'FIELD_TYPE_BOUNDED_WSTRING_ARRAY': 70,

    # Bounded Sequence Types
    'FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE': 97,
    'FIELD_TYPE_INT8_BOUNDED_SEQUENCE': 98,
    'FIELD_TYPE_UINT8_BOUNDED_SEQUENCE': 99,
    'FIELD_TYPE_INT16_BOUNDED_SEQUENCE': 100,
    'FIELD_TYPE_UINT16_BOUNDED_SEQUENCE': 101,
    'FIELD_TYPE_INT32_BOUNDED_SEQUENCE': 102,
    'FIELD_TYPE_UINT32_BOUNDED_SEQUENCE': 103,
    'FIELD_TYPE_INT64_BOUNDED_SEQUENCE': 104,
    'FIELD_TYPE_UINT64_BOUNDED_SEQUENCE': 105,
    'FIELD_TYPE_FLOAT_BOUNDED_SEQUENCE': 106,
    'FIELD_TYPE_DOUBLE_BOUNDED_SEQUENCE': 107,
    'FIELD_TYPE_LONG_DOUBLE_BOUNDED_SEQUENCE': 108,
    'FIELD_TYPE_CHAR_BOUNDED_SEQUENCE': 109,
    'FIELD_TYPE_WCHAR_BOUNDED_SEQUENCE': 110,
    'FIELD_TYPE_BOOLEAN_BOUNDED_SEQUENCE': 111,
    'FIELD_TYPE_BYTE_BOUNDED_SEQUENCE': 112,
    'FIELD_TYPE_STRING_BOUNDED_SEQUENCE': 113,
    'FIELD_TYPE_WSTRING_BOUNDED_SEQUENCE': 114,
    'FIELD_TYPE_FIXED_STRING_BOUNDED_SEQUENCE': 115,
    'FIELD_TYPE_FIXED_WSTRING_BOUNDED_SEQUENCE': 116,
    'FIELD_TYPE_BOUNDED_STRING_BOUNDED_SEQUENCE': 117,
    'FIELD_TYPE_BOUNDED_WSTRING_BOUNDED_SEQUENCE': 118,

    # Unbounded Sequence Types
    'FIELD_TYPE_NESTED_TYPE_UNBOUNDED_SEQUENCE': 145,
    'FIELD_TYPE_INT8_UNBOUNDED_SEQUENCE': 146,
    'FIELD_TYPE_UINT8_UNBOUNDED_SEQUENCE': 147,
    'FIELD_TYPE_INT16_UNBOUNDED_SEQUENCE': 148,
    'FIELD_TYPE_UINT16_UNBOUNDED_SEQUENCE': 149,
    'FIELD_TYPE_INT32_UNBOUNDED_SEQUENCE': 150,
    'FIELD_TYPE_UINT32_UNBOUNDED_SEQUENCE': 151,
    'FIELD_TYPE_INT64_UNBOUNDED_SEQUENCE': 152,
    'FIELD_TYPE_UINT64_UNBOUNDED_SEQUENCE': 153,
    'FIELD_TYPE_FLOAT_UNBOUNDED_SEQUENCE': 154,
    'FIELD_TYPE_DOUBLE_UNBOUNDED_SEQUENCE': 155,
    'FIELD_TYPE_LONG_DOUBLE_UNBOUNDED_SEQUENCE': 156,
    'FIELD_TYPE_CHAR_UNBOUNDED_SEQUENCE': 157,
    'FIELD_TYPE_WCHAR_UNBOUNDED_SEQUENCE': 158,
    'FIELD_TYPE_BOOLEAN_UNBOUNDED_SEQUENCE': 159,
    'FIELD_TYPE_BYTE_UNBOUNDED_SEQUENCE': 160,
    'FIELD_TYPE_STRING_UNBOUNDED_SEQUENCE': 161,
    'FIELD_TYPE_WSTRING_UNBOUNDED_SEQUENCE': 162,
    'FIELD_TYPE_FIXED_STRING_UNBOUNDED_SEQUENCE': 163,
    'FIELD_TYPE_FIXED_WSTRING_UNBOUNDED_SEQUENCE': 164,
    'FIELD_TYPE_BOUNDED_STRING_UNBOUNDED_SEQUENCE': 165,
    'FIELD_TYPE_BOUNDED_WSTRING_UNBOUNDED_SEQUENCE': 166,
}


def field_type_type_name(ftype: definition.AbstractType) -> str:
    value_type = ftype
    name_suffix = ''

    if isinstance(ftype, definition.AbstractNestedType):
        value_type = ftype.value_type
        name_suffix = NESTED_FIELD_TYPE_SUFFIXES[type(ftype)]

    if isinstance(value_type, definition.BasicType):
        value_type_name = FIELD_VALUE_TYPE_NAMES[value_type.typename]
    elif isinstance(value_type, definition.AbstractGenericString):
        value_type_name = FIELD_VALUE_TYPE_NAMES[type(value_type)]
    elif (
        isinstance(value_type, definition.NamespacedType) or
        isinstance(value_type, definition.NamedType)
    ):
        value_type_name = 'FIELD_TYPE_NESTED_TYPE'
    else:
        raise ValueError(f'Unknown value type {value_type}')

    return value_type_name + name_suffix


def field_type_type_id(ftype: definition.AbstractType) -> Tuple[str, int]:
    return FIELD_TYPE_IDS[field_type_type_name(ftype)]


def field_type_capacity(ftype: definition.AbstractType) -> int:
    if isinstance(ftype, definition.AbstractNestedType):
        if ftype.has_maximum_size():
            try:
                return ftype.maximum_size
            except AttributeError:
                return ftype.size
    return 0


def field_type_string_capacity(ftype: definition.AbstractType) -> int:
    value_type = ftype
    if isinstance(ftype, definition.AbstractNestedType):
        value_type = ftype.value_type

    if isinstance(value_type, definition.AbstractGenericString):
        if value_type.has_maximum_size():
            try:
                return value_type.maximum_size
            except AttributeError:
                return value_type.size
    return 0


def field_type_nested_type_name(ftype: definition.AbstractType, joiner='/') -> str:
    value_type = ftype
    if isinstance(ftype, definition.AbstractNestedType):
        value_type = ftype.value_type
    if isinstance(value_type, definition.NamespacedType):
        return joiner.join(value_type.namespaced_name())
    elif isinstance(value_type, definition.NamedType):
        return value_type.name
    return ''


def serialize_field_type(ftype: definition.AbstractType) -> dict:
    return {
        'type_id': field_type_type_id(ftype),
        'capacity': field_type_capacity(ftype),
        'string_capacity': field_type_string_capacity(ftype),
        'nested_type_name': field_type_nested_type_name(ftype),
    }


def serialize_field(member: definition.Member) -> dict:
    return {
        'name': member.name,
        'type': serialize_field_type(member.type),
        'default_value':
            str(member.get_annotation_value('default')['value'])
            if member.has_annotation('default') else ''
    }


def serialize_individual_type_description(
    namespaced_type: definition.NamespacedType, members: List[definition.Member]
) -> dict:
    return {
        'type_name': '/'.join(namespaced_type.namespaced_name()),
        'fields': [serialize_field(member) for member in members]
    }


class InterfaceHasher:
    """Contains context about subinterfaces for a given interface description."""

    @classmethod
    def from_idl(cls, idl: definition.IdlFile):
        for el in idl.content.elements:
            if any(isinstance(el, type_) for type_ in [
                definition.Message, definition.Service, definition.Action
            ]):
                return InterfaceHasher(el)
        raise ValueError('No interface found in IDL')

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
        self.includes = []
        for member in self.members:
            if isinstance(member.type, definition.NamespacedType):
                self.includes.append(member.type.namespaced_name())
            elif (
                isinstance(member.type, definition.AbstractNestedType) and
                isinstance(member.type.value_type, definition.NamespacedType)
            ):
                self.includes.append(member.type.value_type.namespaced_name())

        self.rel_path = Path(*self.namespaced_type.namespaced_name()[1:])
        self.include_path = Path(*self.namespaced_type.namespaced_name())

    def write_unified_json(
        self, output_dir: Path, local_hashers: dict, includes_map: dict
    ) -> List[Path]:
        generated_files = []
        referenced_types = {}

        for key, val in self.subinterfaces.items():
            generated_files += val.write_unified_json(output_dir, local_hashers, includes_map)

        def add_referenced_type(individual_type_description):
            type_name = individual_type_description['type_name']
            if (
                type_name in referenced_types and
                referenced_types[type_name] != individual_type_description
            ):
                raise Exception('Encountered two definitions of the same referenced type')
            referenced_types[type_name] = individual_type_description

        process_includes = self.includes[:]
        while process_includes:
            process_type = process_includes.pop()

            # A type in this package may refer to types, and hasn't been unrolled yet,
            # so process its includes breadth first
            if process_type in local_hashers:
                add_referenced_type(local_hashers[process_type].individual_type_description)
                process_includes += local_hashers[process_type].includes
                continue

            # All nonlocal descriptions will have all recursively referenced types baked in
            p_path = Path(*process_type).with_suffix('.json')
            pkg = p_path.parts[0]
            pkg_dir = includes_map[pkg]
            include_path = pkg_dir / p_path.relative_to(pkg)
            with include_path.open('r') as include_file:
                include_json = json.load(include_file)

            type_description_msg = include_json['type_description_msg']
            add_referenced_type(type_description_msg['type_description'])
            for rt in type_description_msg['referenced_type_descriptions']:
                add_referenced_type(rt)

        self.full_type_description = {
            'type_description': self.individual_type_description,
            'referenced_type_descriptions': sorted(
                referenced_types.values(), key=lambda td: td['type_name'])
        }

        hashed_type_description = {
            'hashes': self._calculate_hash_tree(),
            'type_description_msg': self.full_type_description,
            'subinterfaces': self._all_interface_references(),
        }

        json_path = output_dir / self.rel_path.with_suffix('.json')
        with json_path.open('w', encoding='utf-8') as json_file:
            json_file.write(json.dumps(hashed_type_description, indent=2))
        return generated_files + [json_path]

    def _all_interface_references(self) -> dict:
        results = {
            self.individual_type_description['type_name']: [
                x['type_name']
                for x in self.full_type_description['referenced_type_descriptions']
            ]
        }
        if self.subinterfaces:
            for hasher in self.subinterfaces.values():
                results.update(hasher._all_interface_references())
        return results

    def _calculate_hash_tree(self) -> dict:
        hashable_dict = deepcopy(self.full_type_description)
        for field in hashable_dict['type_description']['fields']:
            del field['default_value']
        for referenced_td in hashable_dict['referenced_type_descriptions']:
            for field in referenced_td['fields']:
                del field['default_value']

        hashable_repr = json.dumps(
            hashable_dict,
            skipkeys=False,
            ensure_ascii=True,
            check_circular=True,
            allow_nan=False,
            indent=None,
            # note: libyaml in C doesn't allow for tweaking these separators, this is its builtin
            separators=(', ', ': '),
            sort_keys=False
        )
        sha = hashlib.sha256()
        sha.update(hashable_repr.encode('utf-8'))
        type_hash = RIHS01_PREFIX + sha.hexdigest()

        type_hash_infos = {
            self.interface_type: type_hash,
        }
        for key, val in self.subinterfaces.items():
            type_hash_infos[key] = val._calculate_hash_tree()

        return type_hash_infos


def extract_subinterface(type_description_msg: dict, field_name: str, subinterfaces: dict):
    """
    Given a full TypeDescription json with all referenced type descriptions,
    produce a top-level TypeDescription for one of its referenced types.
    """
    nested_type_name = next(
        field['type']['nested_type_name']
        for field in type_description_msg['type_description']['fields']
        if field['name'] == field_name)
    toplevel_type_description = next(
        type_description
        for type_description in type_description_msg['referenced_type_descriptions']
        if type_description['type_name'] == nested_type_name)
    referenced_types = subinterfaces[nested_type_name]

    return {
        'type_description': toplevel_type_description,
        'referenced_type_descriptions': [
            td for td in type_description_msg['referenced_type_descriptions']
            if td['type_name'] in referenced_types],
    }
