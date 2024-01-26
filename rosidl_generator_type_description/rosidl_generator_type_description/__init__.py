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

# Used by code generators to create variable names
GET_DESCRIPTION_FUNC = 'get_type_description'
GET_HASH_FUNC = 'get_type_hash'
GET_INDIVIDUAL_SOURCE_FUNC = 'get_individual_type_description_source'
GET_SOURCES_FUNC = 'get_type_description_sources'


def to_type_name(namespaced_type):
    return '/'.join(namespaced_type.namespaced_name())


class GenericInterface:
    def __init__(
        self, namespaced_type: definition.NamespacedType, members: List[definition.Member]
    ):
        self.namespaced_type = namespaced_type
        self.members = members


def add_msg(msg: definition.Message, to_dict: dict):
    to_dict[to_type_name(msg.structure.namespaced_type)] = GenericInterface(
        msg.structure.namespaced_type, msg.structure.members)


def add_srv(srv: definition.Service, to_dict: dict):
    service_members = [
        definition.Member(srv.request_message.structure.namespaced_type, 'request_message'),
        definition.Member(srv.response_message.structure.namespaced_type, 'response_message'),
        definition.Member(srv.event_message.structure.namespaced_type, 'event_message'),
    ]
    to_dict[to_type_name(srv.namespaced_type)] = GenericInterface(
        srv.namespaced_type, service_members)
    add_msg(srv.request_message, to_dict)
    add_msg(srv.response_message, to_dict)
    add_msg(srv.event_message, to_dict)


def add_action(action, to_dict):
    action_members = [
        definition.Member(action.goal.structure.namespaced_type, 'goal'),
        definition.Member(action.result.structure.namespaced_type, 'result'),
        definition.Member(action.feedback.structure.namespaced_type, 'feedback'),
        definition.Member(action.send_goal_service.namespaced_type, 'send_goal_service'),
        definition.Member(action.get_result_service.namespaced_type, 'get_result_service'),
        definition.Member(action.feedback_message.structure.namespaced_type, 'feedback_message'),
    ]
    to_dict[to_type_name(action.namespaced_type)] = GenericInterface(
        action.namespaced_type, action_members)
    add_msg(action.goal, to_dict)
    add_msg(action.result, to_dict)
    add_msg(action.feedback, to_dict)
    add_srv(action.send_goal_service, to_dict)
    add_srv(action.get_result_service, to_dict)
    add_msg(action.feedback_message, to_dict)


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

    # Define all local IndividualTypeDescriptions
    individual_types = {}
    for idl_tuple in idl_tuples:
        idl_parts = idl_tuple.rsplit(':', 1)
        assert len(idl_parts) == 2
        locator = definition.IdlLocator(*idl_parts)
        try:
            idl_file = parse_idl_file(locator)
        except Exception as e:
            print('Error processing idl file: ' +
                  str(locator.get_absolute_path()), file=sys.stderr)
            raise e

        idl_rel_path = Path(idl_parts[1])
        generate_to_dir = (output_dir / idl_rel_path).parent
        generate_to_dir.mkdir(parents=True, exist_ok=True)
        for el in idl_file.content.elements:
            if isinstance(el, definition.Message):
                add_msg(el, individual_types)
            elif isinstance(el, definition.Service):
                add_srv(el, individual_types)
            elif isinstance(el, definition.Action):
                add_action(el, individual_types)

    # Determine needed includes for types from other packages
    pending_includes = set()
    for individual_type in individual_types.values():
        for member in individual_type.members:
            if isinstance(member.type, definition.NamespacedType):
                member_type = member.type
            elif (
                isinstance(member.type, definition.AbstractNestedType) and
                isinstance(member.type.value_type, definition.NamespacedType)
            ):
                member_type = member.type.value_type
            else:
                continue

            if to_type_name(member_type) not in individual_types:
                pending_includes.add(Path(*member_type.namespaced_name()))

    # Load all included types, create lookup maps of included individual descriptions and hashes
    serialized_type_lookup = {
        key: serialize_individual_type_description(val.namespaced_type, val.members)
        for key, val in individual_types.items()
    }
    hash_lookup = {}
    while pending_includes:
        process_include = pending_includes.pop()
        p_path = process_include.with_suffix('.json')
        pkg = p_path.parts[0]
        pkg_dir = include_map[pkg]
        include_path = pkg_dir / p_path.relative_to(pkg)
        with include_path.open('r') as include_file:
            include_json = json.load(include_file)

        type_description_msg = include_json['type_description_msg']
        try:
            hash_lookup.update({
                val['type_name']: val['hash_string'] for val in include_json['type_hashes']
            })
        except KeyError:
            raise Exception(f'Key "type_hashes" not  found in {include_path}')

        serialized_type_lookup[type_description_msg['type_description']['type_name']] = \
            type_description_msg['type_description']
        for referenced_type in type_description_msg['referenced_type_descriptions']:
            serialized_type_lookup[referenced_type['type_name']] = referenced_type

    # Create fully-unrolled TypeDescription instances for local full types, and calculate hashes
    full_types = []
    for type_name, individual_type in individual_types.items():
        full_type_description = extract_full_type_description(type_name, serialized_type_lookup)
        full_types.append(full_type_description)
        hash_lookup[type_name] = calculate_type_hash(full_type_description)

    # Write JSON output for each full TypeDescription
    generated_files = []
    for full_type_description in full_types:
        top_type_name = full_type_description['type_description']['type_name']
        hashes = [{
            'type_name': top_type_name,
            'hash_string': hash_lookup[top_type_name],
        }]
        for referenced_type in full_type_description['referenced_type_descriptions']:
            hashes.append({
                'type_name': referenced_type['type_name'],
                'hash_string': hash_lookup[referenced_type['type_name']],
            })
        json_content = {
            'type_description_msg': full_type_description,
            'type_hashes': hashes,
        }
        rel_path = Path(*top_type_name.split('/')[1:])
        json_path = output_dir / rel_path.with_suffix('.json')
        with json_path.open('w', encoding='utf-8') as json_file:
            json_file.write(json.dumps(json_content, indent=2))
        generated_files.append(json_path)

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
FIELD_TYPE_NAME_TO_ID = {
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

FIELD_TYPE_ID_TO_NAME = {
    val: key for key, val in FIELD_TYPE_NAME_TO_ID.items()
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
    return FIELD_TYPE_NAME_TO_ID[field_type_type_name(ftype)]


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
        'type_name': to_type_name(namespaced_type),
        'fields': [serialize_field(member) for member in members]
    }


def calculate_type_hash(serialized_type_description):
    # Create a copy of the description, removing all default values
    hashable_dict = deepcopy(serialized_type_description)
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
    return type_hash


def extract_full_type_description(output_type_name, type_map):
    # Traverse reference graph to narrow down the references for the output type
    output_type = type_map[output_type_name]
    output_references = set()
    process_queue = [
        field['type']['nested_type_name']
        for field in output_type['fields']
        if field['type']['nested_type_name']
    ]
    while process_queue:
        process_type = process_queue.pop()
        if process_type not in output_references:
            output_references.add(process_type)
            process_queue.extend([
                field['type']['nested_type_name']
                for field in type_map[process_type]['fields']
                if field['type']['nested_type_name']
            ])

    return {
        'type_description': output_type,
        'referenced_type_descriptions': [
            type_map[type_name] for type_name in sorted(output_references)
        ],
    }


def extract_subinterface(type_description_msg: dict, field_name: str):
    """
    Filter full TypeDescription to produce a TypeDescription for one of its fields' types.

    Given the name of a field, finds its type, and finds all its referenced type descriptions
    by doing a DAG traversal on the referenced type descriptions of the input type.
    """
    output_type_name = next(
        field['type']['nested_type_name']
        for field in type_description_msg['type_description']['fields']
        if field['name'] == field_name)
    assert output_type_name, 'Given field is not a nested type'

    # Create a lookup map for matching names to type descriptions
    toplevel_type = type_description_msg['type_description']
    referenced_types = type_description_msg['referenced_type_descriptions']
    type_map = {
        individual_type['type_name']: individual_type
        for individual_type
        in [toplevel_type] + referenced_types
    }
    return extract_full_type_description(output_type_name, type_map)
