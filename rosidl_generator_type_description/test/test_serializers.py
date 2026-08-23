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

from rosidl_generator_type_description import serialize_field_type
from rosidl_generator_type_description import serialize_individual_type_description

from rosidl_parser import definition


def test_field_type_serializer():
    # Sanity check for the more complex capacity/string_capacity types and nesting
    string_limit = 12
    array_size = 22
    test_type = definition.Array(definition.BoundedString(string_limit), array_size)
    expected = {
        'type_id': 69,
        'capacity': array_size,
        'string_capacity': string_limit,
        'nested_type_name': '',
    }

    result = serialize_field_type(test_type)
    assert result == expected

    bounded_sequence_limit = 32
    test_type = definition.BoundedSequence(definition.UnboundedString(), bounded_sequence_limit)
    expected = {
        'type_id': 113,
        'capacity': bounded_sequence_limit,
        'string_capacity': 0,
        'nested_type_name': '',
    }
    result = serialize_field_type(test_type)
    assert result == expected

    test_type = definition.BoundedWString(string_limit)
    expected = {
        'type_id': 22,
        'capacity': 0,
        'string_capacity': string_limit,
        'nested_type_name': '',
    }
    result = serialize_field_type(test_type)
    assert result == expected


def test_nested_type_serializer():
    namespaced_type = definition.NamespacedType(['my_pkg', 'msg'], 'TestThing')
    referenced_type = definition.NamespacedType(['other_pkg', 'msg'], 'RefThing')
    nested_referenced_type = definition.UnboundedSequence(referenced_type)
    members = [
        definition.Member(referenced_type, 'ref_thing'),
        definition.Member(nested_referenced_type, 'ref_things')
    ]
    expected = {
        'type_name': 'my_pkg/msg/TestThing',
        'fields': [
            {
                'default_value': '',
                'name': 'ref_thing',
                'type': {
                    'type_id': 1,
                    'capacity': 0,
                    'string_capacity': 0,
                    'nested_type_name': 'other_pkg/msg/RefThing',
                },
            },
            {
                'default_value': '',
                'name': 'ref_things',
                'type': {
                    'type_id': 145,
                    'capacity': 0,
                    'string_capacity': 0,
                    'nested_type_name': 'other_pkg/msg/RefThing',
                },
            },
        ],
    }
    result = serialize_individual_type_description(namespaced_type, members)

    assert result == expected


def test_rihs02_hash():
    from rosidl_generator_type_description import calculate_type_hash, parse_rihs_string
    # Minimal type description
    desc = {
        'type_description': {
            'type_name': 'test_msgs/msg/Basic',
            'fields': [
                {'name': 'value', 'type': {'type_id': 2, 'capacity': 0, 'string_capacity': 0, 'nested_type_name': ''}}
            ]
        },
        'referenced_type_descriptions': []
    }
    hash_str = calculate_type_hash(desc, version=2)
    assert hash_str.startswith('RIHS02_')
    ver, val = parse_rihs_string(hash_str)
    assert ver == 2
    assert len(val) == 32


def test_rihs01_hash_still_works():
    from rosidl_generator_type_description import calculate_type_hash, parse_rihs_string
    desc = {
        'type_description': {
            'type_name': 'test_msgs/msg/Basic',
            'fields': []
        },
        'referenced_type_descriptions': []
    }
    hash_str = calculate_type_hash(desc, version=1)
    assert hash_str.startswith('RIHS01_')
    ver, val = parse_rihs_string(hash_str)
    assert ver == 1
    assert len(val) == 64
