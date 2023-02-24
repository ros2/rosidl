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

from rosidl_generator_type_hash import serialize_field_type

from rosidl_parser import definition


def test_field_type_serializer():
    # Sanity check for the more complex length/string_length types and nesting
    string_limit = 12
    array_size = 22
    test_type = definition.Array(definition.BoundedString(string_limit), array_size)
    expected = {
        'type_id': 69,
        'length': array_size,
        'string_length': string_limit,
        'nested_type_name': '',
    }

    result = serialize_field_type(test_type)
    assert result == expected

    bounded_sequence_limit = 32
    test_type = definition.BoundedSequence(definition.UnboundedString(), bounded_sequence_limit)
    expected = {
        'type_id': 113,
        'length': bounded_sequence_limit,
        'string_length': 0,
        'nested_type_name': '',
    }
    result = serialize_field_type(test_type)
    assert result == expected

    test_type = definition.BoundedWString(string_limit)
    expected = {
        'type_id': 22,
        'length': 0,
        'string_length': string_limit,
        'nested_type_name': '',
    }
    result = serialize_field_type(test_type)
    assert result == expected
