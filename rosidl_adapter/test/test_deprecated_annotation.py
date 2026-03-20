# Copyright 2024 Open Source Robotics Foundation, Inc.
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

from rosidl_adapter.parser import parse_message_string


def test_deprecated_annotation_on_field() -> None:
    """Test that '# @deprecated' in a field comment sets the deprecated annotation."""
    msg_string = (
        'int32 new_field\n'
        '# This field is replaced by new_field.\n'
        '# @deprecated\n'
        'int32 old_field\n'
    )
    msg_spec = parse_message_string('pkg', 'Foo', msg_string)
    assert len(msg_spec.fields) == 2

    new_field = msg_spec.fields[0]
    assert new_field.name == 'new_field'
    assert new_field.annotations.get('deprecated') is None

    old_field = msg_spec.fields[1]
    assert old_field.name == 'old_field'
    assert old_field.annotations.get('deprecated') is True
    # The @deprecated line should be stripped from the comment text
    comment_lines = old_field.annotations.get('comment', [])
    for line in comment_lines:
        assert '@deprecated' not in line


def test_deprecated_annotation_standalone() -> None:
    """Test that '# @deprecated' placed before a field sets the field annotation."""
    # Test the format: two regular fields, then # @deprecated followed by deprecated field
    msg_string = (
        'geometry_msgs/Pose pose_stamped\n'
        'float64 distance_meters\n'
        '# @deprecated\n'
        'uint8 distance_cm\n'
    )
    msg_spec = parse_message_string('pkg', 'Foo', msg_string)
    assert len(msg_spec.fields) == 3

    # First two fields are not deprecated
    assert msg_spec.fields[0].name == 'pose_stamped'
    assert msg_spec.fields[0].annotations.get('deprecated') is None
    assert msg_spec.fields[1].name == 'distance_meters'
    assert msg_spec.fields[1].annotations.get('deprecated') is None

    # Third field is deprecated
    field = msg_spec.fields[2]
    assert field.name == 'distance_cm'
    assert field.annotations.get('deprecated') is True
    # Comment should be empty or absent after stripping the @deprecated line
    comment_lines = field.annotations.get('comment', [])
    assert comment_lines == [] or all(line == '' for line in comment_lines)


def test_non_deprecated_field_unaffected() -> None:
    """Test that a regular comment does not set the deprecated annotation."""
    msg_string = (
        '# A regular comment.\n'
        'int32 normal_field\n'
    )
    msg_spec = parse_message_string('pkg', 'Foo', msg_string)
    assert len(msg_spec.fields) == 1
    field = msg_spec.fields[0]
    assert field.annotations.get('deprecated') is None
