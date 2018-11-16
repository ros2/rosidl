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

import pytest

from rosidl_adapter.parser import ImplicitFieldCollision
from rosidl_adapter.parser import InvalidActionSpecification
from rosidl_adapter.parser import parse_action_string


def test_invalid_action_specification():
    with pytest.raises(InvalidActionSpecification):
        parse_action_string('pkg', 'Foo', '')

    # too few action separators (---)
    with pytest.raises(InvalidActionSpecification):
        parse_action_string('pkg', 'Foo', 'bool foo\n---\nint8 bar')


def test_action_implicit_field_collision():
    # uuid collision on send goal service
    with pytest.raises(ImplicitFieldCollision):
        parse_action_string('pkg', 'Foo', 'bool foo\nstring uuid\n---\nint8 bar\n---\nbool foo')

    # status collision on get result service
    with pytest.raises(ImplicitFieldCollision):
        parse_action_string('pkg', 'Foo', 'bool foo\n---\nint8 bar\nstring status\n---\nbool foo')


def test_valid_action_string():
    parse_action_string('pkg', 'Foo', 'bool foo\n---\nint8 bar\n---')


def test_valid_action_string1():
    spec = parse_action_string('pkg', 'Foo', 'bool foo\n---\nint8 bar\n---\nbool foo')
    goal_service = spec.goal_service
    result_service = spec.result_service
    feedback_msg = spec.feedback
    # Goal service checks
    assert goal_service.pkg_name == 'pkg'
    assert goal_service.srv_name == 'Foo_Goal'
    assert goal_service.request.base_type.pkg_name == 'pkg'
    assert goal_service.request.base_type.type == 'Foo_Goal_Request'
    assert len(goal_service.request.fields) == 2  # including implicit uuid
    assert len(goal_service.request.constants) == 0
    assert goal_service.response.base_type.pkg_name == 'pkg'
    assert goal_service.response.base_type.type == 'Foo_Goal_Response'
    assert len(goal_service.response.fields) == 2
    assert len(goal_service.response.constants) == 0
    # Result service checks
    assert result_service.pkg_name == 'pkg'
    assert result_service.srv_name == 'Foo_Result'
    assert result_service.request.base_type.pkg_name == 'pkg'
    assert result_service.request.base_type.type == 'Foo_Result_Request'
    assert len(result_service.request.fields) == 1  # implicit uuid
    assert len(result_service.request.constants) == 0
    assert result_service.response.base_type.pkg_name == 'pkg'
    assert result_service.response.base_type.type == 'Foo_Result_Response'
    assert len(result_service.response.fields) == 2
    assert len(result_service.response.constants) == 0
    # Feedback message checks
    assert len(feedback_msg.fields) == 2
    assert feedback_msg.fields[0].type.type == 'uint8'
    assert feedback_msg.fields[0].name == 'uuid'
    assert feedback_msg.fields[1].type.type == 'bool'
    assert feedback_msg.fields[1].name == 'foo'
    assert len(feedback_msg.constants) == 0


def test_valid_action_string2():
    spec = parse_action_string(
        'pkg', 'Foo', '#comment---\n \nbool foo\n---\n#comment\n \nint8 bar\n---\nbool foo')
    goal_service = spec.goal_service
    result_service = spec.result_service
    feedback_msg = spec.feedback
    # Goal service checks
    assert goal_service.pkg_name == 'pkg'
    assert goal_service.srv_name == 'Foo_Goal'
    assert goal_service.request.base_type.pkg_name == 'pkg'
    assert goal_service.request.base_type.type == 'Foo_Goal_Request'
    assert len(goal_service.request.fields) == 2  # including implicit uuid
    assert len(goal_service.request.constants) == 0
    assert goal_service.response.base_type.pkg_name == 'pkg'
    assert goal_service.response.base_type.type == 'Foo_Goal_Response'
    assert len(goal_service.response.fields) == 2
    assert len(goal_service.response.constants) == 0
    # Result service checks
    assert result_service.pkg_name == 'pkg'
    assert result_service.srv_name == 'Foo_Result'
    assert result_service.request.base_type.pkg_name == 'pkg'
    assert result_service.request.base_type.type == 'Foo_Result_Request'
    assert len(result_service.request.fields) == 1  # implicit uuid
    assert len(result_service.request.constants) == 0
    assert result_service.response.base_type.pkg_name == 'pkg'
    assert result_service.response.base_type.type == 'Foo_Result_Response'
    assert len(result_service.response.fields) == 2
    assert len(result_service.response.constants) == 0
    # Feedback message checks
    assert len(feedback_msg.fields) == 2
    assert feedback_msg.fields[0].type.type == 'uint8'
    assert feedback_msg.fields[0].name == 'uuid'
    assert feedback_msg.fields[1].type.type == 'bool'
    assert feedback_msg.fields[1].name == 'foo'
    assert len(feedback_msg.constants) == 0


def test_valid_action_string3():
    spec = parse_action_string(
        'pkg',
        'Foo',
        'bool foo\nstring status\n---\nbool FOO=1\nint8 bar\n---\nbool BAR=1\nbool foo')
    goal_service = spec.goal_service
    result_service = spec.result_service
    feedback_msg = spec.feedback
    # Goal service checks
    assert goal_service.pkg_name == 'pkg'
    assert goal_service.srv_name == 'Foo_Goal'
    assert goal_service.request.base_type.pkg_name == 'pkg'
    assert goal_service.request.base_type.type == 'Foo_Goal_Request'
    assert len(goal_service.request.fields) == 3  # including implicit uuid
    assert len(goal_service.request.constants) == 0
    assert goal_service.response.base_type.pkg_name == 'pkg'
    assert goal_service.response.base_type.type == 'Foo_Goal_Response'
    assert len(goal_service.response.fields) == 2
    assert len(goal_service.response.constants) == 0
    # Result service checks
    assert result_service.pkg_name == 'pkg'
    assert result_service.srv_name == 'Foo_Result'
    assert result_service.request.base_type.pkg_name == 'pkg'
    assert result_service.request.base_type.type == 'Foo_Result_Request'
    assert len(result_service.request.fields) == 1  # implicit uuid
    assert len(result_service.request.constants) == 0
    assert result_service.response.base_type.pkg_name == 'pkg'
    assert result_service.response.base_type.type == 'Foo_Result_Response'
    assert len(result_service.response.fields) == 2
    assert len(result_service.response.constants) == 1
    assert result_service.response.constants[0].type == 'bool'
    assert result_service.response.constants[0].name == 'FOO'
    assert result_service.response.constants[0].value
    # Feedback message checks
    assert len(feedback_msg.fields) == 2
    assert feedback_msg.fields[0].type.type == 'uint8'
    assert feedback_msg.fields[0].name == 'uuid'
    assert feedback_msg.fields[1].type.type == 'bool'
    assert feedback_msg.fields[1].name == 'foo'
    assert len(feedback_msg.constants) == 1
    assert feedback_msg.constants[0].type == 'bool'
    assert feedback_msg.constants[0].name == 'BAR'
    assert feedback_msg.constants[0].value
