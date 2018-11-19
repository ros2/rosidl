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

import pathlib

import pytest

from rosidl_parser.definition import Action
from rosidl_parser.definition import Array
from rosidl_parser.definition import BasicType
from rosidl_parser.definition import BoundedSequence
from rosidl_parser.definition import Constant
from rosidl_parser.definition import IdlLocator
from rosidl_parser.definition import Include
from rosidl_parser.definition import Message
from rosidl_parser.definition import NamespacedType
from rosidl_parser.definition import Service
from rosidl_parser.definition import String
from rosidl_parser.definition import UnboundedSequence
from rosidl_parser.definition import WString
from rosidl_parser.parser import parse_idl_file

MESSAGE_IDL_LOCATOR = IdlLocator(
    pathlib.Path(__file__).parent, pathlib.Path('msg') / 'MyMessage.idl')
SERVICE_IDL_LOCATOR = IdlLocator(
    pathlib.Path(__file__).parent, pathlib.Path('srv') / 'MyService.idl')
ACTION_IDL_LOCATOR = IdlLocator(
    pathlib.Path(__file__).parent, pathlib.Path('action') / 'MyAction.idl')


@pytest.fixture(scope='module')
def message_idl_file():
    return parse_idl_file(MESSAGE_IDL_LOCATOR)


def test_message_parser(message_idl_file):
    messages = message_idl_file.content.get_elements_of_type(Message)
    assert len(messages) == 1


def test_message_parser_includes(message_idl_file):
    includes = message_idl_file.content.get_elements_of_type(Include)
    assert len(includes) == 2
    assert includes[0].locator == 'OtherMessage.idl'
    assert includes[1].locator == 'pkgname/msg/OtherMessage.idl'


def test_message_parser_constants(message_idl_file):
    constants = message_idl_file.content.get_elements_of_type(Constant)
    assert len(constants) == 5

    constant = [c for c in constants if c.name == 'SHORT_CONSTANT']
    assert len(constant) == 1
    constant = constant[0]
    assert isinstance(constant.type, BasicType)
    assert constant.type.type == 'int16'
    assert constant.value == -23

    constant = [c for c in constants if c.name == 'UNSIGNED_LONG_CONSTANT']
    assert len(constant) == 1
    constant = constant[0]
    assert isinstance(constant.type, BasicType)
    assert constant.type.type == 'uint32'
    assert constant.value == 42

    constant = [c for c in constants if c.name == 'FLOAT_CONSTANT']
    assert len(constant) == 1
    constant = constant[0]
    assert isinstance(constant.type, BasicType)
    assert constant.type.type == 'float'
    assert constant.value == 1.25

    constant = [c for c in constants if c.name == 'BOOLEAN_CONSTANT']
    assert len(constant) == 1
    constant = constant[0]
    assert isinstance(constant.type, BasicType)
    assert constant.type.type == 'boolean'
    assert constant.value is True

    constant = [c for c in constants if c.name == 'STRING_CONSTANT']
    assert len(constant) == 1
    constant = constant[0]
    assert isinstance(constant.type, String)
    assert constant.value == 'string_value'


def test_message_parser_structure(message_idl_file):
    messages = message_idl_file.content.get_elements_of_type(Message)
    assert len(messages) == 1
    structure = messages[0].structure

    assert structure.type.namespaces == ['rosidl_parser', 'msg']
    assert structure.type.name == 'MyMessage'
    assert len(structure.members) == 30

    assert isinstance(structure.members[0].type, BasicType)
    assert structure.members[0].type.type == 'int16'
    assert structure.members[0].name == 'short_value'
    assert isinstance(structure.members[1].type, BasicType)
    assert structure.members[1].type.type == 'int16'
    assert structure.members[1].name == 'short_value2'

    assert isinstance(structure.members[22].type, String)
    assert structure.members[22].type.maximum_size is None
    assert structure.members[22].name == 'string_value'
    assert isinstance(structure.members[23].type, String)
    assert structure.members[23].type.maximum_size == 5
    assert structure.members[23].name == 'bounded_string_value'

    assert isinstance(structure.members[24].type, WString)
    assert structure.members[24].type.maximum_size is None
    assert structure.members[24].name == 'wstring_value'
    assert isinstance(structure.members[25].type, WString)
    assert structure.members[25].type.maximum_size == 23
    assert structure.members[25].name == 'bounded_wstring_value'
    assert isinstance(structure.members[26].type, WString)
    assert structure.members[26].type.maximum_size == 'UNSIGNED_LONG_CONSTANT'
    assert structure.members[26].name == 'constant_bounded_wstring_value'

    assert isinstance(structure.members[27].type, UnboundedSequence)
    assert isinstance(structure.members[27].type.basetype, BasicType)
    assert structure.members[27].type.basetype.type == 'int16'
    assert structure.members[27].name == 'unbounded_short_values'
    assert isinstance(structure.members[28].type, BoundedSequence)
    assert isinstance(structure.members[28].type.basetype, BasicType)
    assert structure.members[28].type.basetype.type == 'int16'
    assert structure.members[28].type.upper_bound == 5
    assert structure.members[28].name == 'bounded_short_values'
    assert isinstance(structure.members[29].type, Array)
    assert isinstance(structure.members[29].type.basetype, BasicType)
    assert structure.members[29].type.basetype.type == 'int16'
    assert structure.members[29].type.size == 23
    assert structure.members[29].name == 'array_short_values'


def test_message_parser_annotations(message_idl_file):
    messages = message_idl_file.content.get_elements_of_type(Message)
    assert len(messages) == 1
    structure = messages[0].structure

    assert len(structure.members[2].annotations) == 1

    assert structure.members[2].annotations[0].name == 'default'
    assert len(structure.members[2].annotations[0].value) == 1
    assert 'value' in structure.members[2].annotations[0].value
    assert structure.members[2].annotations[0].value['value'] == 123

    assert len(structure.members[3].annotations) == 2

    assert structure.members[3].annotations[0].name == 'key'
    assert structure.members[3].annotations[0].value is None

    assert structure.members[3].annotations[1].name == 'range'
    assert len(structure.members[3].annotations[1].value) == 2
    assert 'min' in structure.members[3].annotations[1].value
    assert structure.members[3].annotations[1].value['min'] == -10
    assert 'max' in structure.members[3].annotations[1].value
    assert structure.members[3].annotations[1].value['max'] == 10


@pytest.fixture(scope='module')
def service_idl_file():
    return parse_idl_file(SERVICE_IDL_LOCATOR)


def test_service_parser(service_idl_file):
    services = service_idl_file.content.get_elements_of_type(Service)
    assert len(services) == 1

    srv = services[0]
    assert isinstance(srv, Service)
    assert srv.structure_type.namespaces == ['rosidl_parser', 'srv']
    assert srv.structure_type.name == 'MyService'
    assert len(srv.request_message.structure.members) == 2
    assert len(srv.response_message.structure.members) == 1


@pytest.fixture(scope='module')
def action_idl_file():
    return parse_idl_file(ACTION_IDL_LOCATOR)


def test_action_parser(action_idl_file):
    actions = action_idl_file.content.get_elements_of_type(Action)
    assert len(actions) == 1

    action = actions[0]
    assert isinstance(action, Action)
    assert action.structure_type.namespaces == ['rosidl_parser', 'action']
    assert action.structure_type.name == 'MyAction'

    # check messages defined in the idl file
    structure = action.goal_request.structure
    assert structure.type.namespaces == ['rosidl_parser', 'action']
    assert structure.type.name == 'MyAction_Goal_Request'
    assert len(structure.members) == 1
    assert isinstance(structure.members[0].type, BasicType)
    assert structure.members[0].type.type == 'int32'
    assert structure.members[0].name == 'input_value'

    structure = action.result_response.structure
    assert structure.type.namespaces == ['rosidl_parser', 'action']
    assert structure.type.name == 'MyAction_Result_Response'
    assert len(structure.members) == 1
    assert isinstance(structure.members[0].type, BasicType)
    assert structure.members[0].type.type == 'uint32'
    assert structure.members[0].name == 'output_value'

    structure = action.feedback.structure
    assert structure.type.namespaces == ['rosidl_parser', 'action']
    assert structure.type.name == 'MyAction_Feedback'
    assert len(structure.members) == 1
    assert isinstance(structure.members[0].type, BasicType)
    assert structure.members[0].type.type == 'float'
    assert structure.members[0].name == 'progress_value'

    # check derived goal service
    structure_type = action.goal_service.structure_type
    assert structure_type.namespaces == ['rosidl_parser', 'action']
    assert structure_type.name == 'MyAction_Action_Goal'

    structure = action.goal_service.request_message.structure
    assert len(structure.members) == 2

    assert isinstance(structure.members[0].type, Array)
    assert structure.members[0].type.size == 16
    assert isinstance(structure.members[0].type.basetype, BasicType)
    assert structure.members[0].type.basetype.type == 'uint8'
    assert structure.members[0].name == 'uuid'

    assert isinstance(structure.members[1].type, NamespacedType)
    assert structure.members[1].type.namespaces == \
        action.goal_request.structure.type.namespaces
    assert structure.members[1].type.name == \
        action.goal_request.structure.type.name

    structure = action.goal_service.response_message.structure
    assert len(structure.members) == 2

    assert isinstance(structure.members[0].type, BasicType)
    assert structure.members[0].type.type == 'boolean'
    assert structure.members[0].name == 'accepted'

    assert isinstance(structure.members[1].type, NamespacedType)
    assert structure.members[1].type.namespaces == [
        'builtin_interfaces', 'msg']
    assert structure.members[1].type.name == 'Time'
    assert structure.members[1].name == 'stamp'

    # check derived result service
    structure_type = action.result_service.structure_type
    assert structure_type.namespaces == ['rosidl_parser', 'action']
    assert structure_type.name == 'MyAction_Action_Result'

    structure = action.result_service.request_message.structure
    assert len(structure.members) == 1

    assert isinstance(structure.members[0].type, Array)
    assert structure.members[0].type.size == 16
    assert isinstance(structure.members[0].type.basetype, BasicType)
    assert structure.members[0].type.basetype.type == 'uint8'
    assert structure.members[0].name == 'uuid'

    structure = action.result_service.response_message.structure
    assert len(structure.members) == 2

    assert isinstance(structure.members[0].type, BasicType)
    assert structure.members[0].type.type == 'int8'
    assert structure.members[0].name == 'status'

    assert isinstance(structure.members[1].type, NamespacedType)
    assert structure.members[1].type.namespaces == \
        action.result_response.structure.type.namespaces
    assert structure.members[1].type.name == \
        action.result_response.structure.type.name

    # check derived feedback message
    structure = action.feedback_message.structure
    assert structure.type.namespaces == ['rosidl_parser', 'action']
    assert structure.type.name == 'MyAction_Action_Feedback'

    assert len(structure.members) == 2

    assert isinstance(structure.members[0].type, Array)
    assert structure.members[0].type.size == 16
    assert isinstance(structure.members[0].type.basetype, BasicType)
    assert structure.members[0].type.basetype.type == 'uint8'
    assert structure.members[0].name == 'uuid'

    assert isinstance(structure.members[1].type, NamespacedType)
    assert structure.members[1].type.namespaces == \
        action.feedback.structure.type.namespaces
    assert structure.members[1].type.name == \
        action.feedback.structure.type.name
