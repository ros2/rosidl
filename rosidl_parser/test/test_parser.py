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

from rosidl_parser.definition import Array
from rosidl_parser.definition import BasicType
from rosidl_parser.definition import BoundedSequence
from rosidl_parser.definition import Message
from rosidl_parser.definition import Service
from rosidl_parser.definition import String
from rosidl_parser.definition import UnboundedSequence
from rosidl_parser.definition import WString
from rosidl_parser.parser import parse_idl_file

MESSAGE_IDL_FILE = pathlib.Path(__file__).parent / 'MyMessage.idl'
SERVICE_IDL_FILE = pathlib.Path(__file__).parent / 'MyService.idl'


@pytest.fixture(scope='module')
def msg():
    return parse_idl_file(MESSAGE_IDL_FILE)


def test_message_parser_specification(msg):
    assert isinstance(msg, Message)


def test_message_parser_includes(msg):
    assert len(msg.includes) == 2
    assert msg.includes[0] == 'OtherMessage.idl'
    assert msg.includes[1] == 'pkgname/msg/OtherMessage.idl'


def test_message_parser_constants(msg):
    assert len(msg.constants) == 5

    assert 'SHORT_CONSTANT' in msg.constants
    assert isinstance(msg.constants['SHORT_CONSTANT'][0], BasicType)
    assert msg.constants['SHORT_CONSTANT'][0].type == 'int16'
    assert msg.constants['SHORT_CONSTANT'][1] == -23

    assert 'UNSIGNED_LONG_CONSTANT' in msg.constants
    assert isinstance(msg.constants['UNSIGNED_LONG_CONSTANT'][0], BasicType)
    assert msg.constants['UNSIGNED_LONG_CONSTANT'][0].type == 'uint32'
    assert msg.constants['UNSIGNED_LONG_CONSTANT'][1] == 42

    assert 'FLOAT_CONSTANT' in msg.constants
    assert isinstance(msg.constants['FLOAT_CONSTANT'][0], BasicType)
    assert msg.constants['FLOAT_CONSTANT'][0].type == 'float'
    assert msg.constants['FLOAT_CONSTANT'][1] == 1.25

    assert 'BOOLEAN_CONSTANT' in msg.constants
    assert isinstance(msg.constants['BOOLEAN_CONSTANT'][0], BasicType)
    assert msg.constants['BOOLEAN_CONSTANT'][0].type == 'boolean'
    assert msg.constants['BOOLEAN_CONSTANT'][1] is True

    assert 'STRING_CONSTANT' in msg.constants
    assert isinstance(msg.constants['STRING_CONSTANT'][0], String)
    assert msg.constants['STRING_CONSTANT'][1] == 'string_value'


def test_message_parser_structure(msg):
    assert msg.structure.type.namespaces == ['rosidl_parser', 'msg']
    assert msg.structure.type.name == 'MyMessage'
    assert len(msg.structure.members) == 30

    assert isinstance(msg.structure.members[0].type, BasicType)
    assert msg.structure.members[0].type.type == 'int16'
    assert msg.structure.members[0].name == 'short_value'
    assert isinstance(msg.structure.members[1].type, BasicType)
    assert msg.structure.members[1].type.type == 'int16'
    assert msg.structure.members[1].name == 'short_value2'

    assert isinstance(msg.structure.members[22].type, String)
    assert msg.structure.members[22].type.maximum_size is None
    assert msg.structure.members[22].name == 'string_value'
    assert isinstance(msg.structure.members[23].type, String)
    assert msg.structure.members[23].type.maximum_size == 5
    assert msg.structure.members[23].name == 'bounded_string_value'

    assert isinstance(msg.structure.members[24].type, WString)
    assert msg.structure.members[24].type.maximum_size is None
    assert msg.structure.members[24].name == 'wstring_value'
    assert isinstance(msg.structure.members[25].type, WString)
    assert msg.structure.members[25].type.maximum_size == 23
    assert msg.structure.members[25].name == 'bounded_wstring_value'
    assert isinstance(msg.structure.members[26].type, WString)
    assert msg.structure.members[26].type.maximum_size == 'UNSIGNED_LONG_CONSTANT'
    assert msg.structure.members[26].name == 'constant_bounded_wstring_value'

    assert isinstance(msg.structure.members[27].type, UnboundedSequence)
    assert isinstance(msg.structure.members[27].type.basetype, BasicType)
    assert msg.structure.members[27].type.basetype.type == 'int16'
    assert msg.structure.members[27].name == 'unbounded_short_values'
    assert isinstance(msg.structure.members[28].type, BoundedSequence)
    assert isinstance(msg.structure.members[28].type.basetype, BasicType)
    assert msg.structure.members[28].type.basetype.type == 'int16'
    assert msg.structure.members[28].type.upper_bound == 5
    assert msg.structure.members[28].name == 'bounded_short_values'
    assert isinstance(msg.structure.members[29].type, Array)
    assert isinstance(msg.structure.members[29].type.basetype, BasicType)
    assert msg.structure.members[29].type.basetype.type == 'int16'
    assert msg.structure.members[29].type.size == 23
    assert msg.structure.members[29].name == 'array_short_values'


def test_message_parser_annotations(msg):
    assert len(msg.structure.members[2].annotations) == 1

    assert msg.structure.members[2].annotations[0][0] == 'default'
    assert len(msg.structure.members[2].annotations[0][1]) == 1
    assert 'value' in msg.structure.members[2].annotations[0][1]
    assert msg.structure.members[2].annotations[0][1]['value'] == 123

    assert len(msg.structure.members[3].annotations) == 2

    assert msg.structure.members[3].annotations[0][0] == 'key'
    assert msg.structure.members[3].annotations[0][1] is None

    assert msg.structure.members[3].annotations[1][0] == 'range'
    assert len(msg.structure.members[3].annotations[1][1]) == 2
    assert 'min' in msg.structure.members[3].annotations[1][1]
    assert msg.structure.members[3].annotations[1][1]['min'] == -10
    assert 'max' in msg.structure.members[3].annotations[1][1]
    assert msg.structure.members[3].annotations[1][1]['max'] == 10


def test_service_parser():
    srv = parse_idl_file(SERVICE_IDL_FILE)
    assert isinstance(srv, Service)
    assert srv.structure_type.namespaces == ['rosidl_parser', 'srv']
    assert srv.structure_type.name == 'MyService'
    assert len(srv.request_message.structure.members) == 2
    assert len(srv.response_message.structure.members) == 1
