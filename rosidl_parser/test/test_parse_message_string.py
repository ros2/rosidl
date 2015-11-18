# Copyright 2014 Open Source Robotics Foundation, Inc.
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

from nose.tools import assert_raises

from rosidl_parser import InvalidFieldDefinition
from rosidl_parser import InvalidResourceName
from rosidl_parser import parse_message_string


def test_parse_message_string():
    msg_spec = parse_message_string('pkg', 'Foo', '')
    assert msg_spec.base_type.pkg_name == 'pkg'
    assert msg_spec.base_type.type == 'Foo'
    assert len(msg_spec.fields) == 0
    assert len(msg_spec.constants) == 0

    msg_spec = parse_message_string('pkg', 'Foo', '#comment\n \n  # comment')
    assert len(msg_spec.fields) == 0
    assert len(msg_spec.constants) == 0

    with assert_raises(InvalidFieldDefinition):
        parse_message_string('pkg', 'Foo', 'bool  # comment')

    msg_spec = parse_message_string('pkg', 'Foo', 'bool foo')
    assert len(msg_spec.fields) == 1
    assert msg_spec.fields[0].type.type == 'bool'
    assert msg_spec.fields[0].name == 'foo'
    assert msg_spec.fields[0].default_value is None
    assert len(msg_spec.constants) == 0

    msg_spec = parse_message_string('pkg', 'Foo', 'bool foo 1')
    assert len(msg_spec.fields) == 1
    assert msg_spec.fields[0].type.type == 'bool'
    assert msg_spec.fields[0].name == 'foo'
    assert msg_spec.fields[0].default_value
    assert len(msg_spec.constants) == 0

    with assert_raises(InvalidResourceName):
        parse_message_string('pkg', 'Foo', 'Ty_pe foo')
    with assert_raises(TypeError):
        parse_message_string('pkg', 'Foo', 'bool] foo')
    with assert_raises(TypeError):
        parse_message_string('pkg', 'Foo', 'bool[max]] foo')
    with assert_raises(ValueError) as ctx:
        parse_message_string('pkg', 'Foo', 'bool foo\nbool foo')
    assert 'foo' in str(ctx.exception)

    msg_spec = parse_message_string('pkg', 'Foo', 'bool FOO=1')
    assert len(msg_spec.fields) == 0
    assert len(msg_spec.constants) == 1
    assert msg_spec.constants[0].type == 'bool'
    assert msg_spec.constants[0].name == 'FOO'
    assert msg_spec.constants[0].value

    with assert_raises(TypeError):
        parse_message_string('pkg', 'Foo', 'pkg/Bar foo=1')
    with assert_raises(NameError):
        parse_message_string('pkg', 'Foo', 'bool foo=1')
    with assert_raises(ValueError) as ctx:
        parse_message_string('pkg', 'Foo', 'bool FOO=1\nbool FOO=1')
    assert 'FOO' in str(ctx.exception)
