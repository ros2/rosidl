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

from rosidl_parser import BaseType
from rosidl_parser import InvalidResourceName


def test_base_type_constructor():
    primitive_types = [
        'bool',
        'float32',
        'float64',
        'int8',
        'uint8',
        'int16',
        'uint16',
        'int32',
        'uint32',
        'int64',
        'uint64',
        'string']

    deprecated_types = {
        'byte': 'int8',
        'char': 'uint8'
    }

    for primitive_type in primitive_types:
        base_type = BaseType(primitive_type)
        assert base_type.pkg_name is None
        assert base_type.type == primitive_type
        assert base_type.string_upper_bound is None
    for deprecated_type in deprecated_types.keys():
        base_type = BaseType(deprecated_type)
        assert base_type.pkg_name is None
        assert base_type.type == deprecated_types[deprecated_type]
        assert base_type.string_upper_bound is None

    base_type = BaseType('string<=23')
    assert base_type.pkg_name is None
    assert base_type.type == 'string'
    assert base_type.string_upper_bound == 23

    with assert_raises(TypeError):
        BaseType('string<=upperbound')
    with assert_raises(TypeError):
        BaseType('string<=0')

    base_type = BaseType('pkg/Msg')
    assert base_type.pkg_name == 'pkg'
    assert base_type.type == 'Msg'
    assert base_type.string_upper_bound is None

    base_type = BaseType('Msg', 'pkg')
    assert base_type.pkg_name == 'pkg'
    assert base_type.type == 'Msg'
    assert base_type.string_upper_bound is None

    with assert_raises(InvalidResourceName):
        BaseType('Foo')

    with assert_raises(InvalidResourceName):
        BaseType('pkg name/Foo')

    with assert_raises(InvalidResourceName):
        BaseType('pkg/Foo Bar')


def test_base_type_methods():
    assert BaseType('bool').is_primitive_type()
    assert not BaseType('pkg/Foo').is_primitive_type()

    assert BaseType('bool') != 23

    assert BaseType('pkg/Foo') == BaseType('pkg/Foo')
    assert BaseType('bool') != BaseType('pkg/Foo')

    {BaseType('bool'): None}

    assert str(BaseType('pkg/Foo')) == 'pkg/Foo'
    assert str(BaseType('bool')) == 'bool'
    assert str(BaseType('string<=5')) == 'string<=5'
