# Copyright 2016 Open Source Robotics Foundation, Inc.
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

from rosidl_generator_py.msg import Constants
from rosidl_generator_py.msg import Nested
from rosidl_generator_py.msg import Primitives
from rosidl_generator_py.msg import Strings
from rosidl_generator_py.msg import Various


def test_strings():
    a = Strings()

    assert(a.empty_string is str())
    assert(a.def_string == 'Hello world!')


def test_invalid_attribute():
    a = Strings()

    assert_raises(AttributeError, setattr, a, 'invalid_string1', 'foo')

    assert_raises(AttributeError, getattr, a, 'invalid_string2')


def test_constructor():
    a = Strings(empty_string='foo')

    assert(a.empty_string == 'foo')

    assert_raises(AssertionError, Strings, unknown_field='test')


def test_constants():
    assert(Constants.X == 123)
    assert(Constants.Y == -123)
    assert(Constants.FOO == 'foo')
    assert(Constants.TOTO == '\x7F')
    assert(Constants.TATA == b'0')

    assert_raises(AttributeError, setattr, Constants, 'FOO', 'bar')


def test_default_values():
    a = Strings()

    assert(a.empty_string is str())
    assert(a.def_string == 'Hello world!')
    a.def_string = 'Bye world'
    assert(a.def_string == 'Bye world')
    assert(Strings.DEF_STRING__DEFAULT == 'Hello world!')
    assert(a.DEF_STRING__DEFAULT == 'Hello world!')
    assert_raises(AttributeError, setattr, Strings, 'DEF_STRING__DEFAULT', 'bar')

    b = Various()
    assert(b.TWO_UINT16_VALUE__DEFAULT == [5, 23])

    assert(b.UP_TO_THREE_INT32_VALUES_WITH_DEFAULT_VALUES__DEFAULT == [5, 23])

    assert(b.CHAR_VALUE__DEFAULT == '\x01')
    assert(b.CHAR_VALUE__DEFAULT != '1')
    assert(b.BYTE_VALUE__DEFAULT == b'\x01')
    assert(b.BYTE_VALUE__DEFAULT != b'1')


def test_check_constraints():
    a = Strings()
    a.empty_string = 'test'
    assert(a.empty_string == 'test')
    assert_raises(AssertionError, setattr, a, 'empty_string', 1234)
    a.ub_string = 'a' * 22
    assert(a.ub_string == 'a' * 22)
    assert_raises(AssertionError, setattr, a, 'ub_string', 'a' * 23)

    b = Nested()
    primitives = Primitives()
    b.primitives = primitives
    assert(b.primitives == primitives)
    assert_raises(AssertionError, setattr, b, 'primitives', 'foo')

    list_of_primitives = [primitives, primitives]
    tuple_of_primitives = (primitives, primitives)
    b.two_primitives = list_of_primitives
    assert(b.two_primitives == list_of_primitives)
    assert(type(b.two_primitives) == list)
    b.two_primitives = tuple_of_primitives
    assert(b.two_primitives == tuple_of_primitives)
    assert(type(b.two_primitives) == tuple)
    assert_raises(AssertionError, setattr, b, 'two_primitives', Primitives())
    assert_raises(AssertionError, setattr, b, 'two_primitives', [Primitives()])
    assert_raises(AssertionError, setattr, b, 'two_primitives',
                  [primitives, primitives, primitives])

    b.up_to_three_primitives = []
    assert(b.up_to_three_primitives == [])
    b.up_to_three_primitives = [primitives]
    assert(b.up_to_three_primitives == [primitives])
    b.up_to_three_primitives = [primitives, primitives]
    assert(b.up_to_three_primitives == [primitives, primitives])
    b.up_to_three_primitives = [primitives, primitives, primitives]
    assert(b.up_to_three_primitives == [primitives, primitives, primitives])
    assert_raises(AssertionError, setattr, b, 'up_to_three_primitives',
                  [primitives, primitives, primitives, primitives])

    b.unbounded_primitives = [primitives, primitives]
    assert(b.unbounded_primitives == [primitives, primitives])

    c = Various()
    c.byte_value = b'a'
    assert(c.byte_value == b'a')
    assert(c.byte_value != 'a')
    assert_raises(AssertionError, setattr, c, 'byte_value', 'a')
    assert_raises(AssertionError, setattr, c, 'byte_value', b'abc')
    assert_raises(AssertionError, setattr, c, 'byte_value', 'abc')

    c.char_value = 'a'
    assert(c.char_value == 'a')
    assert(c.char_value != b'a')
    assert_raises(AssertionError, setattr, c, 'char_value', b'a')
    assert_raises(AssertionError, setattr, c, 'char_value', 'abc')
    assert_raises(AssertionError, setattr, c, 'char_value', b'abc')

    c.up_to_three_int32_values = []
    assert(c.up_to_three_int32_values == [])
    c.up_to_three_int32_values = [12345, -12345]
    assert(c.up_to_three_int32_values == [12345, -12345])
    c.up_to_three_int32_values = [12345, -12345, 6789]
    assert(c.up_to_three_int32_values == [12345, -12345, 6789])
    c.up_to_three_int32_values = [12345, -12345, 6789]
    assert_raises(
        AssertionError, setattr, c, 'up_to_three_int32_values', [12345, -12345, 6789, -6789])

    c.up_to_three_string_values = []
    assert([] == c.up_to_three_string_values)
    c.up_to_three_string_values = ['foo', 'bar']
    assert(['foo', 'bar'] == c.up_to_three_string_values)
    c.up_to_three_string_values = ['foo', 'bar', 'baz']
    assert(['foo', 'bar', 'baz'] == c.up_to_three_string_values)
    assert_raises(
        AssertionError, setattr, c, 'up_to_three_string_values', ['foo', 'bar', 'baz', 'hello'])
