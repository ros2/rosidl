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
from rosidl_generator_py.msg import StringArrays
from rosidl_generator_py.msg import Strings
from rosidl_generator_py.msg import Various


def test_strings():
    a = Strings()

    assert a.empty_string is ''
    assert 'Hello world!' == a.def_string


def test_arrays_of_bounded_strings():
    a = StringArrays()
    array_valid_string_length = ['a' * 2, 'b' * 3, 'c' * 4]
    array_too_long_strings = ['a' * 2, 'b' * 3, 'c' * 6]
    assert ['', '', ''] == a.ub_string_static_array_value
    a.ub_string_static_array_value = array_valid_string_length
    assert array_valid_string_length == a.ub_string_static_array_value

    assert_raises(
        AssertionError, setattr, a,
        'ub_string_static_array_value', array_too_long_strings)

    assert_raises(
        AssertionError, setattr, a,
        'ub_string_static_array_value', ['a' * 2, 'b' * 3])

    assert [] == a.ub_string_ub_array_value
    a.ub_string_ub_array_value = array_valid_string_length
    assert array_valid_string_length == a.ub_string_ub_array_value

    assert_raises(
        AssertionError, setattr, a,
        'ub_string_ub_array_value', array_too_long_strings)

    array10strings = [] + [str(i) for i in range(10)]
    a.ub_string_ub_array_value = array10strings
    assert array10strings == a.ub_string_ub_array_value

    assert_raises(
        AssertionError, setattr, a,
        'ub_string_ub_array_value', array10strings + ['gfg'])

    assert [] == a.ub_string_dynamic_array_value
    a.ub_string_dynamic_array_value = array_valid_string_length
    assert array_valid_string_length == a.ub_string_dynamic_array_value

    assert_raises(
        AssertionError, setattr, a,
        'ub_string_dynamic_array_value', array_too_long_strings)

    array10strings = [] + [str(i) for i in range(10)]
    a.ub_string_dynamic_array_value = array10strings
    assert array10strings == a.ub_string_dynamic_array_value
    array10strings += ['gfg']
    a.ub_string_dynamic_array_value = array10strings
    assert array10strings == a.ub_string_dynamic_array_value


def test_invalid_attribute():
    a = Strings()

    assert_raises(AttributeError, setattr, a, 'invalid_string1', 'foo')

    assert_raises(AttributeError, getattr, a, 'invalid_string2')


def test_constructor():
    a = Strings(empty_string='foo')

    assert'foo' == a.empty_string

    assert_raises(AssertionError, Strings, unknown_field='test')


def test_constants():
    assert 123 == Constants.X
    assert -123 == Constants.Y
    assert 'foo' == Constants.FOO
    assert '\x7F' == Constants.TOTO
    assert b'0' == Constants.TATA

    assert_raises(AttributeError, setattr, Constants, 'FOO', 'bar')


def test_default_values():
    a = Strings()

    assert a.empty_string is ''
    assert 'Hello world!' == a.def_string
    a.def_string = 'Bye world'
    assert 'Bye world' == a.def_string
    assert 'Hello world!' == Strings.DEF_STRING__DEFAULT
    assert 'Hello world!' == a.DEF_STRING__DEFAULT

    assert 'Hello\'world!' == a.DEF_STRING2__DEFAULT
    assert 'Hello"world!' == a.DEF_STRING3__DEFAULT
    assert 'Hello\'world!' == a.DEF_STRING4__DEFAULT
    assert 'Hello"world!' == a.DEF_STRING5__DEFAULT
    assert_raises(AttributeError, setattr, Strings, 'DEF_STRING__DEFAULT', 'bar')

    b = StringArrays()
    assert ['What', 'a', 'wonderful', 'world', '!'] == b.DEF_STRING_DYNAMIC_ARRAY_VALUE__DEFAULT
    assert ['Hello', 'World', '!'] == b.DEF_STRING_STATIC_ARRAY_VALUE__DEFAULT
    assert ['Hello', 'World', '!'] == b.DEF_STRING_BOUNDED_ARRAY_VALUE__DEFAULT

    assert ['H"el\'lo', 'Wo\'r"ld'] == b.DEF_VARIOUS_QUOTES__DEFAULT
    assert ['Hel,lo', ',World', 'abcd', '!,'] == b.DEF_VARIOUS_COMMAS__DEFAULT

    c = Various()
    assert [5, 23] == c.TWO_UINT16_VALUE__DEFAULT

    assert [5, 23] == c.UP_TO_THREE_INT32_VALUES_WITH_DEFAULT_VALUES__DEFAULT

    assert '\x01' == c.CHAR_VALUE__DEFAULT
    assert '1' != c.CHAR_VALUE__DEFAULT
    assert b'\x01' == c.BYTE_VALUE__DEFAULT
    assert b'1' != c.BYTE_VALUE__DEFAULT


def test_check_constraints():
    a = Strings()
    a.empty_string = 'test'
    assert 'test' == a.empty_string
    assert_raises(AssertionError, setattr, a, 'empty_string', 1234)
    a.ub_string = 'a' * 22
    assert 'a' * 22 == a.ub_string
    assert_raises(AssertionError, setattr, a, 'ub_string', 'a' * 23)

    b = Nested()
    primitives = Primitives()
    b.primitives = primitives
    assert b.primitives == primitives
    assert_raises(AssertionError, setattr, b, 'primitives', 'foo')

    list_of_primitives = [primitives, primitives]
    tuple_of_primitives = (primitives, primitives)
    b.two_primitives = list_of_primitives
    assert b.two_primitives == list_of_primitives
    assert type(b.two_primitives) == list
    b.two_primitives = tuple_of_primitives
    assert b.two_primitives == tuple_of_primitives
    assert type(b.two_primitives) == tuple
    assert_raises(AssertionError, setattr, b, 'two_primitives', Primitives())
    assert_raises(AssertionError, setattr, b, 'two_primitives', [Primitives()])
    assert_raises(AssertionError, setattr, b, 'two_primitives',
                  [primitives, primitives, primitives])

    b.up_to_three_primitives = []
    assert [] == b.up_to_three_primitives
    b.up_to_three_primitives = [primitives]
    assert [primitives] == b.up_to_three_primitives
    b.up_to_three_primitives = [primitives, primitives]
    assert [primitives, primitives] == b.up_to_three_primitives
    b.up_to_three_primitives = [primitives, primitives, primitives]
    assert [primitives, primitives, primitives] == b.up_to_three_primitives
    assert_raises(AssertionError, setattr, b, 'up_to_three_primitives',
                  [primitives, primitives, primitives, primitives])

    b.unbounded_primitives = [primitives, primitives]
    assert [primitives, primitives] == b.unbounded_primitives

    c = Various()
    c.byte_value = b'a'
    assert b'a' == c.byte_value
    assert 'a' != c.byte_value
    assert_raises(AssertionError, setattr, c, 'byte_value', 'a')
    assert_raises(AssertionError, setattr, c, 'byte_value', b'abc')
    assert_raises(AssertionError, setattr, c, 'byte_value', 'abc')

    c.char_value = 'a'
    assert 'a' == c.char_value
    assert b'a' != c.char_value
    assert_raises(AssertionError, setattr, c, 'char_value', b'a')
    assert_raises(AssertionError, setattr, c, 'char_value', 'abc')
    assert_raises(AssertionError, setattr, c, 'char_value', b'abc')

    c.up_to_three_int32_values = []
    assert [] == c.up_to_three_int32_values
    c.up_to_three_int32_values = [12345, -12345]
    assert [12345, -12345] == c.up_to_three_int32_values
    c.up_to_three_int32_values = [12345, -12345, 6789]
    assert [12345, -12345, 6789] == c.up_to_three_int32_values
    c.up_to_three_int32_values = [12345, -12345, 6789]
    assert_raises(
        AssertionError, setattr, c, 'up_to_three_int32_values', [12345, -12345, 6789, -6789])

    c.up_to_three_string_values = []
    assert [] == c.up_to_three_string_values
    c.up_to_three_string_values = ['foo', 'bar']
    assert ['foo', 'bar'] == c.up_to_three_string_values
    c.up_to_three_string_values = ['foo', 'bar', 'baz']
    assert ['foo', 'bar', 'baz'] == c.up_to_three_string_values
    assert_raises(
        AssertionError, setattr, c, 'up_to_three_string_values', ['foo', 'bar', 'baz', 'hello'])


def test_out_of_range():
    a = Primitives()
    assert_raises(
        AssertionError, setattr, a, 'char_value', '\x80')
    for i in [8, 16, 32, 64]:
        assert_raises(
            AssertionError, setattr, a, 'int%d_value' % i, 2**(i - 1))
        assert_raises(
            AssertionError, setattr, a, 'int%d_value' % i, -2**(i - 1) - 1)
        assert_raises(
            AssertionError, setattr, a, 'uint%d_value' % i, -1)
        assert_raises(
            AssertionError, setattr, a, 'int%d_value' % i, 2**i)

    b = Various()
    assert_raises(
        AssertionError, setattr, b, 'two_uint16_value', [2**16])
    assert_raises(
        AssertionError, setattr, b, 'two_uint16_value', [-1])

    assert_raises(
        AssertionError, setattr, b, 'up_to_three_int32_values', [2**31])
    assert_raises(
        AssertionError, setattr, b, 'up_to_three_int32_values', [-2**31 - 1])

    assert_raises(
        AssertionError, setattr, b, 'unbounded_uint64_values', [2**64])
    assert_raises(
        AssertionError, setattr, b, 'unbounded_uint64_values', [-1])
