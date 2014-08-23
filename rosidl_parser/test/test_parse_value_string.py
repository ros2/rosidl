from nose.tools import assert_raises

from rosidl_parser import *


def test_parse_value_string_primitive():
    parse_value_string(Type('bool'), '1')


def test_parse_value_string():
    with assert_raises(InvalidValue):
        parse_value_string(Type('bool[]'), '1')

    with assert_raises(InvalidValue):
        parse_value_string(Type('bool[2]'), '[1]')
    with assert_raises(InvalidValue):
        parse_value_string(Type('bool[<=1]'), '[1, 0]')

    with assert_raises(InvalidValue):
        parse_value_string(Type('bool[]'), '[2]')

    value = parse_value_string(Type('bool[]'), '[1]')
    assert value


def test_parse_value_string_not_implemented():
    with assert_raises(NotImplementedError):
        parse_value_string(Type('string[]'), '[foo, bar]')

    with assert_raises(NotImplementedError):
        parse_value_string(Type('pkg/Foo[]'), '')
