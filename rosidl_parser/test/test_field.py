from nose.tools import assert_raises

from rosidl_parser import Field
from rosidl_parser import Type


def test_field_constructor():
    type_ = Type('bool')
    field = Field(type_, 'foo')
    assert field.type == type_
    assert field.name == 'foo'
    assert field.default_value is None

    field = Field(type_, 'foo', '1')
    assert field.default_value

    with assert_raises(TypeError):
        Field('type', 'foo')

    with assert_raises(NameError):
        Field(type_, 'foo bar')


def test_field_methods():
    assert Field(Type('bool'), 'foo') != 23

    assert (Field(Type('bool'), 'foo', '1') ==
            Field(Type('bool'), 'foo', 'true'))
    assert (Field(Type('bool'), 'foo', '1') !=
            Field(Type('bool'), 'foo', 'false'))
    assert (Field(Type('bool'), 'foo', '1') !=
            Field(Type('bool'), 'bar', '1'))
    assert (Field(Type('bool'), 'foo', '1') !=
            Field(Type('byte'), 'foo', '1'))

    assert str(Field(Type('bool'), 'foo', '1')) == 'bool foo True'

    assert str(Field(Type('string<=5'), 'foo', 'value')) == \
        "string<=5 foo 'value'"
