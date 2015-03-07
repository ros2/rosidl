from nose.tools import assert_raises

from rosidl_parser import Type


def test_type_constructor():
    type_ = Type('bool')
    assert type_.pkg_name is None
    assert type_.type == 'bool'
    assert type_.string_upper_bound is None
    assert not type_.is_array
    assert type_.array_size is None
    assert not type_.is_upper_bound

    type_ = Type('bool[]')
    assert type_.pkg_name is None
    assert type_.type == 'bool'
    assert type_.string_upper_bound is None
    assert type_.is_array
    assert type_.array_size is None
    assert not type_.is_upper_bound

    with assert_raises(TypeError):
        Type('bool]')

    type_ = Type('bool[5]')
    assert type_.pkg_name is None
    assert type_.type == 'bool'
    assert type_.string_upper_bound is None
    assert type_.is_array
    assert type_.array_size == 5
    assert not type_.is_upper_bound

    type_ = Type('bool[<=5]')
    assert type_.pkg_name is None
    assert type_.type == 'bool'
    assert type_.string_upper_bound is None
    assert type_.is_array
    assert type_.array_size == 5
    assert type_.is_upper_bound

    with assert_raises(TypeError):
        Type('bool[size]')
    with assert_raises(TypeError):
        Type('bool[0]')
    with assert_raises(TypeError):
        Type('bool[<=size]')
    with assert_raises(TypeError):
        Type('bool[<=0]')


def test_type_methods():
    assert Type('bool[5]') != 23

    assert Type('pkg/Foo') == Type('pkg/Foo')
    assert Type('pkg/Foo[]') == Type('pkg/Foo[]')
    assert Type('pkg/Foo[5]') == Type('pkg/Foo[5]')
    assert Type('pkg/Foo[<=5]') == Type('pkg/Foo[<=5]')

    assert Type('bool') != Type('pkg/Foo')
    assert Type('pkg/Foo[]') != Type('pkg/Foo[5]')
    assert Type('pkg/Foo[5]') != Type('pkg/Foo[<=5]')
    assert Type('pkg/Foo[<=5]') != Type('pkg/Foo[<=23]')

    {Type('pkg/Foo[5]'): None}

    assert str(Type('pkg/Foo[]')) == 'pkg/Foo[]'
    assert str(Type('pkg/Foo[5]')) == 'pkg/Foo[5]'
    assert str(Type('pkg/Foo[<=5]')) == 'pkg/Foo[<=5]'
