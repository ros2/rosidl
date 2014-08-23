from nose.tools import assert_raises

from rosidl_parser import *


def test_constant_constructor():
    value = Constant('bool', 'FOO', '1')
    assert value

    with assert_raises(TypeError):
        Constant('pkg/Foo', 'FOO', '')

    with assert_raises(NameError):
        Constant('bool', 'FOO BAR', '')

    with assert_raises(ValueError):
        Constant('bool', 'FOO', None)


def test_constant_methods():
    assert Constant('bool', 'FOO', '1') != 23

    assert Constant('bool', 'FOO', '1') == Constant('bool', 'FOO', '1')
    assert Constant('bool', 'FOO', '1') != Constant('bool', 'FOO', '0')
    assert Constant('bool', 'FOO', '1') != Constant('bool', 'BAR', '1')
    assert Constant('bool', 'FOO', '1') != Constant('byte', 'FOO', '1')

    assert str(Constant('bool', 'FOO', '1')) == 'bool FOO=True'

    assert str(Constant('string', 'FOO', 'foo')) == "string FOO='foo'"
