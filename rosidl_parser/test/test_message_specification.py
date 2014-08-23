from nose.tools import assert_raises

from rosidl_parser import *


def test_message_specification_constructor():
    msg_spec = MessageSpecification('pkg', 'Foo', [], [])
    assert msg_spec.base_type.pkg_name == 'pkg'
    assert msg_spec.base_type.type == 'Foo'
    assert len(msg_spec.fields) == 0
    assert len(msg_spec.constants) == 0

    with assert_raises(TypeError):
        MessageSpecification('pkg', 'Foo', None, [])
    with assert_raises(TypeError):
        MessageSpecification('pkg', 'Foo', [], None)
    with assert_raises(TypeError):
        MessageSpecification('pkg', 'Foo', ['field'], [])
    with assert_raises(TypeError):
        MessageSpecification('pkg', 'Foo', [], ['constant'])

    field = Field(Type('bool'), 'foo', '1')
    constant = Constant('bool', 'BAR', '1')
    msg_spec = MessageSpecification('pkg', 'Foo', [field], [constant])
    assert len(msg_spec.fields) == 1
    assert msg_spec.fields[0] == field
    assert len(msg_spec.constants) == 1
    assert msg_spec.constants[0] == constant

    with assert_raises(ValueError):
        MessageSpecification('pkg', 'Foo', [field, field], [])
    with assert_raises(ValueError):
        MessageSpecification('pkg', 'Foo', [], [constant, constant])


def test_message_specification_methods():
    field = Field(Type('bool'), 'foo', '1')
    constant = Constant('bool', 'BAR', '1')
    msg_spec = MessageSpecification('pkg', 'Foo', [field], [constant])

    assert msg_spec != 'spec'
    assert msg_spec == MessageSpecification('pkg', 'Foo', [field], [constant])
    assert msg_spec != MessageSpecification('pkg', 'Bar', [], [])
    assert msg_spec != MessageSpecification('pkg', 'Foo', [field], [])
    assert msg_spec != MessageSpecification('pkg', 'Foo', [], [constant])
