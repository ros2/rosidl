from nose.tools import assert_raises

from rosidl_parser import BaseType
from rosidl_parser import Field
from rosidl_parser import MessageSpecification
from rosidl_parser import Type
from rosidl_parser import UnknownMessageType
from rosidl_parser import validate_field_types


def test_validate_field_types():
    msg_spec = MessageSpecification('pkg', 'Foo', [], [])
    known_msg_type = []
    validate_field_types(msg_spec, known_msg_type)

    msg_spec.fields.append(Field(Type('bool'), 'foo'))
    validate_field_types(msg_spec, known_msg_type)

    msg_spec.fields.append(Field(Type('pkg/Bar'), 'bar'))
    with assert_raises(UnknownMessageType):
        validate_field_types(msg_spec, known_msg_type)

    known_msg_type.append(BaseType('pkg/Bar'))
    validate_field_types(msg_spec, known_msg_type)
