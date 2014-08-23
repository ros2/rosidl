from nose.tools import assert_raises

from rosidl_parser import *


def test_is_valid_package_name():
    for valid_package_name in [
            'foo']:
        assert is_valid_package_name(valid_package_name)
    for invalid_package_name in [
            'foo-bar']:
        assert not is_valid_package_name(invalid_package_name)
    with assert_raises(InvalidResourceName):
        is_valid_package_name(None)


def test_is_valid_field_name():
    for valid_field_name in [
            'foo']:
        is_valid_field_name(valid_field_name)
    for invalid_field_name in [
            'foo-bar']:
        assert not is_valid_field_name(invalid_field_name)
    with assert_raises(InvalidResourceName):
        is_valid_field_name(None)


def test_is_valid_message_name():
    for valid_message_name in [
            'Foo']:
        assert is_valid_message_name(valid_message_name)
    for invalid_message_name in [
            '0foo']:
        assert not is_valid_message_name(invalid_message_name)
    with assert_raises(InvalidResourceName):
        is_valid_message_name(None)


def test_is_valid_constant_name():
    for valid_constant_name in [
            'FOO']:
        assert is_valid_constant_name(valid_constant_name)
    for invalid_constant_name in [
            'Foo']:
        assert not is_valid_constant_name(invalid_constant_name)
    with assert_raises(InvalidResourceName):
        is_valid_constant_name(None)
