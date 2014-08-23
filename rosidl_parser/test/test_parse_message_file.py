from nose.tools import assert_raises
import os
import shutil
import tempfile

from rosidl_parser import *


def test_parse_message_file():
    path = tempfile.mkdtemp(prefix='test_parse_message_file_')
    try:
        filename = os.path.join(path, 'Foo.msg')
        with open(filename, 'w') as handle:
            handle.write('bool foo')
        msg_spec = parse_message_file('pkg', filename)

        assert len(msg_spec.fields) == 1
        assert msg_spec.fields[0].type.type == 'bool'
        assert msg_spec.fields[0].name == 'foo'
        assert msg_spec.fields[0].default_value is None
        assert len(msg_spec.constants) == 0

        with open(filename, 'a') as handle:
            handle.write('\nbool foo')
        with assert_raises(ValueError) as ctx:
            parse_message_file('pkg', filename)
        assert 'foo' in str(ctx.exception)
    finally:
        shutil.rmtree(path)
