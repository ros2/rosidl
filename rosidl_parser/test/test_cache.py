# Copyright 2026 Open Source Robotics Foundation, Inc.
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

import os
import pathlib
import time

import pytest

import rosidl_parser.cache as cache_module
from rosidl_parser.cache import cleanup_cache_if_needed
from rosidl_parser.cache import compute_cache_key
from rosidl_parser.cache import get_cache_config
from rosidl_parser.cache import restore_files_from_cache
from rosidl_parser.cache import restore_object_from_cache
from rosidl_parser.cache import save_files_to_cache
from rosidl_parser.cache import save_object_to_cache


@pytest.fixture(autouse=True)
def reset_cache_config():
    """Reset the cached config and env vars before each test."""
    cache_module._cache_config = None
    saved_env = {}
    for key in ('ROSIDL_CACHE_DIR', 'ROSIDL_CACHE_DEBUG',
                'ROSIDL_CACHE_MAX_SIZE', 'ROSIDL_CACHE_CONFIG'):
        saved_env[key] = os.environ.pop(key, None)
    yield
    cache_module._cache_config = None
    for key, val in saved_env.items():
        if val is None:
            os.environ.pop(key, None)
        else:
            os.environ[key] = val


@pytest.fixture
def cache_dir(tmp_path):
    """Provide a temporary cache directory and set env vars."""
    d = tmp_path / 'cache'
    d.mkdir()
    os.environ['ROSIDL_CACHE_DIR'] = str(d)
    yield d


def test_config_defaults():
    config = get_cache_config()
    assert config['cache_dir'] is None
    assert config['cache_debug'] is False
    assert config['cache_max_size'] == 1073741824


def test_config_env_override(tmp_path):
    os.environ['ROSIDL_CACHE_DIR'] = str(tmp_path)
    os.environ['ROSIDL_CACHE_DEBUG'] = '1'
    os.environ['ROSIDL_CACHE_MAX_SIZE'] = '500'
    config = get_cache_config()
    assert config['cache_dir'] == str(tmp_path)
    assert config['cache_debug'] is True
    assert config['cache_max_size'] == 500


def test_cache_key_none_without_cache_dir():
    assert compute_cache_key('hello') is None


def test_cache_key_deterministic(cache_dir):
    key1 = compute_cache_key('hello', 'world')
    cache_module._cache_config = None
    key2 = compute_cache_key('hello', 'world')
    assert key1 == key2


def test_object_cache_round_trip(cache_dir):
    obj = {'key': 'value', 'list': [1, 2, 3]}
    save_object_to_cache('test_key', 'sub', obj)
    assert restore_object_from_cache('test_key', 'sub') == obj


def test_object_cache_miss(cache_dir):
    assert restore_object_from_cache('nonexistent', 'sub') is None


def test_files_cache_round_trip(cache_dir, tmp_path):
    src_dir = tmp_path / 'src'
    src_dir.mkdir()
    (src_dir / 'a.txt').write_text('aaa')
    (src_dir / 'sub').mkdir()
    (src_dir / 'sub' / 'b.txt').write_text('bbb')

    save_files_to_cache('fkey', 'fsub', ['a.txt', 'sub/b.txt'], str(src_dir))

    out_dir = tmp_path / 'out'
    out_dir.mkdir()
    result = restore_files_from_cache('fkey', 'fsub', str(out_dir))
    assert result is not None
    assert (out_dir / 'a.txt').read_text() == 'aaa'
    assert (out_dir / 'sub' / 'b.txt').read_text() == 'bbb'


def test_save_error_cleans_up(cache_dir, tmp_path):
    """Saving with a nonexistent source file should not leave partial cache."""
    save_files_to_cache('bad_key', 'fsub', ['nonexistent.txt'], str(tmp_path))
    assert restore_files_from_cache('bad_key', 'fsub', str(tmp_path)) is None


def test_cleanup_removes_oldest(cache_dir):
    os.environ['ROSIDL_CACHE_MAX_SIZE'] = '100'
    cache_module._cache_config = None

    sub_dir = cache_dir / 'sub'
    sub_dir.mkdir()

    old_entry = sub_dir / 'old_key'
    old_entry.mkdir()
    (old_entry / 'data.bin').write_bytes(b'x' * 60)

    time.sleep(0.05)

    new_entry = sub_dir / 'new_key'
    new_entry.mkdir()
    (new_entry / 'data.bin').write_bytes(b'x' * 60)

    # Total 120 > 100, should remove oldest
    cleanup_cache_if_needed('sub')

    assert not old_entry.exists()
    assert new_entry.exists()


def test_idl_file_cache_round_trip(cache_dir):
    from rosidl_parser.definition import (
        Array,
        BasicType,
        BoundedSequence,
        BoundedString,
        Constant,
        IdlContent,
        IdlFile,
        IdlLocator,
        Include,
        Member,
        Message,
        NamespacedType,
        Structure,
        UnboundedSequence,
        UnboundedString,
    )

    locator = IdlLocator(pathlib.Path('/base'), pathlib.Path('msg/Test.idl'))
    content = IdlContent()

    ns_type = NamespacedType(['test_pkg', 'msg'], 'TestMessage')
    structure = Structure(ns_type, members=[
        Member(BasicType('int32'), 'field_a'),
        Member(BasicType('boolean'), 'field_b'),
        Member(UnboundedString(), 'field_c'),
        Member(BoundedString(100), 'field_d'),
        Member(Array(BasicType('double'), 5), 'field_e'),
        Member(BoundedSequence(BasicType('uint8'), 10), 'field_f'),
        Member(UnboundedSequence(BasicType('int64')), 'field_g'),
        Member(NamespacedType(['other_pkg', 'msg'], 'OtherType'), 'field_h'),
    ])
    msg = Message(structure)
    msg.constants = [
        Constant('MY_CONST', BasicType('int32'), 42),
    ]

    content.elements.append(Include('other_pkg/msg/OtherType.idl'))
    content.elements.append(msg)

    idl_file = IdlFile(locator, content)

    save_object_to_cache('idl_key', 'sub', idl_file)
    restored = restore_object_from_cache('idl_key', 'sub')

    assert restored is not None
    assert isinstance(restored, IdlFile)
    assert str(restored.locator.basepath) == '/base'
    assert str(restored.locator.relative_path) == 'msg/Test.idl'
    assert len(restored.content.elements) == 2
    assert isinstance(restored.content.elements[0], Include)
    assert restored.content.elements[0].locator == 'other_pkg/msg/OtherType.idl'
    msg_r = restored.content.elements[1]
    assert isinstance(msg_r, Message)
    assert msg_r.structure.namespaced_type.name == 'TestMessage'
    assert len(msg_r.structure.members) == 8
    assert isinstance(msg_r.structure.members[0].type, BasicType)
    assert msg_r.structure.members[0].type.typename == 'int32'
    assert isinstance(msg_r.structure.members[4].type, Array)
    assert msg_r.structure.members[4].type.size == 5
    assert isinstance(msg_r.structure.members[5].type, BoundedSequence)
    assert msg_r.structure.members[5].type.maximum_size == 10
    assert isinstance(msg_r.structure.members[6].type, UnboundedSequence)
    assert len(msg_r.constants) == 1
    assert msg_r.constants[0].name == 'MY_CONST'
    assert msg_r.constants[0].value == 42
