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

from array import array
import gc

import _rosidl_buffer_test_utils as native
import pytest
import rosidl_buffer
from rosidl_buffer import _get_buffer_ptr
from rosidl_buffer import _share_buffer_from_capsule as share
from rosidl_buffer import _take_buffer_from_ptr
from rosidl_buffer import Buffer


def test_contents_and_shared_control_block():
    owner = native.Owner()
    capsule = owner.capsule()
    assert owner.use_count == 2
    first = share(capsule)
    second = share(capsule)
    assert first is not second
    assert owner.use_count == 4
    for buffer in (first, second):
        assert isinstance(buffer, Buffer)
        assert _get_buffer_ptr(buffer) == owner.pointer
        assert buffer.backend_type == 'ownership_test'
        assert len(buffer) == 4
        assert buffer.to_bytes() == bytes([0, 1, 127, 255])
        assert buffer.to_array() == array('B', [0, 1, 127, 255])
        assert repr(buffer) == "Buffer(size=4, backend='ownership_test')"
    assert owner.destructions == 0
    assert '_share_buffer_from_capsule' not in rosidl_buffer.__all__


def test_capsule_destroyed_before_wrappers():
    owner = native.Owner()
    capsule = owner.capsule()
    first, second = share(capsule), share(capsule)
    owner.reset()
    del capsule
    gc.collect()
    assert owner.destructions == 0
    assert first.to_bytes() == second.to_bytes() == bytes([0, 1, 127, 255])
    del first
    gc.collect()
    assert owner.destructions == 0
    assert second.to_bytes() == bytes([0, 1, 127, 255])
    del second
    gc.collect()
    assert owner.destructions == 1


def test_wrappers_destroyed_before_other_owners():
    owner = native.Owner()
    capsule = owner.capsule()
    buffer = share(capsule)
    del buffer
    gc.collect()
    assert owner.use_count == 2
    assert owner.destructions == 0
    # The holder was not consumed; it can still create another wrapper.
    buffer = share(capsule)
    assert owner.use_count == 3
    del capsule, buffer
    gc.collect()
    assert owner.use_count == 1
    assert owner.destructions == 0
    owner.reset()
    assert owner.destructions == 1


def test_capsule_as_last_owner():
    owner = native.Owner()
    capsule = owner.capsule()
    owner.reset()
    buffer = share(capsule)
    del buffer
    gc.collect()
    assert owner.destructions == 0
    del capsule
    gc.collect()
    assert owner.destructions == 1


@pytest.mark.parametrize('value', [None, 42, 'rosidl_buffer.shared_buffer', object()])
def test_reject_non_capsule(value):
    with pytest.raises(ValueError, match="capsule named 'rosidl_buffer.shared_buffer'"):
        share(value)


@pytest.mark.parametrize('factory', [
    native.wrong_name_capsule, native.versioned_capsule, native.unnamed_capsule,
])
def test_reject_wrong_name_before_accessing_holder(factory):
    with pytest.raises(ValueError, match="capsule named 'rosidl_buffer.shared_buffer'"):
        share(factory())


def test_reject_empty_shared_pointer():
    capsule = native.empty_capsule()
    for _ in range(2):
        with pytest.raises(ValueError, match='empty shared_ptr'):
            share(capsule)


def test_legacy_helpers():
    owner = native.Owner()
    owner.reset()
    assert owner.destructions == 1
    pointer = owner.legacy_pointer()
    buffer = _take_buffer_from_ptr(pointer)
    assert _get_buffer_ptr(buffer) == pointer
    assert buffer.to_bytes() == bytes([0, 1, 127, 255])
    assert buffer.backend_type == 'ownership_test'
    del buffer
    gc.collect()
    assert owner.destructions == 2
    with pytest.raises(TypeError):
        _get_buffer_ptr(object())
