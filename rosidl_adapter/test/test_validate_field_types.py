# Copyright 2014 Open Source Robotics Foundation, Inc.
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

from typing import List

import pytest
from rosidl_adapter.parser import BaseType
from rosidl_adapter.parser import Field
from rosidl_adapter.parser import MessageSpecification
from rosidl_adapter.parser import Type
from rosidl_adapter.parser import UnknownMessageType
from rosidl_adapter.parser import validate_field_types


def test_validate_field_types() -> None:
    msg_spec = MessageSpecification('pkg', 'Foo', [], [])
    known_msg_type: List[BaseType] = []
    validate_field_types(msg_spec, known_msg_type)

    msg_spec.fields.append(Field(Type('bool'), 'foo'))
    validate_field_types(msg_spec, known_msg_type)

    msg_spec.fields.append(Field(Type('pkg/Bar'), 'bar'))
    with pytest.raises(UnknownMessageType):
        validate_field_types(msg_spec, known_msg_type)

    known_msg_type.append(BaseType('pkg/Bar'))
    validate_field_types(msg_spec, known_msg_type)
