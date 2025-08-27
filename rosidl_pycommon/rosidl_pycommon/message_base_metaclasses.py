# Copyright 2025 Open Source Robotics Foundation, Inc.
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

# Base Metaclasses
from abc import ABCMeta
from abc import abstractmethod
from collections.abc import MutableMapping
from typing import Any
from typing import ClassVar
from typing import Generic
from typing import TypeVar


class MessageAbstractMeta(ABCMeta):

    _CREATE_ROS_MESSAGE: ClassVar[Any]
    _CONVERT_FROM_PY: ClassVar[Any]
    _CONVERT_TO_PY: ClassVar[Any]
    _DESTROY_ROS_MESSAGE: ClassVar[Any]
    _TYPE_SUPPORT: ClassVar[Any]

    @classmethod
    @abstractmethod
    def __import_type_support__(cls) -> None: ...

    @classmethod
    @abstractmethod
    def __prepare__(metacls, name: str, bases: tuple[type[Any], ...], /, **kwds: Any
                    ) -> MutableMapping[str, object]: ...


RequestT = TypeVar('RequestT')
ResponseT = TypeVar('ResponseT')


class ServiceAbstractMeta(ABCMeta, Generic[RequestT, ResponseT]):

    _TYPE_SUPPORT: ClassVar[Any]

    @classmethod
    @abstractmethod
    def __import_type_support__(cls) -> None: ...


GoalT = TypeVar('GoalT')
ResultT = TypeVar('ResultT')
FeedbackT = TypeVar('FeedbackT')


class ActionAbstractMeta(ABCMeta, Generic[GoalT, ResultT, FeedbackT]):

    _TYPE_SUPPORT: ClassVar[Any]

    @classmethod
    @abstractmethod
    def __import_type_support__(cls) -> None: ...