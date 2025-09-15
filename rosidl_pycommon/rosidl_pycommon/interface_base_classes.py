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
# limitations under the License.

# Base Classes
from abc import ABC
from abc import ABCMeta
from abc import abstractmethod
from typing import Any
from typing import ClassVar
from typing import Generic
from typing import TypeVar


class MessageTypeSupportMeta(ABCMeta):

    _CREATE_ROS_MESSAGE: ClassVar[Any]
    _CONVERT_FROM_PY: ClassVar[Any]
    _CONVERT_TO_PY: ClassVar[Any]
    _DESTROY_ROS_MESSAGE: ClassVar[Any]
    _TYPE_SUPPORT: ClassVar[Any]

    @classmethod
    @abstractmethod
    def __import_type_support__(cls) -> None: ...


class BaseMessage(ABC, metaclass=MessageTypeSupportMeta):

    __slots__ = ()

    @abstractmethod
    def __repr__(self) -> str: ...

    @abstractmethod
    def __eq__(self, other: object) -> bool: ...

    @classmethod
    @abstractmethod
    def get_fields_and_field_types(cls) -> dict[str, str]: ...


RequestT = TypeVar('RequestT')
ResponseT = TypeVar('ResponseT')


class ServiceTypeSupportMeta(ABCMeta):

    _TYPE_SUPPORT: ClassVar[Any]

    @classmethod
    @abstractmethod
    def __import_type_support__(cls) -> None: ...


class BaseService(ABC, Generic[RequestT, ResponseT], metaclass=ServiceTypeSupportMeta):

    Request: type[RequestT]
    Response: type[ResponseT]


GoalT = TypeVar('GoalT')
ResultT = TypeVar('ResultT')
FeedbackT = TypeVar('FeedbackT')


class ActionTypeSupportMeta(ABCMeta):

    _TYPE_SUPPORT: ClassVar[Any]

    @classmethod
    @abstractmethod
    def __import_type_support__(cls) -> None: ...


class BaseAction(ABC, Generic[GoalT, ResultT, FeedbackT], metaclass=ActionTypeSupportMeta):

    Goal: type[GoalT]
    Result: type[ResultT]
    Feedback: type[FeedbackT]
