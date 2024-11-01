# Copyright 2018 Open Source Robotics Foundation, Inc.
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

import pathlib
from typing import Dict
from typing import Final
from typing import Iterable
from typing import List
from typing import Literal
from typing import Optional
from typing import Set
from typing import Tuple
from typing import Type
from typing import TYPE_CHECKING
from typing import TypeVar
from typing import Union

# Basic types as defined by the IDL specification

# 7.4.1.4.4.2 Basic Types
SIGNED_NONEXPLICIT_INTEGER_TYPES: Final = (  # rules (26)
    'short',  # rule (27)
    'long',  # rule (28)
    'long long',  # rule (29)
)
UNSIGNED_NONEXPLICIT_INTEGER_TYPES: Final = (  # rules (30)
    'unsigned short',  # rule (31)
    'unsigned long',  # rule (32)
    'unsigned long long',  # rule (33)
)
NONEXPLICIT_INTEGER_TYPES: Final = (
    *SIGNED_NONEXPLICIT_INTEGER_TYPES,
    *UNSIGNED_NONEXPLICIT_INTEGER_TYPES,
)
FLOATING_POINT_TYPES: Final = (  # rule (24)
    'float',
    'double',
    'long double',
)
CHARACTER_TYPES: Final = (
    'char',  # rule (34)
    'wchar',  # rule (35)
)
BOOLEAN_TYPE: Final = 'boolean'  # rule (36)
OCTET_TYPE: Final = 'octet'  # rule (37)

# 7.4.13.4.4 Integers restricted to holding 8-bits of information
# 7.4.13.4.5 Explicitly-named Integer Types
SIGNED_EXPLICIT_INTEGER_TYPES: Final = (
    'int8',  # rule (208)
    'int16',  # rule (210)
    'int32',  # rule (211)
    'int64',  # rule (212)
)
UNSIGNED_EXPLICIT_INTEGER_TYPES: Final = (
    'uint8',  # rule (209)
    'uint16',  # rule (213)
    'uint32',  # rule (214)
    'uint64',  # rule (215)
)
EXPLICIT_INTEGER_TYPES: Final = (
    *SIGNED_EXPLICIT_INTEGER_TYPES,
    *UNSIGNED_EXPLICIT_INTEGER_TYPES,
)
SIGNED_INTEGER_TYPES: Final = (   # rules (26) + (208) + (210-212)
    *SIGNED_NONEXPLICIT_INTEGER_TYPES,
    *SIGNED_EXPLICIT_INTEGER_TYPES,
)
UNSIGNED_INTEGER_TYPES: Final = (   # rules (30) + (209) + (213-215)
    *UNSIGNED_NONEXPLICIT_INTEGER_TYPES,
    *UNSIGNED_EXPLICIT_INTEGER_TYPES,
)
INTEGER_TYPES: Final = (   # rules (25) + (206-207) + (210-215)
    *SIGNED_INTEGER_TYPES,
    *UNSIGNED_INTEGER_TYPES,
)

"""All basic types as defined by the IDL specification."""
BASIC_TYPES: Final = (
    *INTEGER_TYPES,
    *FLOATING_POINT_TYPES,
    *CHARACTER_TYPES,
    BOOLEAN_TYPE,
    OCTET_TYPE,
)

if TYPE_CHECKING:
    from typing_extensions import TypeAlias
    SignedNonexplicitIntegerTypeValues = Literal['short', 'long', 'long long']
    UnsignedNonexplicitIntegerTypeValues = Literal['unsigned short', 'unsigned long',
                                                   'unsigned long long']

    NonexplicitIntegerTypeValues = Union[SignedNonexplicitIntegerTypeValues,
                                         UnsignedNonexplicitIntegerTypeValues]

    FloatingPointTypeValues = Literal['float', 'double', 'long double']
    CharacterTypeValues = Literal['char', 'wchar']
    BooleanValue = Literal['boolean']
    OctetValue = Literal['octet']

    SignedExplicitIntegerTypeValues = Literal['int8', 'int16', 'int32', 'int64']
    UnsignedExplicitIntegerTypeValues = Literal['uint8', 'uint16', 'uint32', 'uint64']

    ExplicitIntegerTypeValues = Union[SignedExplicitIntegerTypeValues,
                                      UnsignedExplicitIntegerTypeValues]

    SignedIntegerTypeValues = Union[SignedNonexplicitIntegerTypeValues,
                                    SignedExplicitIntegerTypeValues]
    UnsignedIntegerTypeValues = Union[UnsignedNonexplicitIntegerTypeValues,
                                      UnsignedExplicitIntegerTypeValues]
    IntegerTypeValues = Union[SignedIntegerTypeValues, UnsignedIntegerTypeValues]

    BasicTypeValues = Union[IntegerTypeValues, FloatingPointTypeValues,
                            CharacterTypeValues, BooleanValue,
                            OctetValue]

EMPTY_STRUCTURE_REQUIRED_MEMBER_NAME: Final = 'structure_needs_at_least_one_member'

CONSTANT_MODULE_SUFFIX: Final = '_Constants'

SERVICE_REQUEST_MESSAGE_SUFFIX: Final = '_Request'
SERVICE_RESPONSE_MESSAGE_SUFFIX: Final = '_Response'
SERVICE_EVENT_MESSAGE_SUFFIX: Final = '_Event'

ACTION_GOAL_SUFFIX: Final = '_Goal'
ACTION_RESULT_SUFFIX: Final = '_Result'
ACTION_FEEDBACK_SUFFIX: Final = '_Feedback'

ACTION_GOAL_SERVICE_SUFFIX: Final = '_SendGoal'
ACTION_RESULT_SERVICE_SUFFIX: Final = '_GetResult'
ACTION_FEEDBACK_MESSAGE_SUFFIX: Final = '_FeedbackMessage'


class AbstractType:
    """The abstract base class for all types."""

    __slots__ = ()

    def __eq__(self, other: object) -> bool:
        return type(self) is type(other)


class AbstractNestableType(AbstractType):
    """
    The abstract base class for types which can be used inside nested types.

    Arrays and sequences are nested types which contain elements of nestable
    types.
    Nestable types are:
    - BasicType like numerics, character types, boolean and octet
    - NamedType identified by a name which hasn't been resolved yet
    - NamespacedType which describes another Structure
    - Strings with any kind of character types, bounded as well as unbounded
    """

    __slots__ = ()


class BasicType(AbstractNestableType):
    """
    A basic type according to the IDL specification.

    Its `typename` attribute defines the specific numeric, character, boolean
    or octet type.
    """

    __slots__ = ('typename', )

    def __init__(self, typename: 'BasicTypeValues') -> None:
        """
        Create a BasicType.

        :param typename: the name of the basic type
        """
        super().__init__()
        assert typename in BASIC_TYPES
        self.typename = typename

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, BasicType):
            return False
        return super().__eq__(other) and self.typename == other.typename


class NamedType(AbstractNestableType):
    """A type identified by the name."""

    __slots__ = ('name')

    def __init__(self, name: str) -> None:
        """
        Create a NamedType.

        :param name: the name
        """
        super().__init__()
        self.name = name

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, NamedType):
            return False
        return super().__eq__(other) and self.name == other.name


class NamespacedType(AbstractNestableType):
    """A type identified by a name in a namespaced scope."""

    __slots__ = ('namespaces', 'name')

    def __init__(self, namespaces: Iterable[str], name: str) -> None:
        """
        Create a NamespacedType.

        :param namespaces: the names of nested namespaces identifying a
          specific scope
        :param name: the name of the type within that scope
        """
        super().__init__()
        self.namespaces = namespaces
        self.name = name

    def namespaced_name(self) -> Tuple[str, ...]:
        return (*self.namespaces, self.name)

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, NamespacedType):
            return False
        return super().__eq__(other) and \
            self.namespaces == other.namespaces and self.name == other.name


class AbstractGenericString(AbstractNestableType):
    """The abstract base class of all string types."""

    __slots__ = ()

    def has_maximum_size(self) -> bool:
        raise NotImplementedError('Only implemented in subclasses')


class AbstractString(AbstractGenericString):
    """The abstract base class of 8-bit string types."""

    __slots__ = ()


class BoundedString(AbstractString):
    """A 8-bit string type with a limited number of characters."""

    __slots__ = ('maximum_size', )

    def __init__(self, maximum_size: int) -> None:
        """
        Create a BoundedString.

        :param maximum_size: the maximum length of the string in characters
        """
        super().__init__()
        assert maximum_size >= 0
        self.maximum_size = maximum_size

    def has_maximum_size(self) -> 'Literal[True]':
        return True

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, BoundedString):
            return False
        return super().__eq__(other) and \
            self.maximum_size == other.maximum_size


class UnboundedString(AbstractString):
    """A 8-bit string type with an unlimited number of characters."""

    __slots__ = ()

    def has_maximum_size(self) -> 'Literal[False]':
        return False


class AbstractWString(AbstractGenericString):
    """The abstract base class of 16-bit string types."""

    __slots__ = ()


class BoundedWString(AbstractWString):
    """A 16-bit string type."""

    __slots__ = ('maximum_size', )

    def __init__(self, maximum_size: Union[int, str]) -> None:
        """
        Create a BoundedWString.

        :param maximum_size: the maximum length of the string in characters
          (must be greater than zero)
        """
        super().__init__()
        # TODO(dirk-thomas) can't be enforced yet since the parser might pass a
        # constant name
        # assert maximum_size > 0
        self.maximum_size = maximum_size

    def has_maximum_size(self) -> 'Literal[True]':
        return True

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, BoundedWString):
            return False
        return super().__eq__(other) and \
            self.maximum_size == other.maximum_size


class UnboundedWString(AbstractWString):
    """A 16-bit string type."""

    __slots__ = ()

    def has_maximum_size(self) -> 'Literal[False]':
        return False


# the following types are templated on a base type

class AbstractNestedType(AbstractType):
    """
    The abstract base class of nested types.

    Arrays and sequences are nested types which contain elements of nestable
    types.
    """

    __slots__ = ('value_type', )

    def __init__(self, value_type: AbstractNestableType) -> None:
        """
        Create an AbstractNestedType.

        :param value_type: the type of the nested elements
        """
        super().__init__()
        assert isinstance(value_type, AbstractNestableType)
        self.value_type = value_type

    def has_maximum_size(self) -> bool:
        raise NotImplementedError('Only implemented in subclasses')

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, AbstractNestedType):
            return False
        return super().__eq__(other) and self.value_type == other.value_type


class Array(AbstractNestedType):
    """An array type with a static size."""

    __slots__ = ('size')

    def __init__(self, value_type: AbstractNestableType, size: int) -> None:
        """
        Create an Array.

        :param value_type: the type of each element in the array
        :param size: the number of elements in the array (must be greater than
          zero)
        """
        super().__init__(value_type)
        assert size > 0
        self.size = size

    def has_maximum_size(self) -> 'Literal[True]':
        return True

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, Array):
            return False
        return super().__eq__(other) and self.size == other.size


class AbstractSequence(AbstractNestedType):
    """The abstract base class of sequence types."""

    __slots__: Set[str] = set()

    def __init__(self, value_type: AbstractNestableType) -> None:
        super().__init__(value_type)


class BoundedSequence(AbstractSequence):
    """A sequence type with a maximum number of elements."""

    __slots__ = ('maximum_size', )

    def __init__(self, value_type: AbstractNestableType, maximum_size: int) -> None:
        """
        Create a BoundedSequence.

        :param basetype: the type of each element in the sequence
        :param maximum_size: the maximum number of elements in the sequence
        """
        super().__init__(value_type)
        assert maximum_size > 0
        self.maximum_size = maximum_size

    def has_maximum_size(self) -> 'Literal[True]':
        return True

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, BoundedSequence):
            return False
        return super().__eq__(other) and \
            self.maximum_size == other.maximum_size


class UnboundedSequence(AbstractSequence):
    """A sequence type with an unlimited number of elements."""

    __slots__ = ()

    def __init__(self, value_type: AbstractNestableType) -> None:
        """
        Create an UnboundedSequence.

        :param value_type: the type of each element in the sequence
        """
        super().__init__(value_type)

    def has_maximum_size(self) -> 'Literal[False]':
        return False


ValueType = Union[str, int, float, bool, Dict[str, Union[str, int, float, bool]], None]


class Annotation:
    """An annotation identified by a name with an arbitrary value."""

    __slots__ = ('name', 'value')

    def __init__(self, name: str, value: ValueType) -> None:
        """
        Create an Annotation.

        :param name: the type of the annotation as defined in the IDL spec
        :param value: the type of the value is defined by the annotation, it
          can be a primitive type like int or str or a dictionary containing
          multiple key-value pairs
        """
        assert isinstance(name, str)
        self.name = name
        self.value = value


class Annotatable:
    """The base class for types which can have annotations."""

    __slots__ = ('annotations', )

    def __init__(self) -> None:
        self.annotations: List[Annotation] = []

    def get_annotation_value(self, name: str) -> ValueType:
        """
        Get the unique value of an annotation of a specific type.

        :param name: the name of the annotation type
        :returns: the annotation value
        :raises: ValueError if there is no or multiple annotations with the
          given name
        """
        values = self.get_annotation_values(name)
        if not values:
            raise ValueError(f"No '{name}' annotation")
        if len(values) > 1:
            raise ValueError(f"Multiple '{name}' annotations")
        return values[0]

    def get_annotation_values(self, name: str) -> List[ValueType]:
        """
        Get the values of annotations of a specific type.

        :param name: the name of the annotation type
        :returns: a list of annotation values
        """
        return [a.value for a in self.annotations if a.name == name]

    def get_comment_lines(self) -> List[str]:
        """
        Get the comment lines of the annotatable.

        :returns: a list of comment lines
        """
        comments = [
            x['text'] for x in self.get_annotation_values('verbatim') if
            isinstance(x, Dict) and
            'language' in x and 'text' in x and x['language'] == 'comment'
        ]
        lines: List[str] = []
        for comment in comments:
            lines.extend(str(comment).splitlines())
        return lines

    def has_annotation(self, name: str) -> bool:
        """
        Check if there is exactly one annotation of a specific type.

        :param name: the name of the annotation type
        :returns: True if there is exactly one annotation, False otherwise
        """
        values = self.get_annotation_values(name)
        return len(values) == 1

    def has_annotations(self, name: str) -> bool:
        """
        Check if there are any annotations of a specific type.

        :param name: the name of the annotation type
        :returns: True if there are any annotations, False otherwise
        """
        annotations = self.get_annotation_values(name)
        return bool(annotations)


class Member(Annotatable):
    """A member of a structure."""

    __slots__ = ('type', 'name')

    def __init__(self, type_: AbstractType, name: str):
        """
        Create a Member.

        :param type_: the type of the member
        :param name: the name of the member
        """
        super().__init__()
        assert isinstance(type_, AbstractType)
        self.type = type_
        self.name = name


class Structure(Annotatable):
    """A namespaced type containing of a list of members."""

    __slots__ = ('namespaced_type', 'members')

    def __init__(self, namespaced_type: NamespacedType,
                 members: Optional[List['Member']] = None) -> None:
        """
        Create a Structure.

        :param namespaced_type: the namespaced type identifying the structure
        :param list members: the members of the structure
        """
        super().__init__()
        assert isinstance(namespaced_type, NamespacedType)
        self.namespaced_type = namespaced_type
        self.members = members or []

    def has_any_member_with_annotation(self, name: str) -> bool:
        """
        Check whether any member has a particular annotation.

        :param str name: the name of the annotation
        :returns: True if there is at least one member with the annotation, False otherwise
        """
        has_any = [member.name for member in self.members if member.has_annotation(name)]
        return bool(has_any)


class Include:
    """An include statement."""

    __slots__ = ('locator', )

    def __init__(self, locator: str) -> None:
        """
        Create an Include.

        :param locator: a URI identifying the included file
        """
        self.locator = locator


class Constant(Annotatable):
    """A constant definition."""

    __slots__ = ('name', 'type', 'value')

    def __init__(self, name: str, type_: AbstractType,
                 value: Union[str, int, float, bool]) -> None:
        """
        Create a Constant.

        :param name: the name of the constant
        :param type_: the type of the constant
        :param value: the value of the constant
        """
        super().__init__()
        assert isinstance(type_, AbstractType)
        self.name = name
        self.type = type_
        self.value = value


class Message:
    """A structure containing constants."""

    __slots__ = ('structure', 'constants')

    def __init__(self, structure: Structure) -> None:
        """
        Create a Message.

        :param structure: the structure of the message
        """
        super().__init__()
        assert isinstance(structure, Structure)
        self.structure = structure
        self.constants: List[Constant] = []


class Service:
    """A namespaced type containing a request and response message."""

    __slots__ = ('namespaced_type', 'request_message', 'response_message', 'event_message')

    def __init__(
        self, namespaced_type: NamespacedType, request: Message,
        response: Message
    ) -> None:
        """
        Create a Service.

        :param namespaced_type: the namespaced type identifying the
          service
        :param request: the request message
        :param response: the response message
        """
        super().__init__()

        assert isinstance(namespaced_type, NamespacedType)
        self.namespaced_type = namespaced_type

        assert isinstance(request, Message)
        assert request.structure.namespaced_type.namespaces == \
            namespaced_type.namespaces
        assert request.structure.namespaced_type.name == \
            namespaced_type.name + SERVICE_REQUEST_MESSAGE_SUFFIX
        self.request_message = request

        assert isinstance(response, Message)
        assert response.structure.namespaced_type.namespaces == \
            namespaced_type.namespaces
        assert response.structure.namespaced_type.name == \
            namespaced_type.name + SERVICE_RESPONSE_MESSAGE_SUFFIX
        self.response_message = response

        self.event_message = Message(
            Structure(
                NamespacedType(
                    namespaces=namespaced_type.namespaces,
                    name=f'{namespaced_type.name}{SERVICE_EVENT_MESSAGE_SUFFIX}'
                ),
                members=[
                    Member(NamespacedType(['service_msgs', 'msg'], 'ServiceEventInfo'), 'info'),
                    Member(BoundedSequence(request.structure.namespaced_type, 1), 'request'),
                    Member(BoundedSequence(response.structure.namespaced_type, 1), 'response'),
                ]
            )
        )


class Action:
    """A namespaced type of an action including the derived types."""

    __slots__ = (
        'namespaced_type', 'goal', 'result', 'feedback',
        'send_goal_service', 'get_result_service', 'feedback_message',
        'implicit_includes')

    def __init__(
        self, namespaced_type: NamespacedType, goal: Message, result: Message,
        feedback: Message
    ) -> None:
        """
        Create an Action.

        From the provided types the actually used services and messages are
        derived.

        :param namespaced_type: the namespaced type identifying the action
        :param goal: the goal message
        :param result: the result message
        :param feedback: the feedback message
        """
        super().__init__()

        assert isinstance(namespaced_type, NamespacedType)
        self.namespaced_type = namespaced_type

        # explicitly defined types
        assert isinstance(goal, Message)
        assert goal.structure.namespaced_type.namespaces == \
            namespaced_type.namespaces
        assert goal.structure.namespaced_type.name == namespaced_type.name + \
            ACTION_GOAL_SUFFIX
        self.goal = goal

        assert isinstance(result, Message)
        assert result.structure.namespaced_type.namespaces == \
            namespaced_type.namespaces
        assert result.structure.namespaced_type.name == \
            namespaced_type.name + ACTION_RESULT_SUFFIX
        self.result = result

        assert isinstance(feedback, Message)
        assert feedback.structure.namespaced_type.namespaces == \
            namespaced_type.namespaces
        assert feedback.structure.namespaced_type.name == \
            namespaced_type.name + ACTION_FEEDBACK_SUFFIX
        self.feedback = feedback

        # necessary include for injected timestamp member
        self.implicit_includes = [
            Include('builtin_interfaces/msg/Time.idl'),
            Include('unique_identifier_msgs/msg/UUID.idl'),
        ]

        # derived types
        goal_id_type = NamespacedType(
                namespaces=['unique_identifier_msgs', 'msg'], name='UUID')

        goal_service_name = namespaced_type.name + ACTION_GOAL_SERVICE_SUFFIX
        self.send_goal_service = Service(
            NamespacedType(
                namespaces=namespaced_type.namespaces, name=goal_service_name),
            request=Message(Structure(
                NamespacedType(
                    namespaces=namespaced_type.namespaces,
                    name=goal_service_name + SERVICE_REQUEST_MESSAGE_SUFFIX),
                members=[
                    Member(goal_id_type, 'goal_id'),
                    Member(goal.structure.namespaced_type, 'goal')]
            )),
            response=Message(Structure(
                NamespacedType(
                    namespaces=namespaced_type.namespaces,
                    name=goal_service_name + SERVICE_RESPONSE_MESSAGE_SUFFIX),
                members=[
                    Member(BasicType('boolean'), 'accepted'),
                    Member(
                        NamespacedType(['builtin_interfaces', 'msg'], 'Time'),
                        'stamp')]
            )),
        )

        result_service_name = namespaced_type.name + \
            ACTION_RESULT_SERVICE_SUFFIX
        self.get_result_service = Service(
            NamespacedType(
                namespaces=namespaced_type.namespaces,
                name=result_service_name),
            request=Message(Structure(
                NamespacedType(
                    namespaces=namespaced_type.namespaces,
                    name=result_service_name + SERVICE_REQUEST_MESSAGE_SUFFIX),
                members=[Member(goal_id_type, 'goal_id')]
            )),
            response=Message(Structure(
                NamespacedType(
                    namespaces=namespaced_type.namespaces,
                    name=result_service_name + SERVICE_RESPONSE_MESSAGE_SUFFIX),
                members=[
                    Member(BasicType('int8'), 'status'),
                    Member(result.structure.namespaced_type, 'result')]
            )),
        )

        self.feedback_message = Message(Structure(
            NamespacedType(
                namespaces=namespaced_type.namespaces,
                name=namespaced_type.name + ACTION_FEEDBACK_MESSAGE_SUFFIX),
            members=[
                Member(goal_id_type, 'goal_id'),
                Member(feedback.structure.namespaced_type, 'feedback')]
        ))


class IdlLocator:
    """A URL of an IDL file."""

    __slots__ = ('basepath', 'relative_path')

    def __init__(self, basepath: pathlib.Path, relative_path: pathlib.Path) -> None:
        """
        Create an IdlLocator.

        :param basepath: the basepath of file
        :param relative_path: the path relative to the basepath of the file
        """
        super().__init__()
        self.basepath = pathlib.Path(basepath)
        self.relative_path = pathlib.Path(relative_path)

    def get_absolute_path(self) -> pathlib.Path:
        return self.basepath / self.relative_path


IdlContentElement: 'TypeAlias' = Union[Include, Message, Service, Action]
IdlContentElementT = TypeVar('IdlContentElementT', bound=IdlContentElement)


class IdlContent:
    """The content of an IDL file consisting of a list of elements."""

    __slots__ = ('elements', )

    def __init__(self) -> None:
        super().__init__()
        self.elements: List[IdlContentElement] = []

    def get_elements_of_type(
            self,
            type_: Type[IdlContentElementT]
    ) -> List[IdlContentElementT]:
        return [e for e in self.elements if isinstance(e, type_)]


class IdlFile:
    """Descriptor for a parsed IDL file."""

    __slots__ = ('locator', 'content')

    def __init__(self, locator: IdlLocator, content: IdlContent) -> None:
        """
        Create an IdlFile.

        :param locator: the locator of the IDL file
        :param content: the content of the IDL file
        """
        super().__init__()
        assert isinstance(locator, IdlLocator)
        self.locator = locator
        assert isinstance(content, IdlContent)
        self.content = content
