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

from collections import OrderedDict
import pathlib


"""Basic types as defined by the IDL specification."""
BASIC_TYPES = [
    'short',
    'unsigned short',
    'long',
    'unsigned long',
    'long long',
    'unsigned long long',
    'float',
    'double',
    'long double',
    'char',
    'wchar',
    'boolean',
    'octet',
    'int8',
    'uint8',
    'int16',
    'uint16',
    'int32',
    'uint32',
    'int64',
    'uint64',
]

SERVICE_REQUEST_MESSAGE_SUFFIX = '_Request'
SERVICE_RESPONSE_MESSAGE_SUFFIX = '_Response'

ACTION_GOAL_SERVICE_SUFFIX = '_Goal'
ACTION_RESULT_SERVICE_SUFFIX = '_Result'
ACTION_FEEDBACK_MESSAGE_SUFFIX = '_Feedback'
ACTION_WRAPPER_TYPE_SUFFIX = '_Action'


class AbstractType:
    """The base class for all types."""

    __slots__ = ()

    def __eq__(self, other):
        return type(self) == type(other)


class BaseType(AbstractType):
    """The base class for types which can be used inside nested types."""

    __slots__ = ()


class BasicType(BaseType):
    """A basic type according to the IDL specification."""

    __slots__ = ('type', )

    def __init__(self, typename):
        """
        Constructor.

        :param str typename: the name of the basic type
        """
        super().__init__()
        assert typename in BASIC_TYPES
        self.type = typename

    def __eq__(self, other):
        return super().__eq__(other) and self.type == other.type


class NamedType(BaseType):
    """A type identified by the name."""

    __slots__ = ('name')

    def __init__(self, name):
        """
        Constructor.

        :param str name: the name
        """
        super().__init__()
        self.name = name

    def __eq__(self, other):
        return super().__eq__(other) and self.name == other.name


class NamespacedType(BaseType):
    """A type identified by a name in a namespaced scope."""

    __slots__ = ('namespaces', 'name')

    def __init__(self, namespaces, name):
        """
        Constructor.

        :param list[str] namespaces: the names of nested namespaces identifying
          a specific scope
        :param str name: the name of the type within that scope
        """
        super().__init__()
        self.namespaces = namespaces
        self.name = name

    def __eq__(self, other):
        return super().__eq__(other) and \
            self.namespaces == other.namespaces and self.name == other.name


class BaseString(BaseType):
    """The base class of string types."""

    __slots__ = ('maximum_size', )

    def __init__(self, maximum_size=None):
        """
        Constructor.

        :param int maximum_size: the maximum length of the string in
          characters, or None if unlimited
        """
        self.maximum_size = maximum_size

    def __eq__(self, other):
        return super().__eq__(other) and \
            self.maximum_size == other.maximum_size


class String(BaseString):
    """A 8-bit string type."""

    __slots__ = ()

    def __init__(self, maximum_size=None):
        """
        Constructor.

        :param int maximum_size: the maximum length of the string in
          characters, or None if unlimited
        """
        super().__init__(maximum_size=maximum_size)


class WString(BaseString):
    """A 16-bit string type."""

    __slots__ = ()

    def __init__(self, maximum_size=None):
        """
        Constructor.

        :param int maximum_size: the maximum length of the string in
          characters, or None if unlimited
        """
        super().__init__(maximum_size=maximum_size)


# the following types are templated on a base type

class NestedType(AbstractType):
    """The base class of nested types."""

    __slots__ = ('basetype', )

    def __init__(self, basetype):
        """
        Constructor.

        :param BaseType basetype: the base of the nested type
        """
        super().__init__()
        assert isinstance(basetype, BaseType), basetype
        self.basetype = basetype

    def __eq__(self, other):
        return super().__eq__(other) and self.basetype == other.basetype


class Array(NestedType):
    """An array type with a static size."""

    __slots__ = ('size')

    def __init__(self, basetype, size):
        """
        Constructor.

        :param BaseType basetype: the type of each element in the nested type
        :param int size: the number of elements in the array
        """
        super().__init__(basetype)
        self.size = size

    def __eq__(self, other):
        return super().__eq__(other) and self.size == other.size


class Sequence(NestedType):
    """The base class of sequence types."""

    __slots__ = set()

    def __init__(self, basetype):
        super().__init__(basetype)


class UnboundedSequence(Sequence):
    """A sequence type with an unlimited number of elements."""

    __slots__ = ()

    def __init__(self, basetype):
        """
        Constructor.

        :param BaseType basetype: the type of each element in the sequence
        """
        super().__init__(basetype)


class BoundedSequence(Sequence):
    """A sequence type with a maximum number of elements."""

    __slots__ = ('upper_bound', )

    def __init__(self, basetype, upper_bound):
        """
        Constructor.

        :param BaseType basetype: the type of each element in the sequence
        :param int upper_bound: the maximum number of elements in the sequence
        """
        super().__init__(basetype)
        assert isinstance(upper_bound, int)
        self.upper_bound = upper_bound

    def __eq__(self, other):
        return super().__eq__(other) and self.upper_bound == other.upper_bound


class Annotation:
    """An annotation identified by a name with an arbitrary value."""

    __slots__ = ('name', 'value')

    def __init__(self, name, value):
        """
        Constructor.

        :param str name: the type of the annotation as defined in the IDL spec
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

    def __init__(self):
        self.annotations = []

    def get_annotation_value(self, name):
        """
        Get the unique value of an annotation of a specific type.

        :param str name: the name of the annotation type
        :returns: the annotation value
        :raises: ValueError if there is no or multiple annotations with the
          given name
        """
        values = self.get_annotation_values(name)
        if not values:
            raise ValueError("No '{name}' annotation".format_map(locals()))
        if len(values) > 1:
            raise ValueError("Multiple '{name}' annotations".format_map(locals()))
        return values[0]

    def get_annotation_values(self, name):
        """
        Get the values of annotations of a specific type.

        :param str name: the name of the annotation type
        :returns: a list of annotation values
        """
        return [a.value for a in self.annotations if a.name == name]

    def has_annotation(self, name):
        """
        Check if there is exactly one annotation of a specific type.

        :param str name: the name of the annotation type
        :returns: True if there is exactly one annotation, False otherwise
        """
        values = self.get_annotation_values(name)
        return len(values) == 1

    def has_annotations(self, name):
        """
        Check if there are any annotations of a specific type.

        :param str name: the name of the annotation type
        :returns: True if there are any annotations, False otherwise
        """
        annotations = self.get_annotation_values(name)
        return bool(annotations)


class Member(Annotatable):
    """A member of a structure."""

    __slots__ = ('type', 'name')

    def __init__(self, type_, name):
        """
        Constructor.

        :param AbstractTypestr type_: the type of the member
        :param str name: the name of the member
        """
        super().__init__()
        assert isinstance(type_, AbstractType)
        self.type = type_
        self.name = name


class Structure(Annotatable):
    """A namespaced type containing of a list of members."""

    __slots__ = ('type', 'members')

    def __init__(self, type_, members=None):
        """
        Constructor.

        :param NamespacedType type_: the namespaced type identifying the
          structure
        :param list members: the members of the structure
        """
        super().__init__()
        assert isinstance(type_, NamespacedType)
        self.type = type_
        self.members = members or []


class Include:
    """An include statement."""

    __slots__ = ('locator', )

    def __init__(self, locator):
        """
        Constructor.

        :param str locator: a URI identifying the included file
        """
        self.locator = locator


class Constant(Annotatable):
    """A constant definition."""

    __slots__ = ('name', 'type', 'value')

    def __init__(self, name, type_, value):
        """
        Constructor.

        :param str name: the name of the constant
        :param AbstractTypestr type_: the type of the constant
        :param value: the value of the constant
        """
        super().__init__()
        assert isinstance(type_, AbstractType)
        self.name = name
        self.type = type_
        self.value = value


class Message:
    """A structure containing constants and enums."""

    __slots__ = ('structure', 'constants', 'enums')

    def __init__(self, structure):
        """
        Constructor.

        :param Structure structure: the structure of the message
        """
        super().__init__()
        assert isinstance(structure, Structure)
        self.structure = structure
        self.constants = OrderedDict()
        self.enums = []


class Service:
    """A namespaced type containing a request and response message."""

    __slots__ = ('structure_type', 'request_message', 'response_message')

    def __init__(self, type_, request, response):
        """
        Constructor.

        :param NamespacedType type_: the namespaced type identifying the
          service
        :param Message request: the request message
        :param Message response: the response message
        """
        super().__init__()

        assert isinstance(type_, NamespacedType)
        self.structure_type = type_

        assert isinstance(request, Message)
        assert request.structure.type.namespaces == type_.namespaces
        assert request.structure.type.name == type_.name + \
            SERVICE_REQUEST_MESSAGE_SUFFIX
        self.request_message = request

        assert isinstance(response, Message)
        assert response.structure.type.namespaces == type_.namespaces
        assert response.structure.type.name == type_.name + \
            SERVICE_RESPONSE_MESSAGE_SUFFIX
        self.response_message = response


class Action:
    """A namespaced type of an action including the derived types."""

    __slots__ = (
        'structure_type', 'goal_request', 'result_response', 'feedback',
        'goal_service', 'result_service', 'feedback_message')

    def __init__(self, type_, goal_request, result_response, feedback):
        """
        Constructor.

        From the provide type the actually used services and message are
        derived.

        :param NamespacedType type_: the namespaced type identifying the action
        :param Message goal_request: the goal request message
        :param Message result_response: the result response message
        :param Message feedback: the feedback message
        """
        super().__init__()

        assert isinstance(type_, NamespacedType)
        self.structure_type = type_

        assert isinstance(goal_request, Message)
        assert goal_request.structure.type.namespaces == type_.namespaces
        assert goal_request.structure.type.name == type_.name + \
            ACTION_GOAL_SERVICE_SUFFIX + SERVICE_REQUEST_MESSAGE_SUFFIX
        self.goal_request = goal_request

        assert isinstance(result_response, Message)
        assert result_response.structure.type.namespaces == type_.namespaces
        assert result_response.structure.type.name == type_.name + \
            ACTION_RESULT_SERVICE_SUFFIX + SERVICE_RESPONSE_MESSAGE_SUFFIX
        self.result_response = result_response

        assert isinstance(feedback, Message)
        assert feedback.structure.type.namespaces == type_.namespaces
        assert feedback.structure.type.name == type_.name + \
            ACTION_FEEDBACK_MESSAGE_SUFFIX
        self.feedback = feedback

        # derived types
        goal_service_name = type_.name + ACTION_WRAPPER_TYPE_SUFFIX + \
            ACTION_GOAL_SERVICE_SUFFIX
        self.goal_service = Service(
            NamespacedType(
                namespaces=type_.namespaces, name=goal_service_name),
            request=Message(Structure(
                NamespacedType(
                    namespaces=type_.namespaces,
                    name=goal_service_name + SERVICE_REQUEST_MESSAGE_SUFFIX),
                members=[
                    Member(Array(BasicType('uint8'), 16), 'uuid'),
                    Member(goal_request.structure.type, 'request')]
            )),
            response=Message(Structure(
                NamespacedType(
                    namespaces=type_.namespaces,
                    name=goal_service_name + SERVICE_RESPONSE_MESSAGE_SUFFIX),
                members=[
                    Member(BasicType('boolean'), 'accepted'),
                    Member(
                        NamespacedType(['builtin_interfaces', 'msg'], 'Time'),
                        'stamp')]
            )),
        )

        result_service_name = type_.name + ACTION_WRAPPER_TYPE_SUFFIX + \
            ACTION_RESULT_SERVICE_SUFFIX
        self.result_service = Service(
            NamespacedType(
                namespaces=type_.namespaces, name=result_service_name),
            request=Message(Structure(
                NamespacedType(
                    namespaces=type_.namespaces,
                    name=result_service_name + SERVICE_REQUEST_MESSAGE_SUFFIX),
                members=[Member(Array(BasicType('uint8'), 16), 'uuid')]
            )),
            response=Message(Structure(
                NamespacedType(
                    namespaces=type_.namespaces,
                    name=result_service_name + SERVICE_RESPONSE_MESSAGE_SUFFIX),
                members=[
                    Member(BasicType('int8'), 'status'),
                    Member(result_response.structure.type, 'response')]
            )),
        )

        self.feedback_message = Message(Structure(
            NamespacedType(
                namespaces=type_.namespaces,
                name=type_.name + ACTION_WRAPPER_TYPE_SUFFIX +
                ACTION_FEEDBACK_MESSAGE_SUFFIX),
            members=[
                Member(Array(BasicType('uint8'), 16), 'uuid'),
                Member(feedback.structure.type, 'feedback')]
        ))


class IdlLocator:
    """A URL of an IDL file."""

    __slots__ = ('basepath', 'relative_path')

    def __init__(self, basepath, relative_path):
        """
        Constructor.

        :param str basepath: the basepath of file
        :param str relative_path: the path relative to the basepath of the file
        """
        super().__init__()
        self.basepath = pathlib.Path(basepath)
        self.relative_path = pathlib.Path(relative_path)

    def get_absolute_path(self):
        return self.basepath / self.relative_path


class IdlContent:
    """The content of an IDL file consisting of a list of elements."""

    __slots__ = ('elements', )

    def __init__(self):
        super().__init__()
        self.elements = []

    def get_elements_of_type(self, type_):
        return [e for e in self.elements if isinstance(e, type_)]


class IdlFile:
    """Descriptor for a parsed IDL file."""

    __slots__ = ('locator', 'content')

    def __init__(self, locator, content):
        """
        Constructor.

        :param IdlLocator locator: the locator of the IDL file
        :param IdlContent content: the content of the IDL file
        """
        super().__init__()
        assert isinstance(locator, IdlLocator)
        self.locator = locator
        assert isinstance(content, IdlContent)
        self.content = content
