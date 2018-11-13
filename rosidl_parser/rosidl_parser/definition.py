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
    # 'string',
    # 'wstring',
]

SERVICE_REQUEST_MESSAGE_SUFFIX = '_Request'
SERVICE_RESPONSE_MESSAGE_SUFFIX = '_Response'


class AbstractType:

    __slots__ = ()

    def __eq__(self, other):
        return type(self) == type(other)


class BaseType(AbstractType):

    __slots__ = ()


class BasicType(BaseType):

    __slots__ = ('type', )

    def __init__(self, typename):
        super().__init__()
        assert typename in BASIC_TYPES
        self.type = typename

    def __eq__(self, other):
        return super().__eq__(other) and self.type == other.type


class NamedType(BaseType):

    __slots__ = ('name')

    def __init__(self, name):
        super().__init__()
        self.name = name

    def __eq__(self, other):
        return super().__eq__(other) and self.name == other.name


class NamespacedType(BaseType):

    __slots__ = ('namespaces', 'name')

    def __init__(self, namespaces, name):
        super().__init__()
        self.namespaces = namespaces
        self.name = name

    def __eq__(self, other):
        return super().__eq__(other) and \
            self.namespaces == other.namespaces and self.name == other.name


class BaseString(BaseType):

    __slots__ = ('maximum_size', )

    def __init__(self, maximum_size=None):
        self.maximum_size = maximum_size

    def __eq__(self, other):
        return super().__eq__(other) and \
            self.maximum_size == other.maximum_size


class String(BaseString):

    __slots__ = ()

    def __init__(self, maximum_size=None):
        super().__init__(maximum_size=maximum_size)


class WString(BaseString):

    __slots__ = ()

    def __init__(self, maximum_size=None):
        super().__init__(maximum_size=maximum_size)


# the following types are templated on a base type

class NestedType(AbstractType):

    __slots__ = ('basetype', )

    def __init__(self, basetype):
        super().__init__()
        assert isinstance(basetype, BaseType), basetype
        self.basetype = basetype

    def __eq__(self, other):
        return super().__eq__(other) and self.basetype == other.basetype


class Array(NestedType):

    __slots__ = ('size')

    def __init__(self, basetype, size):
        super().__init__(basetype)
        self.size = size

    def __eq__(self, other):
        return super().__eq__(other) and self.size == other.size


class Sequence(NestedType):

    __slots__ = set()

    def __init__(self, basetype):
        super().__init__(basetype)


class UnboundedSequence(Sequence):

    __slots__ = ()

    def __init__(self, basetype):
        super().__init__(basetype)


class BoundedSequence(Sequence):

    __slots__ = ('upper_bound', )

    def __init__(self, basetype, upper_bound):
        super().__init__(basetype)
        assert isinstance(upper_bound, int)
        self.upper_bound = upper_bound

    def __eq__(self, other):
        return super().__eq__(other) and self.upper_bound == other.upper_bound


# class Enumeration:

#     pass


class Annotation:

    __slots__ = ('name', 'value')

    def __init__(self, name, value):
        assert isinstance(name, str)
        self.name = name
        self.value = value


class Annotatable:

    __slots__ = ('annotations', )

    def __init__(self):
        self.annotations = []

    def get_annotation_value(self, name):
        values = self.get_annotation_values(name)
        if not values:
            raise ValueError("No '{name}' annotation".format_map(locals()))
        if len(values) > 1:
            raise ValueError("Multiple '{name}' annotations".format_map(locals()))
        return values[0]

    def get_annotation_values(self, name):
        return [a.value for a in self.annotations if a.name == name]

    def has_annotation(self, name):
        values = self.get_annotation_values(name)
        return len(values) == 1

    def has_annotations(self, name):
        annotations = self.get_annotation_values(name)
        return bool(annotations)


class Member(Annotatable):

    __slots__ = ('type', 'name')

    def __init__(self, type_, name):
        super().__init__()
        assert isinstance(type_, AbstractType)
        self.type = type_
        self.name = name


class Structure(Annotatable):

    __slots__ = ('type', 'members')

    def __init__(self, type_):
        super().__init__()
        assert isinstance(type_, NamespacedType)
        self.type = type_
        self.members = []


class Include:

    __slots__ = ('locator', )

    def __init__(self, locator):
        self.locator = locator


class Constant(Annotatable):

    __slots__ = ('name', 'type', 'value')

    def __init__(self, name, type_, value):
        super().__init__()
        assert isinstance(type_, AbstractType)
        self.name = name
        self.type = type_
        self.value = value


class Message:

    __slots__ = ('structure', 'constants', 'enums')

    def __init__(self, structure):
        super().__init__()
        assert isinstance(structure, Structure)
        self.structure = structure
        self.constants = OrderedDict()
        self.enums = []


class Service:

    __slots__ = ('structure_type', 'request_message', 'response_message')

    def __init__(self, type_, request, response):
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


class IdlIdentifier:

    __slots__ = ('package_name', 'relative_path')

    def __init__(self, package_name, relative_path):
        super().__init__()
        assert isinstance(package_name, str)
        self.package_name = package_name
        assert isinstance(relative_path, pathlib.Path)
        self.relative_path = relative_path


class IdlLocator:

    __slots__ = ('basepath', 'relative_path')

    def __init__(self, basepath, relative_path):
        super().__init__()
        self.basepath = pathlib.Path(basepath)
        self.relative_path = pathlib.Path(relative_path)

    def get_absolute_path(self):
        return self.basepath / self.relative_path


class IdlContent:

    __slots__ = ('elements', )

    def __init__(self):
        super().__init__()
        self.elements = []

    def get_elements_of_type(self, type_):
        return [e for e in self.elements if isinstance(e, type_)]


class IdlFile:

    __slots__ = ('locator', 'content')

    def __init__(self, locator, content):
        super().__init__()
        assert isinstance(locator, IdlLocator)
        self.locator = locator
        assert isinstance(content, IdlContent)
        self.content = content
