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


class BaseType(AbstractType):

    __slots__ = ()


class BasicType(BaseType):

    __slots__ = ('type', )

    def __init__(self, typename):
        super().__init__()
        assert typename in BASIC_TYPES
        self.type = typename


class StructureType(BaseType):

    __slots__ = ('namespaces', 'name')

    def __init__(self, namespaces, name):
        super().__init__()

        assert len(namespaces) == 2
        assert namespaces[1] in ('msg', 'srv', 'action')
        self.namespaces = namespaces
        self.name = name


class BaseString(BaseType):

    __slots__ = ('maximum_size', )

    def __init__(self, maximum_size=None):
        self.maximum_size = maximum_size


class String(BaseString):

    __slots__ = ()

    def __init__(self, maximum_size=None):
        super().__init__(maximum_size=maximum_size)


class WString(BaseString):

    __slots__ = ()

    def __init__(self, maximum_size=None):
        super().__init__(maximum_size=maximum_size)


# the following types are templated on a base type

class Array(AbstractType):

    __slots__ = ('basetype', 'size')

    def __init__(self, basetype, size):
        super().__init__()
        assert isinstance(basetype, BaseType)
        self.basetype = basetype
        self.size = size


class Sequence(AbstractType):

    __slots__ = ('basetype', )

    def __init__(self, basetype):
        super().__init__()
        assert isinstance(basetype, BaseType)
        self.basetype = basetype


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


# class Enumeration:

#     pass


class Annotatable:

    __slots__ = ('annotations', )

    def __init__(self):
        self.annotations = []


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
        assert isinstance(type_, StructureType)
        self.type = type_
        self.members = []


class Constant(Annotatable):

    __slots__ = ('type', 'name', 'value')

    def __init__(self, type_, name, value):
        super().__init__()
        assert isinstance(type_, AbstractType)
        self.type = type_
        self.name = name
        self.value = value


class Interface:

    __slots__ = ('includes')

    def __init__(self):
        super().__init__()
        self.includes = []


class Message(Interface):

    __slots__ = ('structure', 'constants', 'enums')

    def __init__(self, structure):
        super().__init__()
        assert isinstance(structure, Structure)
        self.structure = structure
        self.constants = OrderedDict()
        self.enums = []


class Service(Interface):

    __slots__ = ('structure_type', 'request_message', 'response_message')

    def __init__(self, type_, request, response):
        super().__init__()

        assert isinstance(type_, StructureType)
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
