import os
import re

PACKAGE_NAME_MESSAGE_TYPE_SEPARATOR = '/'
COMMENT_DELIMITER = '#'
CONSTANT_SEPARATOR = '='

# TODO reconsider final set of primitive types
PRIMITIVE_TYPES = [
    'bool',
    'float32',
    'float64',
    'int8',
    'uint8',
    'int16',
    'uint16',
    'int32',
    'uint32',
    'int64',
    'uint64',
    'string',
    'char',  # deprecated
    'byte',  # deprecated
    'duration',  # for compatibility only
    'time',  # for compatibility only
]

VALID_PACKAGE_NAME_PATTERN = re.compile('^[a-z][a-z0-9_]*$')
# TODO replace relaxed patterns used for compatibility
#VALID_FIELD_NAME_PATTERN = re.compile('^[a-z][a-z0-9_]*$')
VALID_FIELD_NAME_PATTERN = re.compile('^[A-Za-z][A-Za-z0-9_]*$')
# TODO replace relaxed patterns used for compatibility
#VALID_MESSAGE_NAME_PATTERN = re.compile('^[A-Z][A-Za-z0-9]*$')
VALID_MESSAGE_NAME_PATTERN = re.compile('^[A-Za-z][A-Za-z0-9]*$')
VALID_CONSTANT_NAME_PATTERN = re.compile('^[A-Z][A-Z0-9_]*$')


class InvalidMessageSpecification(Exception):
    pass


class InvalidResourceName(InvalidMessageSpecification):
    pass


class UnknownMessageType(InvalidMessageSpecification):
    pass


def is_valid_package_name(name):
    try:
        m = VALID_PACKAGE_NAME_PATTERN.match(name)
    except TypeError:
        raise InvalidResourceName(name)
    return m is not None and m.group(0) == name


def is_valid_field_name(name):
    try:
        m = VALID_FIELD_NAME_PATTERN.match(name)
    except TypeError:
        raise InvalidResourceName(name)
    return m is not None and m.group(0) == name


def is_valid_message_name(name):
    try:
        m = VALID_MESSAGE_NAME_PATTERN.match(name)
    except TypeError:
        raise InvalidResourceName(name)
    return m is not None and m.group(0) == name


def is_valid_constant_name(name):
    try:
        m = VALID_CONSTANT_NAME_PATTERN.match(name)
    except TypeError:
        raise InvalidResourceName(name)
    return m is not None and m.group(0) == name


class BaseType(object):

    __slots__ = ['pkg_name', 'type']

    def __init__(self, type_string, context_package_name=None):
        # check for primitive types
        if type_string in PRIMITIVE_TYPES:
            self.pkg_name = None
            self.type = type_string
        else:
            # split non-primitive type information
            parts = type_string.split(PACKAGE_NAME_MESSAGE_TYPE_SEPARATOR)
            if not (len(parts) == 2 or
                   (len(parts) == 1 and context_package_name is not None)):
                raise InvalidResourceName(type_string)

            if len(parts) == 2:
                # either the type string contains the package name
                self.pkg_name = parts[0]
                self.type = parts[1]
            else:
                # or the package name is provided by context
                self.pkg_name = context_package_name
                self.type = type_string
            if not is_valid_package_name(self.pkg_name):
                raise InvalidResourceName(self.pkg_name)
            if not is_valid_message_name(self.type):
                raise InvalidResourceName(self.type)

    def is_primitive_type(self):
        return self.pkg_name is None

    def __eq__(self, other):
        if other is None or not isinstance(other, BaseType):
            return False
        return self.pkg_name == other.pkg_name and \
            self.type == other.type

    def __str__(self):
        if self.pkg_name is None:
            return self.type
        return '%s/%s' % (self.pkg_name, self.type)


class Type(BaseType):

    __slots__ = ['is_array', 'array_size']

    def __init__(self, type_string, context_package_name=None):
        # check for array brackets
        self.is_array = type_string[-1] == ']'

        # check for array size, explicit or infinite
        self.array_size = None
        if self.is_array:
            try:
                index = type_string.rindex('[')
            except ValueError:
                raise InvalidResourceName(type_string)
            if index < len(type_string) - 2:
                self.array_size = int(type_string[index + 1:-1])
                if self.array_size <= 0:
                    raise InvalidResourceName(type_string)
            type_string = type_string[:index]

        super(Type, self).__init__(
            type_string,
            context_package_name=context_package_name)

    def __eq__(self, other):
        if other is None or not isinstance(other, Type):
            return False
        return super(Type, self).__eq__(other) and \
            self.is_array == other.is_array and \
            self.array_size == other.array_size

    def __str__(self):
        super_str = super(Type, self).__str__()
        if not self.is_array:
            return super_str
        if self.array_size is None:
            return '%s[]' % super_str
        return '%s[%d]' % (super_str, self._array_size)


class Constant(object):

    __slots__ = ['primitive_type', 'name', 'value']

    def __init__(self, primitive_type, name, value):
        assert(primitive_type in PRIMITIVE_TYPES)
        self.primitive_type = primitive_type
        assert(is_valid_constant_name(name))
        self.name = name
        assert(value is not None)

        if self.primitive_type == 'bool':
            # TODO specify string to bool conversion
            pass
        elif self.primitive_type in ['float32', 'float64']:
            self.value = float(value)
        elif self.primitive_type in [
            'int8', 'uint8',
            'int16', 'uint16',
            'int32', 'uint32',
            'int64', 'uint64',
        ]:
            self.value = int(value)
        else:
            self.value = value

    def __eq__(self, other):
        if other is None or not isinstance(other, Constant):
            return False
        return self.primitive_type == other.primitive_type and \
            self.name == other.name and \
            self.value == other.value

    def __repr__(self):
        return '%s %s=%s' % (self.primitive_type, self.name, self.value)

    def __str__(self):
        return '%s %s=%s' % (self.primitive_type, self.name, self.value)


class Field(object):

    def __init__(self, type_, name):
        assert(isinstance(type_, Type))
        self.type = type_
        assert(is_valid_field_name(name))
        self.name = name

    def __eq__(self, other):
        if other is None or not isinstance(other, Field):
            return False
        else:
            return self.type == other.type and \
                self.name == other.name

    def __repr__(self):
        return '%s %s' % (str(self.type), self.name)


class MessageSpecification(object):

    def __init__(self, pkg_name, msg_name, fields, constants):
        assert(msg_name not in PRIMITIVE_TYPES)
        self.base_type = BaseType(
            pkg_name + PACKAGE_NAME_MESSAGE_TYPE_SEPARATOR + msg_name)

        self.fields = []
        for field in fields:
            assert(isinstance(field, Field))
            self.fields.append(field)
        # ensure that there are no duplicate field names
        assert(len(set([f.name for f in self.fields])) == len(self.fields))

        self.constants = []
        for constant in constants:
            assert(isinstance(constant, Constant))
            self.constants.append(constant)
        # ensure that there are no duplicate constant names
        assert(len(set([c.name for c in self.constants])) ==
               len(self.constants))

    def __eq__(self, other):
        if not other or not isinstance(other, MessageSpecification):
            return False
        # TODO write comparison
        return self.base_type == other.base_type


def parse_message_file(pkg_name, interface_filename):
    basename = os.path.basename(interface_filename)
    msg_name = os.path.splitext(basename)[0]
    with open(interface_filename, 'r') as h:
        return parse_message_string(
            pkg_name, msg_name, h.read())


def parse_message_string(pkg_name, msg_name, message_string):
    fields = []
    constants = []

    lines = message_string.splitlines()
    for line in lines:
        # ignore whitespaces and comments
        line = line.strip()
        if not line:
            continue
        index = line.find(COMMENT_DELIMITER)
        if index == 0:
            continue
        if index != -1:
            line = line[:index]
            line = line.rstrip()

        type_string, _, rest = line.partition(' ')
        rest = rest.lstrip()
        index = rest.find(CONSTANT_SEPARATOR)
        if index == -1:
            # line contains a field
            fields.append(Field(
                Type(type_string, context_package_name=pkg_name), rest))
        else:
            # line contains a constant
            name, _, value = rest.partition(CONSTANT_SEPARATOR)
            name = name.rstrip()
            value = value.lstrip()
            constants.append(Constant(type_string, name, value))

    return MessageSpecification(pkg_name, msg_name, fields, constants)


def validate_field_types(spec, known_msg_types):
    for field in spec.fields:
        if field.type.is_primitive_type():
            continue
        base_type = BaseType(BaseType.__str__(field.type))
        if base_type not in known_msg_types:
            raise UnknownMessageType(str(field))
