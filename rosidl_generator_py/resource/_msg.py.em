# generated from rosidl_generator_py/resource/_msg.py.em
# generated code does not contain a copyright notice

@#######################################################################
@# EmPy template for generating _<msg>.py files
@#
@# Context:
@#  - module_name
@#  - package_name
@#  - spec (rosidl_parser.MessageSpecification)
@#    Parsed specification of the .msg file
@#  - constant_value_to_py (function)
@#  - get_python_type (function)
@#  - value_to_py (function)
@#######################################################################
@
import logging
import traceback


class Metaclass(type):
    """Metaclass of message '@(spec.base_type.type)'."""

    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
@[for constant in spec.constants]@
        '@(constant.name)': @constant_value_to_py(constant.type, constant.value),
@[end for]@
    }

    @@classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('@(package_name)')
        except ImportError:
            logger = logging.getLogger('rosidl_generator_py.@(spec.base_type.type)')
            logger.debug(
                'Failed to import needed modules for type support:\n' + traceback.format_exc())
        else:
            cls._CONVERT_FROM_PY = module.convert_from_py_msg_@(module_name)
            cls._CONVERT_TO_PY = module.convert_to_py_msg_@(module_name)
            cls._TYPE_SUPPORT = module.type_support_msg_@(module_name)
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg_@(module_name)
@{
importable_typesupports = {}
for field in spec.fields:
    if not field.type.is_primitive_type():
        key = '%s.msg.%s' % (field.type.pkg_name, field.type.type)
        if key not in importable_typesupports:
            importable_typesupports[key] = [field.type.pkg_name, field.type.type]
for key in sorted(importable_typesupports.keys()):
    (pkg_name, field_name) = importable_typesupports[key]
    print('%sfrom %s.msg import %s' % (' ' * 4 * 3, pkg_name, field_name))
    print('%sif %s.__class__._TYPE_SUPPORT is None:' % (' ' * 4 * 3, field_name))
    print('%s%s.__class__.__import_type_support__()' % (' ' * 4 * 4, field_name))
}@

    @@classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
@[for constant in spec.constants]@
            '@(constant.name)': cls.__constants['@(constant.name)'],
@[end for]@
@[for field in spec.fields]@
@[  if field.default_value]@
            '@(field.name.upper())__DEFAULT': @value_to_py(field.type, field.default_value),
@[  end if]@
@[end for]@
        }
@[for constant in spec.constants]@

    @@property
    def @(constant.name)(self):
        """Message constant '@(constant.name)'."""
        return Metaclass.__constants['@(constant.name)']
@[end for]@
@[for field in spec.fields]@
@[  if field.default_value]@

    @@property
    def @(field.name.upper())__DEFAULT(cls):
        """Return default value for message field '@(field.name)'."""
        return @value_to_py(field.type, field.default_value)
@[  end if]@
@[end for]@


class @(spec.base_type.type)(metaclass=Metaclass):
@[if not spec.constants]@
    """Message class '@(spec.base_type.type)'."""
@[else]@
    """
    Message class '@(spec.base_type.type)'.

    Constants:
@[  for constant in spec.constants]@
      @(constant.name)
@[  end for]@
    """
@[end if]@

    __slots__ = [
@[for field in spec.fields]@
        '_@(field.name)',
@[end for]@
    ]
@
@[if len(spec.fields) > 0]@

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
@[  for field in spec.fields]@
@[    if field.default_value]@
        self.@(field.name) = kwargs.get(
            '@(field.name)', @(spec.base_type.type).@(field.name.upper())__DEFAULT)
@[    else]@
@[      if not field.type.is_primitive_type() and (not field.type.is_array or
          (field.type.array_size and not field.type.is_upper_bound))]@
        from @(field.type.pkg_name).msg import @(field.type.type)
@[      end if]@
@[      if field.type.array_size and not field.type.is_upper_bound]@
@[        if field.type.type == 'byte']@
        self.@(field.name) = kwargs.get(
            '@(field.name)',
            [bytes([0]) for x in range(@(field.type.array_size))]
        )
@[        elif field.type.type == 'char']@
        self.@(field.name) = kwargs.get(
            '@(field.name)',
            [chr(0) for x in range(@(field.type.array_size))]
        )
@[        else]@
        self.@(field.name) = kwargs.get(
            '@(field.name)',
            [@(get_python_type(field.type))() for x in range(@(field.type.array_size))]
        )
@[        end if]@
@[      elif field.type.is_array]@
        self.@(field.name) = kwargs.get('@(field.name)', [])
@[      elif field.type.type == 'byte']@
        self.@(field.name) = kwargs.get('@(field.name)', bytes([0]))
@[      elif field.type.type == 'char']@
        self.@(field.name) = kwargs.get('@(field.name)', chr(0))
@[      else]@
        self.@(field.name) = kwargs.get('@(field.name)', @(get_python_type(field.type))())
@[      end if]@
@[    end if]@
@[  end for]@
@[end if]@

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = [s[1:] + '=' + repr(getattr(self, s, None)) for s in self.__slots__]
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
@[for field in spec.fields]@
        if self.@(field.name) != other.@(field.name):
            return False
@[end for]@
        return True
@[for field in spec.fields]@

    @@property
    def @(field.name)(self):
        """Message field '@(field.name)'."""
        return self._@(field.name)

    @@@(field.name).setter
    def @(field.name)(self, value):
        if __debug__:
@[  if not field.type.is_primitive_type()]@
            from @(field.type.pkg_name).msg import @(field.type.type)
@[  end if]@
@[  if field.type.is_array]@
            from collections import Sequence
            from collections import Set
            from collections import UserList
            from collections import UserString
@[  elif field.type.string_upper_bound]@
            from collections import UserString
@[  elif field.type.type == 'byte']@
            from collections import ByteString
@[  elif field.type.type in ['char']]@
            from collections import UserString
@[  end if]@
            assert \
@[  if field.type.is_array]@
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
@{assert_msg_suffixes = ['a set or sequence']}@
@[    if field.type.type == 'string' and field.type.string_upper_bound]@
                 all(len(val) <= @field.type.string_upper_bound for val in value) and
@{assert_msg_suffixes.append('and each string value not longer than %d' % field.type.string_upper_bound)}@
@[    end if]@
@[    if field.type.array_size]@
@[      if field.type.is_upper_bound]@
                 len(value) <= @(field.type.array_size) and
@{assert_msg_suffixes.insert(1, 'with length <= %d' % field.type.array_size)}@
@[      else]@
                 len(value) == @(field.type.array_size) and
@{assert_msg_suffixes.insert(1, 'with length %d' % field.type.array_size)}@
@[      end if]@
@[    end if]@
                 all(isinstance(v, @(get_python_type(field.type))) for v in value) and
@{assert_msg_suffixes.append("and each value of type '%s'" % get_python_type(field.type))}@
@[    if field.type.type.startswith('int')]@
@{
nbits = int(field.type.type[3:])
bound = 2**(nbits - 1)
}@
                 all(val >= -@(bound) and val < @(bound) for val in value)), \
@{assert_msg_suffixes.append('and each integer in [%d, %d]' % (-bound, bound - 1))}@
@[    elif field.type.type.startswith('uint')]@
@{
nbits = int(field.type.type[4:])
bound = 2**nbits
}@
                 all(val >= 0 and val < @(bound) for val in value)), \
@{assert_msg_suffixes.append('and each unsigned integer in [0, %d]' % (bound - 1))}@
@[    elif field.type.type == 'char']@
                 all(ord(val) >= -128 and ord(val) < 128 for val in value)), \
@{assert_msg_suffixes.append('and each characters ord() in [-128, 127]')}@
@[    else]@
                 True), \
@[    end if]@
                "The '@(field.name)' field must be @(' '.join(assert_msg_suffixes))"
@[  elif field.type.string_upper_bound]@
                ((isinstance(value, str) or isinstance(value, UserString)) and
                 len(value) <= @(field.type.string_upper_bound)), \
                "The '@(field.name)' field must be string value " \
                'not longer than @(field.type.string_upper_bound)'
@[  elif not field.type.is_primitive_type()]@
                isinstance(value, @(field.type.type)), \
                "The '@(field.name)' field must be a sub message of type '@(field.type.type)'"
@[  elif field.type.type == 'byte']@
                ((isinstance(value, bytes) or isinstance(value, ByteString)) and
                 len(value) == 1), \
                "The '@(field.name)' field must of type 'bytes' or 'ByteString' with a length 1"
@[  elif field.type.type == 'char']@
                ((isinstance(value, str) or isinstance(value, UserString)) and
                 len(value) == 1 and ord(value) >= -128 and ord(value) < 128), \
                "The '@(field.name)' field must of type 'str' or 'UserString' " \
                'with a length 1 and the character ord() in [-128, 127]'
@[  elif field.type.type in [
        'bool',
        'float32', 'float64',
        'int8', 'uint8',
        'int16', 'uint16',
        'int32', 'uint32',
        'int64', 'uint64',
        'string',
    ]]@
                isinstance(value, @(get_python_type(field.type))), \
                "The '@(field.name)' field must of type '@(get_python_type(field.type))'"
@[    if field.type.type.startswith('int')]@
@{
nbits = int(field.type.type[3:])
bound = 2**(nbits - 1)
}@
            assert value >= -@(bound) and value < @(bound), \
                "The '@(field.name)' field must be an integer in [@(-bound), @(bound - 1)]"
@[    elif field.type.type.startswith('uint')]@
@{
nbits = int(field.type.type[4:])
bound = 2**nbits
}@
            assert value >= 0 and value < @(bound), \
                "The '@(field.name)' field must be an unsigned integer in [0, @(bound - 1)]"
@[    end if]@
@[  else]@
                False
@[  end if]@
        self._@(field.name) = value
@[end for]@
