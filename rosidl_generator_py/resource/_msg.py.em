# generated from rosidl_generator_py/resource/_msg.py.em
# generated code does not contain a copyright notice

import logging
import traceback

_convert_from_py = None
_convert_to_py = None
_type_support = None
__type_support_importable = False
try:
    import rclpy
    from rosidl_generator_py import import_type_support
    __type_support_importable = True
except ImportError:
    logger = logging.getLogger('rosidl_generator_py.@(spec.base_type.type)')
    logger.debug('Failed to import needed modules for type support:\n' + traceback.format_exc())

if __type_support_importable:
    rclpy_implementation = rclpy._rclpy.rclpy_get_rmw_implementation_identifier()
    module = import_type_support(
        '@(package_name)', '@(subfolder)', '@(module_name)', rclpy_implementation)
    _convert_from_py = module.convert_from_py_@(module_name)
    _convert_to_py = module.convert_to_py_@(module_name)
    _type_support = module.type_support_@(module_name)


class Metaclass(type):
    """Metaclass of message '@(spec.base_type.type)'."""

    _CONVERT_FROM_PY = _convert_from_py
    _CONVERT_TO_PY = _convert_to_py
    _TYPE_SUPPORT = _type_support

    __constants = {
@[for constant in spec.constants]@
        '@(constant.name)': @constant_value_to_py(constant.type, constant.value),
@[end for]@
    }

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
            '@(field.name.upper())__DEFAULT': @value_to_py(field.type, field.default_value,
                                                           array_as_tuple=True),
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
        """Default value for message field '@(field.name)'."""
        return @value_to_py(field.type, field.default_value, array_as_tuple=True)
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
        assert all(['_' + key in self.__slots__ for key in kwargs.keys()]), \
            "Invalid arguments passed to constructor: %r" % kwargs.keys()
@[  for field in spec.fields]@
@[    if field.default_value]@
        self.@(field.name) = kwargs.get(
            '@(field.name)', @(spec.base_type.type).@(field.name.upper())__DEFAULT)
@[    else]@
@[      if not field.type.is_primitive_type() and (not field.type.is_array or field.type.array_size)]@
        from @(field.type.pkg_name).msg import @(field.type.type)
@[      end if]@
@[      if field.type.array_size]
        self.@(field.name) = kwargs.get(
            '@(field.name)',
            tuple([@(get_python_type(field.type))() for x in range(@(field.type.array_size))])
        )
@[      elif field.type.is_array]
        self.@(field.name) = kwargs.get('@(field.name)', list())
@[      elif field.type.type == 'byte']@
        self.@(field.name) = kwargs.get('@(field.name)', b'0')
@[      elif field.type.type == 'char']@
        self.@(field.name) = kwargs.get('@(field.name)', '\0')
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
@[for field in spec.fields]@

    @@property
    def @(field.name)(self):
        """Message field '@(field.name)'."""
        return self._@(field.name)

    @@@(field.name).setter
    def @(field.name)(self, value):
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
@[  elif not field.type.is_primitive_type()]@
@[  elif field.type.type == 'byte']@
        from collections import ByteString
@[  elif field.type.type in ['char']]@
        from collections import UserString
@[  end if]@
        assert isinstance(value, type(None)) or \
@[  if field.type.is_array]@
            ((isinstance(value, Sequence) or
              isinstance(value, Set) or
              isinstance(value, UserList)) and
             not isinstance(value, str) and
             not isinstance(value, UserString) and
@[    if field.type.array_size]@
@[      if field.type.is_upper_bound]@
             len(value) <= @(field.type.array_size) and
@[      else]@
             len(value) == @(field.type.array_size) and
@[      end if]@
@[    end if]@
             all([isinstance(v, @(get_python_type(field.type))) for v in value]))
@[  elif field.type.string_upper_bound]@
            ((isinstance(value, str) or isinstance(value, UserString)) and
             len(value) <= @(field.type.string_upper_bound))
@[  elif not field.type.is_primitive_type()]@
            isinstance(value, @(field.type.type))
@[  elif field.type.type == 'byte']@
            ((isinstance(value, bytes) or isinstance(value, ByteString)) and
             len(value) == 1)
@[  elif field.type.type == 'char']@
            ((isinstance(value, str) or isinstance(value, UserString)) and
             len(value) == 1)
@[  elif field.type.type in [
        'bool',
        'float32', 'float64',
        'int8', 'uint8',
        'int16', 'uint16',
        'int32', 'uint32',
        'int64', 'uint64',
        'string',
    ]]@
            isinstance(value, @(get_python_type(field.type)))
@[  else]@
            False
@[  end if]@
        self._@(field.name) = value
@[end for]@
