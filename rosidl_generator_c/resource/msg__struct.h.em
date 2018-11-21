@# Included from rosidl_generator_c/resource/idl__struct.h.em
@{
from rosidl_parser.definition import BaseString
from rosidl_parser.definition import BasicType
from rosidl_parser.definition import BoundedSequence
from rosidl_parser.definition import NamespacedType
from rosidl_parser.definition import NestedType
from rosidl_parser.definition import Sequence
from rosidl_parser.definition import String
from rosidl_parser.definition import WString
from rosidl_generator_c import basetype_to_c
from rosidl_generator_c import idl_declaration_to_c
from rosidl_generator_c import idl_structure_type_sequence_to_c_typename
from rosidl_generator_c import idl_structure_type_to_c_include_prefix
from rosidl_generator_c import idl_structure_type_to_c_typename
from rosidl_generator_c import interface_path_to_string
from rosidl_generator_c import value_to_c
}@
@#<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
@# Collect necessary include directives for all members
@{
from collections import OrderedDict
includes = OrderedDict()
for member in message.structure.members:
    if isinstance(member.type, Sequence) and isinstance(member.type.basetype, BasicType):
        member_names = includes.setdefault(
            'rosidl_generator_c/primitives_sequence.h', [])
        member_names.append(member.name)
        continue
    type_ = member.type
    if isinstance(type_, NestedType):
        type_ = type_.basetype
    if isinstance(type_, String):
        member_names = includes.setdefault('rosidl_generator_c/string.h', [])
        member_names.append(member.name)
    elif isinstance(type_, WString):
        member_names = includes.setdefault(
            'rosidl_generator_c/u16string.h', [])
        member_names.append(member.name)
    elif isinstance(type_, NamespacedType):
        include_prefix = idl_structure_type_to_c_include_prefix(type_)
        if include_prefix.endswith('__request'):
            include_prefix = include_prefix[:-9]
        elif include_prefix.endswith('__response'):
            include_prefix = include_prefix[:-10]
        if include_prefix.endswith('__goal'):
            include_prefix = include_prefix[:-6]
        elif include_prefix.endswith('__result'):
            include_prefix = include_prefix[:-8]
        elif include_prefix.endswith('__feedback'):
            include_prefix = include_prefix[:-10]
        member_names = includes.setdefault(
            include_prefix + '__struct.h', [])
        member_names.append(member.name)
}@
@#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

@#<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// Constants defined in the message
@[for constant in message.constants.values()]@

/// Constant '@(constant.name)'.
@[    if isinstance(constant.type, BasicType)]@
@[        if constant.type.type in (
              'short', 'unsigned short', 'long', 'unsigned long', 'long long', 'unsigned long long',
              'char', 'wchar', 'octet',
              'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64',
          )]@
enum
{
  @(idl_structure_type_to_c_typename(message.structure.type))__@(constant.name) = @(value_to_c(constant.type, constant.value))
};
@[        elif constant.type.type in ('float', 'double', 'long double', 'boolean')]@
static const @(basetype_to_c(constant.type)) @(idl_structure_type_to_c_typename(message.structure.type))__@(constant.name) = @(value_to_c(constant.type, constant.value));
@[        else]@
@{assert False, 'Unhandled basic type: ' + str(constant.type)}@
@[        end if]@
@[    elif isinstance(constant.type, String)]@
static const char * const @(idl_structure_type_to_c_typename(message.structure.type))__@(constant.name) = @(value_to_c(constant.type, constant.value));
@[    end if]@
@[end for]@
@#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
@
@#<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
@[if includes]@

// Include directives for member types
@[    for header_file, member_names in includes.items()]@
@[        for member_name in member_names]@
// Member '@(member_name)'
@[        end for]@
@[        if header_file in include_directives]@
// already included above
// @
@[        else]@
@{include_directives.add(header_file)}@
@[        end if]@
#include "@(header_file)"
@[    end for]@
@[end if]@
@#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
@
@#<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
@# Constants for array and string fields with an upper bound
@{
upper_bounds = []
for member in message.structure.members:
    type_ = member.type
    if isinstance(type_, BoundedSequence):
        upper_bounds.append((
            member.name,
            '%s__%s__MAX_SIZE' % (idl_structure_type_to_c_typename(message.structure.type), member.name),
            type_.upper_bound,
        ))
    if isinstance(type_, NestedType):
        type_ = type_.basetype
    if isinstance(type_, BaseString) and type_.maximum_size is not None:
        upper_bounds.append((
            member.name,
            '%s__%s__MAX_STRING_SIZE' % (idl_structure_type_to_c_typename(message.structure.type), member.name),
            type_.maximum_size,
        ))
}@
@[if upper_bounds]@

// constants for array fields with an upper bound
@[  for field_name, enum_name, enum_value in upper_bounds]@
// @(field_name)
enum
{
  @(enum_name) = @(enum_value)
};
@[  end for]@
@[end if]@
@#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

@#<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// Struct defined in @(interface_path_to_string(interface_path)) in the package @(package_name).
typedef struct @(idl_structure_type_to_c_typename(message.structure.type))
{
@[for member in message.structure.members]@
  @(idl_declaration_to_c(member.type, member.name));
@[end for]@
} @(idl_structure_type_to_c_typename(message.structure.type));
@#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

@#<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// Struct for a sequence of @(idl_structure_type_to_c_typename(message.structure.type)).
typedef struct @(idl_structure_type_sequence_to_c_typename(message.structure.type))
{
  @(idl_structure_type_to_c_typename(message.structure.type)) * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} @(idl_structure_type_sequence_to_c_typename(message.structure.type));
