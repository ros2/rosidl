@# Included from rosidl_generator_c/resource/idl__struct.h.em
@{
from rosidl_parser.definition import BasicType
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
            'rosidl_generator_c/primitives_array.h', [])
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
        member_names = includes.setdefault(
            idl_structure_type_to_c_include_prefix(type_) + '__struct.h', [])
        member_names.append(member.name)
}@
@#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

@#<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// Constants defined in the message
@[for constant in message.constants.values()]@

/// Constant `@(constant.name)`.
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
// Member `@(member_name)`
@[        end for]@
#include "@(header_file)"
@[    end for]@
@[end if]@
@#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
@
@
@
@#######################################################################
@# Constants for array fields with an upper bound
@#######################################################################
@#@{
@#upper_bounds = []
@#for field in spec.fields:
@#    if field.type.type == 'string' and field.type.string_upper_bound is not None:
@#        upper_bounds.append((
@#            field.name,
@#            '%s__%s__MAX_STRING_SIZE' % (msg_typename, field.name),
@#            field.type.string_upper_bound,
@#        ))
@#    if field.type.is_array and field.type.array_size and field.type.is_upper_bound:
@#        upper_bounds.append((
@#            field.name,
@#            '%s__%s__MAX_SIZE' % (msg_typename, field.name),
@#            field.type.array_size,
@#        ))
@#}@
@#@[if upper_bounds]@
@#// constants for array fields with an upper bound
@#@[  for field_name, enum_name, enum_value in upper_bounds]@
@#// @(field_name)
@#enum
@#{
@#  @(enum_name) = @(enum_value)
@#};
@#@[  end for]@
@#
@#@[end if]@
@
@

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
