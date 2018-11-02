// generated from rosidl_generator_c/resource/msg__struct.h.em
// generated code does not contain a copyright notice

@#######################################################################
@# EmPy template for generating <msg>__struct.h files
@#
@# Context:
@#  - spec (rosidl_parser.MessageSpecification)
@#    Parsed specification of the .msg file
@#  - subfolder (string)
@#    The subfolder / subnamespace of the message
@#    Could be 'msg', 'srv' or 'action'
@#  - get_header_filename_from_msg_name (function)
@#######################################################################
@
@{
from rosidl_generator_c import msg_type_to_c
from rosidl_generator_c import MSG_TYPE_TO_C
from rosidl_generator_c import primitive_value_to_c

header_guard_parts = [
    spec.base_type.pkg_name, subfolder,
    get_header_filename_from_msg_name(spec.base_type.type) + '__struct_h']
header_guard_variable = '__'.join([x.upper() for x in header_guard_parts]) + '_'

msg_typename = '%s__%s__%s' % (spec.base_type.pkg_name, subfolder, spec.base_type.type)
sequence_typename = '%s__Sequence' % msg_typename
}@
#ifndef @(header_guard_variable)
#define @(header_guard_variable)

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

@#######################################################################
@# include message dependencies
@#######################################################################
@{
from collections import OrderedDict
includes = OrderedDict()
for field in spec.fields:
    if field.type.is_primitive_type():
        if field.type.type == 'string':
            field_names = includes.setdefault('rosidl_generator_c/string.h', [])
            field_names.append(field.name)
        else:
            if field.type.is_dynamic_array():
                field_names = includes.setdefault('rosidl_generator_c/primitives_sequence.h', [])
                field_names.append(field.name)
    else:
        field_names = includes.setdefault(
            '%s/msg/%s__struct.h' %
                (field.type.pkg_name, get_header_filename_from_msg_name(field.type.type)),
            [])
        field_names.append(field.name)
}@
@
@#######################################################################
@# constants defined in the message
@#######################################################################
@{
constants = []
for constant in spec.constants:
    if constant.type in ['byte', 'char', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']:
        constants.append((
            'enum',
            constant.name,
            '%s__%s' % (msg_typename, constant.name),
            primitive_value_to_c(constant.type, constant.value),
        ))
    else:
        constants.append((
            'static',
            constant.name,
            '%s %s' % (constant.type, msg_typename + '__' + constant.name),
            primitive_value_to_c(constant.type, constant.value),
        ))
}@
@[if includes]@
// include message dependencies
@[  for header_file, field_names in includes.items()]@
@[    for field_name in field_names]@
// @(field_name)
@[    end for]@
#include "@(header_file)"
@[  end for]@

@[end if]@
@[if constants]@
// constants defined in the message
@[  for constant_type, constant_name, key, value in constants]@
// @(constant_name)
@[    if constant_type == 'enum']@
enum
{
  @(key) = @(value)
};
@[    else]@
@{
(const_idl_type, const_c_name) = key.split()
if const_idl_type == 'string':
    const_c_type = 'char * const'
else:
    const_c_type = MSG_TYPE_TO_C[const_idl_type]
}@
static const @(const_c_type) @(const_c_name) = @(value);
@[    end if]@
@[  end for]@

@[end if]@
@
@#######################################################################
@# Constants for array fields with an upper bound
@#######################################################################
@{
upper_bounds = []
for field in spec.fields:
    if field.type.type == 'string' and field.type.string_upper_bound is not None:
        upper_bounds.append((
            field.name,
            '%s__%s__MAX_STRING_SIZE' % (msg_typename, field.name),
            field.type.string_upper_bound,
        ))
    if field.type.is_array and field.type.array_size and field.type.is_upper_bound:
        upper_bounds.append((
            field.name,
            '%s__%s__MAX_SIZE' % (msg_typename, field.name),
            field.type.array_size,
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
@
@#######################################################################
@# Struct of message
@#######################################################################
/// Struct of message @(spec.base_type.pkg_name)/@(spec.base_type.type)
typedef struct @(msg_typename)
{
@[for field in spec.fields]@
  @(msg_type_to_c(field.type, field.name));
@[end for]@
@[if not spec.fields]@
  bool _dummy;
@[end if]@
} @(msg_typename);

@#######################################################################
@# Struct for an array of messages
@#######################################################################
/// Struct for an array of messages
typedef struct @(sequence_typename)
{
  @(msg_typename) * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} @(sequence_typename);

#ifdef __cplusplus
}
#endif

#endif  // @(header_guard_variable)
