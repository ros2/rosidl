// generated from rosidl_generator_c/resource/msg__bounds.h.em
// generated code does not contain a copyright notice

@#######################################################################
@# EmPy template for generating <msg>__bounds.h files
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
    get_header_filename_from_msg_name(spec.base_type.type) + '__bounds_h']
header_guard_variable = '__'.join([x.upper() for x in header_guard_parts]) + '_'

msg_typename = '%s__%s__%s' % (spec.base_type.pkg_name, subfolder, spec.base_type.type)
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
            field_names = includes.setdefault('rosidl_generator_c/string_bounds.h', [])
            field_names.append(field.name)
    else:
        field_names = includes.setdefault(
            '%s/msg/%s__bounds.h' %
                (field.type.pkg_name, get_header_filename_from_msg_name(field.type.type)),
            [])
        field_names.append(field.name)
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

@{
unbounded_fields = []
for field in spec.fields:
    field.is_string_array = False
    field.is_primitive_array = False
    if field.type.is_array and not field.type.array_size and not field.type.is_upper_bound:
        if field.type.type == 'string':
            field.is_string_array = True
        elif field.type.is_primitive_type():
            field.is_primitive_array = True
        else:
            field.is_compound_array = True
        unbounded_fields.append(field)
}@


@#######################################################################
@# Struct of message bounds
@#######################################################################
/// Struct of message bounds @(spec.base_type.pkg_name)/@(spec.base_type.type)
typedef struct @(msg_typename)__bounds
{
@[for field in unbounded_fields]@
@[  if field.is_string_array]@
  size_t @(field.name)__length;
  rosidl_generator_c__String__bounds @(field.name)__bounds;
@[  elif field.is_primitive_array]@
  size_t @(field.name)__length;
@[ elif field.is_compound_array]@
  size_t @(field.name)__length;
  @('%s__msg__%s__bounds' % (field.type.pkg_name, field.type.type)) @(field.name)__bounds;
@[ end if]@
@[end for]@
} @(msg_typename)__bounds;

#ifdef __cplusplus
}
#endif

#endif  // @(header_guard_variable)



