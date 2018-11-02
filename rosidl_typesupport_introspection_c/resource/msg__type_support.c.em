// generated from rosidl_typesupport_introspection_c/resource/msg__type_support.c.em
// generated code does not contain a copyright notice

@#######################################################################
@# EmPy template for generating <msg>__type_support.c files
@#
@# Context:
@#  - spec (rosidl_parser.MessageSpecification)
@#    Parsed specification of the .msg file
@#  - subfolder (string)
@#    The subfolder / subnamespace of the message
@#    Either 'msg', 'srv' or 'action'
@#  - get_header_filename_from_msg_name (function)
@#######################################################################
@
@{
function_prefix = '%s__%s__rosidl_typesupport_introspection_c' % (spec.base_type.pkg_name, subfolder)
}@

// providing offsetof()
#include <stddef.h>

#include <@(spec.base_type.pkg_name)/@(subfolder)/@(get_header_filename_from_msg_name(spec.base_type.type))__rosidl_typesupport_introspection_c.h>
#include "@(spec.base_type.pkg_name)/msg/rosidl_typesupport_introspection_c__visibility_control.h"

#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"

#include "@(spec.base_type.pkg_name)/@(subfolder)/@(get_header_filename_from_msg_name(spec.base_type.type))__struct.h"

@#######################################################################
@# include message dependencies
@#######################################################################
@{
from collections import OrderedDict
includes = OrderedDict()
for field in spec.fields:
    if field.type.is_primitive_type() and field.type.is_dynamic_array():
        if field.type.type == 'string':
            field_names = includes.setdefault('rosidl_generator_c/string_functions.h', [])
        else:
            field_names = includes.setdefault('rosidl_generator_c/primitives_sequence_functions.h', [])
        field_names.append(field.name)
    if not field.type.is_primitive_type():
        field_names = includes.setdefault(
            '%s/msg/%s.h' %
            (field.type.pkg_name, get_header_filename_from_msg_name(field.type.type)),
            [])
        field_names.append(field.name)
        field_names = includes.setdefault(
            '%s/msg/%s__rosidl_typesupport_introspection_c.h' %
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
#ifdef __cplusplus
extern "C"
{
#endif

@
@#######################################################################
@# include message dependencies
@#######################################################################
@[if spec.fields]@
@[  for field in spec.fields]@
@[    if not field.type.is_primitive_type() and field.type.is_array]@
size_t @(function_prefix)__size_function__@(spec.base_type.type)__@(field.name)(
  const void * untyped_member)
{
@[      if field.type.array_size and not field.type.is_upper_bound]@
  (void)untyped_member;
  return @(field.type.array_size);
@[      else]@
  const @(field.type.pkg_name)__msg__@(field.type.type)__Sequence * member =
    (const @(field.type.pkg_name)__msg__@(field.type.type)__Sequence *)(untyped_member);
  return member->size;
@[      end if]@
}

const void * @(function_prefix)__get_const_function__@(spec.base_type.type)__@(field.name)(
  const void * untyped_member, size_t index)
{
@[      if field.type.array_size and not field.type.is_upper_bound]@
  const @(field.type.pkg_name)__msg__@(field.type.type) ** member =
    (const @(field.type.pkg_name)__msg__@(field.type.type) **)(untyped_member);
  return &(*member)[index];
@[      else]@
  const @(field.type.pkg_name)__msg__@(field.type.type)__Sequence * member =
    (const @(field.type.pkg_name)__msg__@(field.type.type)__Sequence *)(untyped_member);
  return &member->data[index];
@[      end if]@
}

void * @(function_prefix)__get_function__@(spec.base_type.type)__@(field.name)(
  void * untyped_member, size_t index)
{
@[      if field.type.array_size and not field.type.is_upper_bound]@
  @(field.type.pkg_name)__msg__@(field.type.type) ** member =
    (@(field.type.pkg_name)__msg__@(field.type.type) **)(untyped_member);
  return &(*member)[index];
@[      else]@
  @(field.type.pkg_name)__msg__@(field.type.type)__Sequence * member =
    (@(field.type.pkg_name)__msg__@(field.type.type)__Sequence *)(untyped_member);
  return &member->data[index];
@[      end if]@
}

@[      if not field.type.array_size or field.type.is_upper_bound]@
bool @(function_prefix)__resize_function__@(spec.base_type.type)__@(field.name)(
  void * untyped_member, size_t size)
{
  @(field.type.pkg_name)__msg__@(field.type.type)__Sequence * member =
    (@(field.type.pkg_name)__msg__@(field.type.type)__Sequence *)(untyped_member);
  @(field.type.pkg_name)__msg__@(field.type.type)__Sequence__fini(member);
  return @(field.type.pkg_name)__msg__@(field.type.type)__Sequence__init(member, size);
}

@[      end if]@
@[    end if]@
@[  end for]@
static rosidl_typesupport_introspection_c__MessageMember @(function_prefix)__@(spec.base_type.type)_message_member_array[@(len(spec.fields))] = {
@{
for index, field in enumerate(spec.fields):
    print('  {')

    # const char * name_
    print('    "%s",  // name' % field.name)
    if field.type.is_primitive_type():
        # uint8_t type_id_
        print('    rosidl_typesupport_introspection_c__ROS_TYPE_%s,  // type' % field.type.type.upper())
        # size_t string_upper_bound
        print('    %u,  // upper bound of string' % (field.type.string_upper_bound if field.type.string_upper_bound is not None else 0))
        # const rosidl_generator_c::MessageTypeSupportHandle * members_
        print('    NULL,  // members of sub message')
    else:
        # uint8_t type_id_
        print('    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type')
        # size_t string_upper_bound
        print('    0,  // upper bound of string')
        # const rosidl_message_type_support_t * members_
        print('    NULL,  // members of sub message (initialized later)')
    # bool is_array_
    print('    %s,  // is array' % ('true' if field.type.is_array else 'false'))
    # size_t array_size_
    print('    %u,  // array size' % (field.type.array_size if field.type.array_size else 0))
    # bool is_upper_bound_
    print('    %s,  // is upper bound' % ('true' if field.type.is_upper_bound else 'false'))
    # unsigned long offset_
    print('    offsetof(%s__%s__%s, %s),  // bytes offset in struct' % (spec.base_type.pkg_name, subfolder, spec.base_type.type, field.name))
    # void * default_value_
    print('    NULL,  // default value')  # TODO default value to be set

    function_suffix = '%s__%s' % (spec.base_type.type, field.name) if not field.type.is_primitive_type() and field.type.is_array else None

    # size_t(const void *) size_function
    print('    %s,  // size() function pointer' % ('%s__size_function__%s' % (function_prefix, function_suffix) if function_suffix else 'NULL'))
    # const void *(const void *, size_t) get_const_function
    print('    %s,  // get_const(index) function pointer' % ('%s__get_const_function__%s' % (function_prefix, function_suffix) if function_suffix else 'NULL'))
    # void *(void *, size_t) get_function
    print('    %s,  // get(index) function pointer' % ('%s__get_function__%s' % (function_prefix, function_suffix) if function_suffix else 'NULL'))
    # void(void *, size_t) resize_function
    print('    %s  // resize(index) function pointer' % ('%s__resize_function__%s' % (function_prefix, function_suffix) if function_suffix and (not field.type.array_size or field.type.is_upper_bound) else 'NULL'))

    if index < len(spec.fields) - 1:
        print('  },')
    else:
        print('  }')
}@
};

@[end if]@
static const rosidl_typesupport_introspection_c__MessageMembers @(function_prefix)__@(spec.base_type.type)_message_members = {
  "@(spec.base_type.pkg_name)",  // package name
  "@(spec.base_type.type)",  // message name
  @(len(spec.fields)),  // number of fields
  sizeof(@(spec.base_type.pkg_name)__@(subfolder)__@(spec.base_type.type)),
@[if spec.fields]@
  @(function_prefix)__@(spec.base_type.type)_message_member_array  // message members
@[else]@
  0  // message members
@[end if]@
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t @(function_prefix)__@(spec.base_type.type)_message_type_support_handle = {
  0,
  &@(function_prefix)__@(spec.base_type.type)_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_@(spec.base_type.pkg_name)
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, @(spec.base_type.pkg_name), @(subfolder), @(spec.msg_name))() {
@[for i, field in enumerate(spec.fields)]@
@[    if not field.type.is_primitive_type()]@
  @(function_prefix)__@(spec.base_type.type)_message_member_array[@(i)].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, @(field.type.pkg_name), msg, @(field.type.type))();
@[    end if]@
@[end for]@
  if (!@(function_prefix)__@(spec.base_type.type)_message_type_support_handle.typesupport_identifier) {
    @(function_prefix)__@(spec.base_type.type)_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &@(function_prefix)__@(spec.base_type.type)_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
