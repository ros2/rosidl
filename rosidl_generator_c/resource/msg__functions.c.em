// generated from rosidl_generator_c/resource/msg__functions.c.em
// generated code does not contain a copyright notice

@#######################################################################
@# EmPy template for generating <msg>__functions.c files
@#
@# Context:
@#  - spec (rosidl_parser.MessageSpecification)
@#    Parsed specification of the .msg file
@#  - subfolder (string)
@#    The subfolder / subnamespace of the message
@#    Either 'msg' or 'srv'
@#  - get_header_filename_from_msg_name (function)
@#######################################################################
@
@{
from rosidl_generator_c import get_typename_of_base_type
from rosidl_generator_c import primitive_value_to_c
from rosidl_generator_c import value_to_c

msg_typename = '%s__%s__%s' % (spec.base_type.pkg_name, subfolder, spec.base_type.type)
sequence_typename = '%s__Sequence' % msg_typename
}@
#include "@(spec.base_type.pkg_name)/@(subfolder)/@(get_header_filename_from_msg_name(spec.base_type.type))__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

@#######################################################################
@# include message dependencies
@#######################################################################
@{
from collections import OrderedDict
includes = OrderedDict()
for field in spec.fields:
    if field.type.is_primitive_type():
        if field.type.type == 'string':
            field_names = includes.setdefault('rosidl_generator_c/string_functions.h', [])
            field_names.append(field.name)
        else:
            if field.type.is_dynamic_array():
                field_names = includes.setdefault('rosidl_generator_c/primitives_sequence_functions.h', [])
                field_names.append(field.name)
    else:
        field_names = includes.setdefault(
            '%s/msg/%s__functions.h' %
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
@
@#######################################################################
@# message functions
@#######################################################################
bool
@(msg_typename)__init(@(msg_typename) * msg)
{
  if (!msg) {
    return false;
  }
@{
label_prefix = 'abort_init_'
last_label_index = 0
lines = []
abort_lines = []
for field in spec.fields:
    lines.append('// ' + field.name)
    if not field.type.is_array:
        # non-array field
        if field.type.is_primitive_type():
            if field.type.type == 'string':
                lines.append('if (!rosidl_generator_c__String__init(&msg->%s)) {' % field.name)
                lines.append('  %s__fini(msg);' % msg_typename)
                lines.append('  return false;')
                lines.append('}')
                if field.default_value is not None:
                    lines.append('{')
                    value = value_to_c(field.type, field.default_value)
                    lines.append('  bool success = rosidl_generator_c__String__assign(&msg->%s, %s);' % (field.name, value))
                    lines.append('  if (!success) {')
                    lines.append('    goto %s%s;' % (label_prefix, last_label_index))
                    abort_lines[0:0] = [
                      '  rosidl_generator_c__String__fini(&msg->%s);' % field.name,
                      '%s%d:' % (label_prefix, last_label_index),
                    ]
                    last_label_index += 1
                    lines.append('  }')
                    lines.append('}')
            elif field.default_value is not None:
                # set default value of primitive type
                lines.append('msg->%s = %s;' % (field.name, value_to_c(field.type, field.default_value)))

        else:
            # initialize the sub message
            lines.append('if (!%s__%s__%s__init(&msg->%s)) {' % (field.type.pkg_name, 'msg', field.type.type, field.name))
            lines.append('  %s__fini(msg);' % msg_typename)
            lines.append('  return false;')
            lines.append('}')
        # no default value for nested messages yet
    elif field.type.is_fixed_size_array():
        if field.type.is_primitive_type() and field.type.type != 'string':
            if field.default_value is not None:
                # set default value for each array element
                for i, default_value in enumerate(field.default_value):
                    lines.append('msg->%s[%d] = %s;' % (field.name, i, primitive_value_to_c(field.type.type, field.default_value[i])))
        if not field.type.is_primitive_type() or field.type.type == 'string':
            # initialize each array element
            lines.append('for (size_t i = 0; i < %d; ++i) {' % field.type.array_size)
            lines.append('  if (!%s__init(&msg->%s[i])) {' % (get_typename_of_base_type(field.type), field.name))
            lines.append('    %s__fini(msg);' % msg_typename)
            lines.append('    return false;')
            lines.append('  }')
            lines.append('}')

            if field.default_value is not None:
                for i, default_value in enumerate(field.default_value):
                    if field.type.type == 'string':
                        lines.append('{')
                        lines.append(
                            '  bool success = rosidl_generator_c__String__assign(&msg->%s[%d], %s);' % \
                            (field.name, i, primitive_value_to_c(field.type.type, field.default_value[i])))
                        lines.append('  if (!success) {')
                        lines.append('    goto %s%s;' % (label_prefix, last_label_index))
                        abort_lines[0:0] = [
                            '  rosidl_generator_c__String__fini(&msg->%s[%d]);' % (field.name, i),
                            '%s%d:' % (label_prefix, last_label_index),
                        ]
                        last_label_index += 1
                        lines.append('  }')
                        lines.append('}')

    else:  # dynamic array
        if field.default_value is None:
            # initialize the dynamic array with a capacity of zero
            lines.append('if (!%s__Sequence__init(&msg->%s, 0)) {' % (get_typename_of_base_type(field.type), field.name))
            lines.append('  %s__fini(msg);' % msg_typename)
            lines.append('  return false;')
            lines.append('}')
        else:
            # initialize the dynamic array with the number of default values
            lines.append('{')
            lines.append('  bool success = %s__Sequence__init(&msg->%s, %d);' % (get_typename_of_base_type(field.type), field.name, len(field.default_value)))
            lines.append('  if (!success) {')
            lines.append('    goto %s%d;' % (label_prefix, last_label_index))
            abort_lines[0:0] = [
                '  %s__Sequence__fini(&msg->%s);' % (get_typename_of_base_type(field.type), field.name),
                '%s%d:' % (label_prefix, last_label_index),
            ]
            last_label_index += 1
            lines.append('  }')
            lines.append('}')
            # set default value for each array element
            for i, default_value in enumerate(field.default_value):
                if field.type.type == 'string':
                    lines.append('{')
                    lines.append(
                        '  bool success = rosidl_generator_c__String__assign(&msg->%s.data[%d], %s);' % \
                        (field.name, i, primitive_value_to_c(field.type.type, field.default_value[i])))
                    lines.append('  if (!success) {')
                    lines.append('    goto %s%s;' % (label_prefix, last_label_index))
                    abort_lines[0:0] = [
                        '  rosidl_generator_c__String__fini(&msg->%s.data[%d]);' % (field.name, i),
                        '%s%d:' % (label_prefix, last_label_index),
                    ]
                    last_label_index += 1
                    lines.append('  }')
                    lines.append('}')
                else:
                    lines.append('msg->%s.data[%d] = %s;' % (field.name, i, primitive_value_to_c(field.type.type, field.default_value[i])))

for line in lines:
    print('  ' + line)
}@
  return true;
@{
if abort_lines:
    # remove lines before the first label since they are unreachable
    while not abort_lines[0].startswith(label_prefix):
        abort_lines.pop(0)
    for line in abort_lines:
        print(line)
    print('  return false;')
}@
}

void
@(msg_typename)__fini(@(msg_typename) * msg)
{
  if (!msg) {
    return;
  }
@{
lines = []
for field in spec.fields:
    lines.append('// ' + field.name)
    if not field.type.is_array:
        # non-array field
        if not field.type.is_primitive_type() or field.type.type == 'string':
            # finalize sub messages and strings
            lines.append('%s__fini(&msg->%s);' % (get_typename_of_base_type(field.type), field.name))

    elif field.type.is_fixed_size_array():
        if not field.type.is_primitive_type() or field.type.type == 'string':
            lines.append('for (size_t i = 0; i < %d; ++i) {' % field.type.array_size)
            # initialize each array element
            lines.append('  %s__fini(&msg->%s[i]);' % (get_typename_of_base_type(field.type), field.name))
            lines.append('}')

    else:
        # finalize the dynamic array
        lines.append('%s__Sequence__fini(&msg->%s);' % (get_typename_of_base_type(field.type), field.name))
for line in lines:
    print('  ' + line)
}@
}

@(msg_typename) *
@(msg_typename)__create()
{
  @(msg_typename) * msg = (@(msg_typename) *)malloc(sizeof(@(msg_typename)));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(@(msg_typename)));
  bool success = @(msg_typename)__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
@(msg_typename)__destroy(@(msg_typename) * msg)
{
  if (msg) {
    @(msg_typename)__fini(msg);
  }
  free(msg);
}


@#######################################################################
@# array functions
@#######################################################################
bool
@(sequence_typename)__init(@(sequence_typename) * array, size_t size)
{
  if (!array) {
    return false;
  }
  @(msg_typename) * data = NULL;
  if (size) {
    data = (@(msg_typename) *)calloc(size, sizeof(@(msg_typename)));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = @(msg_typename)__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        @(msg_typename)__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
@(sequence_typename)__fini(@(sequence_typename) * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      @(msg_typename)__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

@(sequence_typename) *
@(sequence_typename)__create(size_t size)
{
  @(sequence_typename) * array = (@(sequence_typename) *)malloc(sizeof(@(sequence_typename)));
  if (!array) {
    return NULL;
  }
  bool success = @(sequence_typename)__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
@(sequence_typename)__destroy(@(sequence_typename) * array)
{
  if (array) {
    @(sequence_typename)__fini(array);
  }
  free(array);
}
