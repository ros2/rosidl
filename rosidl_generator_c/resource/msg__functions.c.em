@# Included from rosidl_generator_c/resource/idl__functions.c.em
@{
from ast import literal_eval
from rosidl_parser.definition import AbstractNestedType
from rosidl_parser.definition import AbstractSequence
from rosidl_parser.definition import AbstractString
from rosidl_parser.definition import AbstractWString
from rosidl_parser.definition import Array
from rosidl_parser.definition import BasicType
from rosidl_parser.definition import AbstractGenericString
from rosidl_parser.definition import NamespacedType
from rosidl_generator_c import basetype_to_c
from rosidl_generator_c import idl_structure_type_sequence_to_c_typename
from rosidl_generator_c import idl_structure_type_to_c_include_prefix
from rosidl_generator_c import idl_structure_type_to_c_typename
from rosidl_generator_c import idl_type_to_c
from rosidl_generator_c import interface_path_to_string
from rosidl_generator_c import value_to_c
from rosidl_pycommon import convert_camel_case_to_lower_case_underscore

message_typename = idl_structure_type_to_c_typename(message.structure.namespaced_type)
array_typename = idl_structure_type_sequence_to_c_typename(
    message.structure.namespaced_type)

include_parts = [package_name] + list(interface_path.parents[0].parts) + [
    'detail', convert_camel_case_to_lower_case_underscore(interface_path.stem)]
include_base = '/'.join(include_parts)
}@
@#<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
@# Collect necessary include directives for all members
@{
from collections import OrderedDict
includes = OrderedDict()
for member in message.structure.members:
    if isinstance(member.type, AbstractSequence) and isinstance(member.type.value_type, BasicType):
        member_names = includes.setdefault(
            'rosidl_runtime_c/primitives_sequence_functions.h', [])
        member_names.append(member.name)
        continue
    type_ = member.type
    if isinstance(type_, AbstractNestedType):
        type_ = type_.value_type
    if isinstance(type_, AbstractString):
        member_names = includes.setdefault('rosidl_runtime_c/string_functions.h', [])
        member_names.append(member.name)
    elif isinstance(type_, AbstractWString):
        member_names = includes.setdefault(
            'rosidl_runtime_c/u16string_functions.h', [])
        member_names.append(member.name)
    elif isinstance(type_, NamespacedType):
        include_prefix = idl_structure_type_to_c_include_prefix(
          type_, 'detail')
        impl_header = include_prefix + '__functions_impl.h'
        # Skip if this is the current message's implementation header
        if impl_header != include_base + '__functions_impl.h':
            member_names = includes.setdefault(impl_header, [])
            member_names.append(member.name)
}@
@#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
@
@#<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
@[if includes]@

// Include directives for member types
@[    for header_file, member_names in includes.items()]@
@[        for member_name in member_names]@
// Member `@(member_name)`
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

@#######################################################################
@# message functions
@#######################################################################
bool
@(message_typename)__init(@(message_typename) * msg)
{
  return @(message_typename)__init_impl(msg);
}

void
@(message_typename)__fini(@(message_typename) * msg)
{
  @(message_typename)__fini_impl(msg);
}

bool
@(message_typename)__are_equal(const @(message_typename) * lhs, const @(message_typename) * rhs)
{
  return @(message_typename)__are_equal_impl(lhs, rhs);
}

bool
@(message_typename)__copy(
  const @(message_typename) * input,
  @(message_typename) * output)
{
  return @(message_typename)__copy_impl(input, output);
}

@(message_typename) *
@(message_typename)__create(void)
{
  return @(message_typename)__create_impl();
}

void
@(message_typename)__destroy(@(message_typename) * msg)
{
  @(message_typename)__destroy_impl(msg);
}


@#######################################################################
@# array functions
@#######################################################################
bool
@(array_typename)__init(@(array_typename) * array, size_t size)
{
  return @(array_typename)__init_impl(array, size);
}

void
@(array_typename)__fini(@(array_typename) * array)
{
  @(array_typename)__fini_impl(array);
}

@(array_typename) *
@(array_typename)__create(size_t size)
{
  return @(array_typename)__create_impl(size);
}

void
@(array_typename)__destroy(@(array_typename) * array)
{
  @(array_typename)__destroy_impl(array);
}

bool
@(array_typename)__are_equal(const @(array_typename) * lhs, const @(array_typename) * rhs)
{
  return @(array_typename)__are_equal_impl(lhs, rhs);
}

bool
@(array_typename)__copy(
  const @(array_typename) * input,
  @(array_typename) * output)
{
  return @(array_typename)__copy_impl(input, output);
}
