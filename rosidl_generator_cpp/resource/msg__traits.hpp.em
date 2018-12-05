@# Included from rosidl_generator_cpp/resource/idl__traits.hpp.em
@{
from rosidl_parser.definition import Array
from rosidl_parser.definition import BaseString
from rosidl_parser.definition import NamespacedType
from rosidl_parser.definition import Sequence
from rosidl_parser.definition import UnboundedSequence

message_typename = '::'.join(message.structure.type.namespaces + [message.structure.type.name])
}@
@
@#<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
@# Collect necessary include directives for all members
@{
from collections import OrderedDict
from rosidl_cmake import convert_camel_case_to_lower_case_underscore
includes = OrderedDict()
for member in message.structure.members:
    type_ = member.type
    if isinstance(type_, Array):
        type_ = type_.basetype
    if isinstance(type_, NamespacedType):
        filename_prefix = convert_camel_case_to_lower_case_underscore(type_.name)
        if filename_prefix.endswith('__request'):
            filename_prefix = filename_prefix[:-9]
        elif filename_prefix.endswith('__response'):
            filename_prefix = filename_prefix[:-10]
        if filename_prefix.endswith('__goal'):
            filename_prefix = filename_prefix[:-6]
        elif filename_prefix.endswith('__result'):
            filename_prefix = filename_prefix[:-8]
        elif filename_prefix.endswith('__feedback'):
            filename_prefix = filename_prefix[:-10]
        member_names = includes.setdefault(
            '/'.join((type_.namespaces + [filename_prefix])) + '__traits.hpp', [])
        member_names.append(member.name)
}@
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
namespace rosidl_generator_traits
{

template<>
inline const char * data_type<@(message_typename)>()
{
  return "@(message_typename)";
}

@{
fixed_template_string = 'true'
fixed_template_strings = set()
for member in message.structure.members:
    type_ = member.type
    if isinstance(type_, Sequence):
        fixed_template_string = 'false'
        break
    if isinstance(type_, Array):
        type_ = type_.basetype
    if isinstance(type_, BaseString):
        fixed_template_string = 'false'
        break
    if isinstance(type_, NamespacedType):
        typename = '::'.join(type_.namespaces + [type_.name])
        fixed_template_strings.add('has_fixed_size<{typename}>::value'.format_map(locals()))
else:
    if fixed_template_strings:
        fixed_template_string = ' && '.join(sorted(fixed_template_strings))
}@
template<>
struct has_fixed_size<@(message_typename)>
  : std::integral_constant<bool, @(fixed_template_string)> {};

@{
bounded_template_string = 'true'
bounded_template_strings = set()
for member in message.structure.members:
    type_ = member.type
    if isinstance(type_, UnboundedSequence):
        bounded_template_string = 'false'
        break
    if isinstance(type_, Array):
        type_ = type_.basetype
    if isinstance(type_, BaseString) and type_.maximum_size is None:
        bounded_template_string = 'false'
        break
    if isinstance(type_, NamespacedType):
        typename = '::'.join(type_.namespaces + [type_.name])
        bounded_template_strings.add('has_bounded_size<{typename}>::value'.format_map(locals()))
else:
    if bounded_template_strings:
        bounded_template_string = ' && '.join(sorted(bounded_template_strings))
}@
template<>
struct has_bounded_size<@(message_typename)>
  : std::integral_constant<bool, @(bounded_template_string)> {};

}  // namespace rosidl_generator_traits
