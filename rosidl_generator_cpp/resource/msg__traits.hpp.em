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
