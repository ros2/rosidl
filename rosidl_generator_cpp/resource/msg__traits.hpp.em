@# Included from rosidl_generator_cpp/resource/idl__traits.hpp.em
@{
from rosidl_parser.definition import ACTION_FEEDBACK_SUFFIX
from rosidl_parser.definition import ACTION_GOAL_SUFFIX
from rosidl_parser.definition import ACTION_RESULT_SUFFIX
from rosidl_parser.definition import Array
from rosidl_parser.definition import AbstractGenericString
from rosidl_parser.definition import BasicType
from rosidl_parser.definition import BoundedSequence
from rosidl_parser.definition import EMPTY_STRUCTURE_REQUIRED_MEMBER_NAME
from rosidl_parser.definition import NamespacedType
from rosidl_parser.definition import AbstractSequence
from rosidl_parser.definition import UnboundedSequence

message_typename = '::'.join(message.structure.namespaced_type.namespaced_name())
message_fully_qualified_name = '/'.join(message.structure.namespaced_type.namespaced_name())
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
    if isinstance(type_, (AbstractSequence, Array)):
        type_ = type_.value_type
    if isinstance(type_, NamespacedType):
        if (
            type_.name.endswith(ACTION_GOAL_SUFFIX) or
            type_.name.endswith(ACTION_RESULT_SUFFIX) or
            type_.name.endswith(ACTION_FEEDBACK_SUFFIX)
        ):
            typename = type_.name.rsplit('_', 1)[0]
        else:
            typename = type_.name
        member_names = includes.setdefault(
            '/'.join((type_.namespaces + ['detail', convert_camel_case_to_lower_case_underscore(typename)])) + '__traits.hpp', [])
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

inline void to_yaml(
  const @(message_typename) & msg,
  std::ostream & out, size_t indentation = 0)
{
@[if len(message.structure.members) == 1 and message.structure.members[0].name == EMPTY_STRUCTURE_REQUIRED_MEMBER_NAME]@
  (void)msg;
  (void)indentation;
  out << "null\n";
@[else]@
@[  for i, member in enumerate(message.structure.members)]@
@[    if i]@

@[    end if]@
  // member: @(member.name)
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
@[    if isinstance(member.type, BasicType)]@
    out << "@(member.name): ";
@[      if member.type.typename in ('octet', 'char', 'wchar')]@
    character_value_to_yaml(msg.@(member.name), out);
@[      else]@
    value_to_yaml(msg.@(member.name), out);
@[      end if]@
    out << "\n";
@[    elif isinstance(member.type, AbstractGenericString)]@
    out << "@(member.name): ";
    value_to_yaml(msg.@(member.name), out);
    out << "\n";
@[    elif isinstance(member.type, NamespacedType)]@
    out << "@(member.name):\n";
    to_yaml(msg.@(member.name), out, indentation + 2);
@[    elif isinstance(member.type, (AbstractSequence, Array))]@
    if (msg.@(member.name).size() == 0) {
      out << "@(member.name): []\n";
    } else {
      out << "@(member.name):\n";
      for (auto item : msg.@(member.name)) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
@[      if isinstance(member.type.value_type, BasicType)]@
        out << "- ";
@[        if member.type.value_type.typename in ('octet', 'char', 'wchar')]@
        character_value_to_yaml(item, out);
@[        else]@
        value_to_yaml(item, out);
@[        end if]@
        out << "\n";
@[      elif isinstance(member.type.value_type, AbstractGenericString)]@
        out << "- ";
        value_to_yaml(item, out);
        out << "\n";
@[      elif isinstance(member.type.value_type, NamespacedType)]@
        out << "-\n";
        to_yaml(item, out, indentation + 2);
@[      end if]@
      }
    }
@[    end if]@
  }
@[  end for]@
@[end if]@
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const @(message_typename) & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<@(message_typename)>()
{
  return "@(message_typename)";
}

template<>
inline const char * name<@(message_typename)>()
{
  return "@(message_fully_qualified_name)";
}

@{
fixed_template_string = 'true'
fixed_template_strings = set()
for member in message.structure.members:
    type_ = member.type
    if isinstance(type_, AbstractSequence):
        fixed_template_string = 'false'
        break
    if isinstance(type_, Array):
        type_ = type_.value_type
    if isinstance(type_, AbstractGenericString):
        fixed_template_string = 'false'
        break
    if isinstance(type_, NamespacedType):
        typename = '::'.join(type_.namespaced_name())
        fixed_template_strings.add(f'has_fixed_size<{typename}>::value')
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
    if isinstance(type_, (Array, BoundedSequence)):
        type_ = type_.value_type
    if isinstance(type_, AbstractGenericString) and not type_.has_maximum_size():
        bounded_template_string = 'false'
        break
    if isinstance(type_, NamespacedType):
        typename = '::'.join(type_.namespaced_name())
        bounded_template_strings.add(f'has_bounded_size<{typename}>::value')
else:
    if bounded_template_strings:
        bounded_template_string = ' && '.join(sorted(bounded_template_strings))
}@
template<>
struct has_bounded_size<@(message_typename)>
  : std::integral_constant<bool, @(bounded_template_string)> {};

template<>
struct is_message<@(message_typename)>
  : std::true_type {};

}  // namespace rosidl_generator_traits
