// generated from rosidl_generator_cpp/resource/msg__traits.hpp.em
// generated code does not contain a copyright notice

@#######################################################################
@# EmPy template for generating <msg>__traits.hpp files
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
header_guard_parts = [
    spec.base_type.pkg_name, subfolder,
    get_header_filename_from_msg_name(spec.base_type.type) + '__traits_hpp']
header_guard_variable = '__'.join([x.upper() for x in header_guard_parts]) + '_'
}@
#ifndef @(header_guard_variable)
#define @(header_guard_variable)

@{
cpp_namespace = '%s::%s::' % (spec.base_type.pkg_name, subfolder)
}@
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

#ifndef __ROSIDL_GENERATOR_CPP_TRAITS
#define __ROSIDL_GENERATOR_CPP_TRAITS

template<typename T>
inline const char * data_type();

template<typename T>
struct has_fixed_size : std::false_type {};

template<typename T>
struct has_bounded_size : std::false_type {};

#endif  // __ROSIDL_GENERATOR_CPP_TRAITS

#include "@(spec.base_type.pkg_name)/@(subfolder)/@(get_header_filename_from_msg_name(spec.base_type.type))__struct.hpp"

@{
fixed_template_strings = []
fixed = False

for field in spec.fields:
    if field.type.type == 'string':
        break
    if field.type.is_dynamic_array():
        break
    if not field.type.is_primitive_type():
        tmp_fixed_string = "has_fixed_size<{}::msg::{}>::value".format(
            field.type.pkg_name, field.type.type)
        if tmp_fixed_string not in fixed_template_strings:
            fixed_template_strings.append(tmp_fixed_string)
else:
    fixed = True

if fixed:
    fixed_template_string = ' && '.join(fixed_template_strings) if fixed_template_strings else 'true'
else:
    fixed_template_string = 'false'
}@

template<>
struct has_fixed_size<@(cpp_namespace)@(spec.base_type.type)>
  : std::integral_constant<bool, @(fixed_template_string)> {};

@{
bounded_template_strings = []
bounded = False

for field in spec.fields:
    if field.type.type == 'string' and field.type.string_upper_bound is None:
        break
    if field.type.is_dynamic_array() and not field.type.is_upper_bound:
        break
    if not field.type.is_primitive_type():
        tmp_bounded_string = "has_bounded_size<{}::msg::{}>::value".format(
            field.type.pkg_name, field.type.type)
        if tmp_bounded_string not in bounded_template_strings:
            bounded_template_strings.append(tmp_bounded_string)
else:
    bounded = True

if bounded:
    bounded_template_string = ' && '.join(bounded_template_strings) if bounded_template_strings else 'true'
else:
    bounded_template_string = 'false'
}@
template<>
struct has_bounded_size<@(cpp_namespace)@(spec.base_type.type)>
  : std::integral_constant<bool, @(bounded_template_string)> {};

template<>
inline const char * data_type<@(cpp_namespace)@(spec.base_type.type)>()
{
  return "@(cpp_namespace)@(spec.base_type.type)";
}

}  // namespace rosidl_generator_traits

#endif  // @(header_guard_variable)
