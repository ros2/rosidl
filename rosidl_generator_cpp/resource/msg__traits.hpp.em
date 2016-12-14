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
from rosidl_generator_cpp import MSG_TYPE_TO_CPP

cpp_namespace = '%s::%s::' % (spec.base_type.pkg_name, subfolder)
}@
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

#ifndef __ROSIDL_GENERATOR_CPP_TRAITS
#define __ROSIDL_GENERATOR_CPP_TRAITS

template<typename T>
struct has_fixed_size : std::false_type {};

#endif  // __ROSIDL_GENERATOR_CPP_TRAITS

#include "@(spec.base_type.pkg_name)/@(subfolder)/@(get_header_filename_from_msg_name(spec.base_type.type))__struct.hpp"

@{
fixed_template_strings = []
fixed = True

for field in spec.fields:
   if field.type.type == 'string':
       fixed = False
       break
   elif field.type.is_array and (field.type.is_upper_bound or field.type.array_size is None):
       fixed = False
       break
   elif not field.type.is_primitive_type():
       fixed_template_strings.append("has_fixed_size<{}::msg::{}>::value".format(field.type.pkg_name, field.type.type))

if fixed:
    fixed_template_string = ' && '.join(fixed_template_strings) if fixed_template_strings else 'true'
else:
    fixed_template_string = 'false'
}@

template<>
struct has_fixed_size<@(cpp_namespace)@(spec.base_type.type)>
  : std::integral_constant<bool, @(fixed_template_string)>{};

}  // namespace rosidl_generator_traits

#endif  // @(header_guard_variable)
