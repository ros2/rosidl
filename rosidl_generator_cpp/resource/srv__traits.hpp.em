// generated from rosidl_generator_cpp/resource/srv__traits.hpp.em
// generated code does not contain a copyright notice

@#######################################################################
@# EmPy template for generating <srv>__traits.hpp files
@#
@# Context:
@#  - spec (rosidl_parser.ServiceSpecification)
@#    Parsed specification of the .srv file
@#  - subfolder (string)
@#    The subfolder / subnamespace of the message
@#    Either 'srv' or 'action'
@#  - get_header_filename_from_msg_name (function)
@#######################################################################
@
@{
header_guard_parts = [
    spec.pkg_name, subfolder,
    get_header_filename_from_msg_name(spec.srv_name) + '__traits_hpp']
header_guard_variable = '__'.join([x.upper() for x in header_guard_parts]) + '_'
cpp_namespace = '%s::%s::' % (spec.pkg_name, subfolder)
}@

#include "@(spec.pkg_name)/@(subfolder)/@(get_header_filename_from_msg_name(spec.srv_name))__struct.hpp"

#ifndef @(header_guard_variable)
#define @(header_guard_variable)

namespace rosidl_generator_traits
{

#ifndef __ROSIDL_GENERATOR_CPP_TRAITS
#define __ROSIDL_GENERATOR_CPP_TRAITS

template<typename T>
struct has_fixed_size : std::false_type {};

template<typename T>
struct has_bounded_size : std::false_type {};

#endif  // __ROSIDL_GENERATOR_CPP_TRAITS

template<>
struct has_fixed_size<@(cpp_namespace)@(spec.srv_name)>
  : std::integral_constant<
    bool,
    has_fixed_size<@(cpp_namespace)@(spec.srv_name)_Request>::value &&
    has_fixed_size<@(cpp_namespace)@(spec.srv_name)_Response>::value
  >
{
};

template<>
struct has_bounded_size<@(cpp_namespace)@(spec.srv_name)>
  : std::integral_constant<
    bool,
    has_bounded_size<@(cpp_namespace)@(spec.srv_name)_Request>::value &&
    has_bounded_size<@(cpp_namespace)@(spec.srv_name)_Response>::value
  >
{
};

}  // namespace rosidl_generator_traits

#endif  // @(header_guard_variable)
