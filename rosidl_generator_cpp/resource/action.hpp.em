// generated from rosidl_generator_cpp/resource/action.hpp.em
// generated code does not contain a copyright notice

@#######################################################################
@# EmPy template for generating <action>.hpp files
@#
@# Context:
@#  - spec (rosidl_parser.ActionSpecification)
@#    Parsed specification of the .action file
@#  - subfolder (string)
@#    The subfolder / subnamespace of the message, usually 'action'
@#  - get_header_filename_from_msg_name (function)
@#######################################################################
@
@{
header_guard_parts = [
    spec.pkg_name, subfolder,
    get_header_filename_from_msg_name(spec.action_name) + '_hpp']
header_guard_variable = '__'.join([x.upper() for x in header_guard_parts]) + '_'
}@
#ifndef @(header_guard_variable)
#define @(header_guard_variable)

#include <@(spec.pkg_name)/@(subfolder)/@(get_header_filename_from_msg_name(spec.action_name))__struct.hpp>

#endif  // @(header_guard_variable)
