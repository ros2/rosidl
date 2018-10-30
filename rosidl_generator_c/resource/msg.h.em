// generated from rosidl_generator_c/resource/msg.h.em
// generated code does not contain a copyright notice

@#######################################################################
@# EmPy template for generating <msg>-c.h files
@#
@# Context:
@#  - spec (rosidl_parser.MessageSpecification)
@#    Parsed specification of the .msg file
@#  - subfolder (string)
@#    The subfolder / subnamespace of the message
@#    Could be 'msg', 'srv' or 'action'
@#  - get_header_filename_from_msg_name (function)
@#######################################################################
@
@{
header_guard_parts = [
    spec.base_type.pkg_name, subfolder,
    get_header_filename_from_msg_name(spec.base_type.type) + '_h']
header_guard_variable = '__'.join([x.upper() for x in header_guard_parts]) + '_'
pkg = spec.base_type.pkg_name
type = spec.base_type.type
}@
#ifndef @(header_guard_variable)
#define @(header_guard_variable)

#include "@(pkg)/@(subfolder)/@(get_header_filename_from_msg_name(type))__struct.h"
#include "@(pkg)/@(subfolder)/@(get_header_filename_from_msg_name(type))__functions.h"
#include "@(pkg)/@(subfolder)/@(get_header_filename_from_msg_name(type))__type_support.h"

#endif  // @(header_guard_variable)
