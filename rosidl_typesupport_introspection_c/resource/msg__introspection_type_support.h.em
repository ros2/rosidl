// generated from rosidl_typesupport_introspection_c/resource/msg__introspection_type_support.h.em
// generated code does not contain a copyright notice

@#######################################################################
@# EmPy template for generating <msg>__introspection_type_support.h files
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
    get_header_filename_from_msg_name(spec.base_type.type) + '__introspection_type_support_h']
header_guard_variable = '__'.join([x.upper() for x in header_guard_parts]) + '_'
function_prefix = '%s__%s__rosidl_typesupport_introspection_c' % (spec.base_type.pkg_name, subfolder)
}@
#ifndef @(header_guard_variable)
#define @(header_guard_variable)

#include <rosidl_generator_c/message_type_support.h>

#endif  // @(header_guard_variable)
