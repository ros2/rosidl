// generated from rosidl_generator_c/resource/msg__type_support.h.em
// generated code does not contain a copyright notice

@#######################################################################
@# EmPy template for generating <msg>__type_support.h files
@#
@# Context:
@#  - spec (rosidl_parser.MessageSpecification)
@#    Parsed specification of the .msg file
@#  - pkg (string)
@#    name of the containing package; equivalent to spec.base_type.pkg_name
@#  - msg (string)
@#    name of the message; equivalent to spec.msg_name
@#  - type (string)
@#    full type of the message; equivalent to spec.base_type.type
@#  - subfolder (string)
@#    The subfolder / subnamespace of the message
@#    Could be 'msg', 'srv' or 'action'
@#  - get_header_filename_from_msg_name (function)
@#######################################################################
@
@{
header_guard_parts = [
    spec.base_type.pkg_name, subfolder,
    get_header_filename_from_msg_name(spec.base_type.type) + '__type_support_h']
header_guard_variable = '__'.join([x.upper() for x in header_guard_parts]) + '_'

msg_typename = '%s__%s__%s' % (spec.base_type.pkg_name, subfolder, spec.base_type.type)
}@
#ifndef @(header_guard_variable)
#define @(header_guard_variable)

#ifdef __cplusplus
extern "C"
{
#endif

#include "rosidl_generator_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"

#include "@(spec.base_type.pkg_name)/msg/rosidl_generator_c__visibility_control.h"

// Forward declare the get type support functions for this type.
ROSIDL_GENERATOR_C_PUBLIC_@(spec.base_type.pkg_name)
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, @(pkg), @(subfolder), @(type))();

#ifdef __cplusplus
}
#endif

#endif  // @(header_guard_variable)
