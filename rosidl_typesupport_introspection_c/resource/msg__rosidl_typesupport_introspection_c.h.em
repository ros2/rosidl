// generated from
// rosidl_typesupport_introspection_c/resource/msg__rosidl_typesupport_introspection_c.h.em
// generated code does not contain a copyright notice

@#######################################################################
@# EmPy template for generating
@# <msg>__rosidl_typesupport_introspection_c.h files
@#
@# Context:
@#  - spec (rosidl_parser.MessageSpecification)
@#    Parsed specification of the .msg file
@#  - subfolder (string)
@#    The subfolder / subnamespace of the message
@#    Either 'msg', 'srv' or 'action'
@#  - get_header_filename_from_msg_name (function)
@#######################################################################
@
@{
header_guard_parts = [
    spec.base_type.pkg_name, subfolder,
    get_header_filename_from_msg_name(spec.base_type.type) + '__rosidl_typesupport_introspection_c_h']
header_guard_variable = '__'.join([x.upper() for x in header_guard_parts]) + '_'
function_prefix = '%s__%s__rosidl_typesupport_introspection_c' % (spec.base_type.pkg_name, subfolder)
}@
#ifndef @(header_guard_variable)
#define @(header_guard_variable)

#include <rosidl_generator_c/message_type_support_struct.h>
#include <rosidl_typesupport_interface/macros.h>

#include "@(spec.base_type.pkg_name)/msg/rosidl_typesupport_introspection_c__visibility_control.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_C_PUBLIC_@(spec.base_type.pkg_name)
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, @(spec.base_type.pkg_name), @(subfolder), @(spec.msg_name))();

#ifdef __cplusplus
}
#endif

#endif  // @(header_guard_variable)
