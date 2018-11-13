// generated from rosidl_generator_c/resource/action__type_support.h.em
// generated code does not contain a copyright notice

@#######################################################################
@# EmPy template for generating <action>__type_support.h files
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
    get_header_filename_from_msg_name(spec.action_name) + '__type_support_h']
header_guard_variable = '__'.join([x.upper() for x in header_guard_parts]) + '_'
}@

#ifndef @(header_guard_variable)
#define @(header_guard_variable)

#ifdef __cplusplus
extern "C"
{
#endif

#include "rosidl_generator_c/action_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"

#include "@(spec.pkg_name)/@(subfolder)/rosidl_generator_c__visibility_control.h"

/* *INDENT-OFF* */
// Forward declare the get type support functions for this type.
ROSIDL_GENERATOR_C_PUBLIC_@(spec.pkg_name)_ACTION
const rosidl_action_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__ACTION_SYMBOL_NAME(
  rosidl_typesupport_c, @(spec.pkg_name), @(subfolder), @(spec.action_name))();
/* *INDENT-ON* */

#ifdef __cplusplus
}
#endif

#endif  // @(header_guard_variable)
