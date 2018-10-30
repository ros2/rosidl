// generated from
// rosidl_typesupport_introspection_cpp/resource/msg__rosidl_typesupport_introspection_cpp.hpp.em
// generated code does not contain a copyright notice

@#######################################################################
@# EmPy template for generating
@# <msg>__rosidl_typesupport_introspection_cpp.hpp files
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
    get_header_filename_from_msg_name(spec.base_type.type) + '__rosidl_typesupport_introspection_cpp_hpp']
header_guard_variable = '__'.join([x.upper() for x in header_guard_parts]) + '_'
}@
#ifndef @(header_guard_variable)
#define @(header_guard_variable)

#include "rosidl_generator_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

#ifdef __cplusplus
extern "C"
{
#endif

// TODO(dirk-thomas) these visibility macros should be message package specific
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, @(spec.base_type.pkg_name), @(subfolder), @(spec.base_type.type))();

#ifdef __cplusplus
}
#endif

#endif  // @(header_guard_variable)
