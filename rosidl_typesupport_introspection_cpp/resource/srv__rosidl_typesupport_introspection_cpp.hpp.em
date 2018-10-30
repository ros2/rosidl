// generated from
// rosidl_typesupport_introspection_cpp/resource/srv__rosidl_typesupport_introspection_cpp.hpp.em
// generated code does not contain a copyright notice

@#######################################################################
@# EmPy template for generating
@# <srv>__rosidl_typesupport_introspection_cpp.hpp files
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
    get_header_filename_from_msg_name(spec.srv_name) + '__rosidl_typesupport_introspection_cpp_hpp']
header_guard_variable = '__'.join([x.upper() for x in header_guard_parts]) + '_'
}@
#ifndef @(header_guard_variable)
#define @(header_guard_variable)

#include <rosidl_generator_c/service_type_support_struct.h>
#include "rosidl_typesupport_interface/macros.h"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, @(spec.pkg_name), @(subfolder), @(spec.srv_name))();

#ifdef __cplusplus
}
#endif

#endif  // @(header_guard_variable)
