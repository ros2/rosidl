// generated from rosidl_generator_c/resource/srv.h.em
// generated code does not contain a copyright notice

@#######################################################################
@# EmPy template for generating <srv>-c.h files
@#
@# Context:
@#  - spec (rosidl_parser.ServiceSpecification)
@#    Parsed specification of the .srv file
@#######################################################################
@
@{
from rosidl_cmake import convert_camel_case_to_lower_case_underscore
header_guard_parts = [
    spec.pkg_name, 'srv',
    convert_camel_case_to_lower_case_underscore(spec.srv_name) + '_h']
header_guard_variable = '__'.join([x.upper() for x in header_guard_parts]) + '_'
pkg = spec.pkg_name
}@
#ifndef @(header_guard_variable)
#define @(header_guard_variable)

#include "@(spec.pkg_name)/srv/@(convert_camel_case_to_lower_case_underscore(spec.srv_name))__request.h"
#include "@(spec.pkg_name)/srv/@(convert_camel_case_to_lower_case_underscore(spec.srv_name))__response.h"

#include "rosidl_generator_c/message_type_support_struct.h"

// This header provides a definition for the rosidl_service_type_support_t struct.
#include "rosidl_generator_c/service_type_support_struct.h"

#include "rosidl_typesupport_interface/macros.h"

#include "@(spec.pkg_name)/msg/rosidl_generator_c__visibility_control.h"

#if defined(__cplusplus)
extern "C"
{
#endif

// Forward declare the get type support functions for this type.
ROSIDL_GENERATOR_C_PUBLIC_@(spec.pkg_name)
const rosidl_service_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_c, @(spec.pkg_name), @(spec.srv_name))();

#if defined(__cplusplus)
}
#endif

#endif  // @(header_guard_variable)
