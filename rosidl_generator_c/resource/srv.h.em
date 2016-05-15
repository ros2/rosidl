// generated from rosidl_generator_c/resource/srv.h.em
// generated code does not contain a copyright notice

@#######################################################################
@# EmPy template for generating <srv>-c.h files
@#
@# Context:
@#  - spec (rosidl_parser.ServiceSpecification)
@#    Parsed specification of the .srv file
@#  - get_header_filename_from_srv_name (function)
@#######################################################################
@
@{
header_guard_parts = [
    spec.pkg_name, 'srv',
    get_header_filename_from_msg_name(spec.srv_name) + '_h']
header_guard_variable = '__'.join([x.upper() for x in header_guard_parts]) + '_'
pkg = spec.pkg_name
}@
#ifndef @(header_guard_variable)
#define @(header_guard_variable)

#include "@(spec.pkg_name)/srv/@(get_header_filename_from_msg_name(spec.srv_name))__request.h"
#include "@(spec.pkg_name)/srv/@(get_header_filename_from_msg_name(spec.srv_name))__response.h"

// This header is provided by the rmw implementation specific type support
// package, and defines macros which expand to get type support functions.
#include "rosidl_generator_c/message_type_support.h"

// This header provides a definition for the rosidl_service_type_support_t struct.
#include "rosidl_generator_c/service_type_support.h"

#if defined(__cplusplus)
extern "C"
{
#endif

// Forward declare the get type support functions for this type.
ROSIDL_GENERATOR_C_PUBLIC_@(spec.pkg_name)
const rosidl_service_type_support_t *
  ROSIDL_GET_TYPE_SUPPORT_FUNCTION(@(spec.pkg_name), srv, @(spec.srv_name))();

#if defined(__cplusplus)
}
#endif

#endif  // @(header_guard_variable)
