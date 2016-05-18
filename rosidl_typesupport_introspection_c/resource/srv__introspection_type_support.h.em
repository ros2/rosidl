// generated from rosidl_typesupport_introspection_c/resource/srv__introspection_type_support.h.em
// generated code does not contain a copyright notice

@#######################################################################
@# EmPy template for generating <srv>__introspection_type_support.h files
@#
@# Context:
@#  - spec (rosidl_parser.MessageSpecification)
@#    Parsed specification of the .srv file
@#  - get_header_filename_from_msg_name (function)
@#######################################################################
@
@{
header_guard_parts = [
    spec.pkg_name, 'srv',
    get_header_filename_from_msg_name(spec.srv_name) + '__introspection_type_support_h']
header_guard_variable = '__'.join([x.upper() for x in header_guard_parts]) + '_'

function_prefix = '%s__%s__rosidl_typesupport_introspection_c' % (spec.pkg_name, 'srv')
}@
#ifndef @(header_guard_variable)
#define @(header_guard_variable)

#include <rosidl_generator_c/service_type_support.h>

#include "@(spec.pkg_name)/msg/rosidl_generator_c__visibility_control.h"

static rosidl_service_type_support_t @(function_prefix)__@(spec.srv_name)_service_type_support_handle;

#endif  // @(header_guard_variable)
