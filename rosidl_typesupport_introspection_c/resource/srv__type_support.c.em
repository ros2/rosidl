// generated from rosidl_typesupport_introspection_c/resource/srv__type_support.c.em
// generated code does not contain a copyright notice

@#######################################################################
@# EmPy template for generating <srv>__type_support.c files
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
    get_header_filename_from_msg_name(spec.srv_name) + '__type_support_h']
header_guard_variable = '__'.join([x.upper() for x in header_guard_parts]) + '_'

function_prefix = '%s__%s__rosidl_typesupport_introspection_c' % (spec.pkg_name, subfolder)
}@
#ifndef @(header_guard_variable)
#define @(header_guard_variable)

#include <rosidl_generator_c/service_type_support_struct.h>
#include "@(spec.pkg_name)/msg/rosidl_typesupport_introspection_c__visibility_control.h"

#include "@(spec.pkg_name)/@(subfolder)/@(get_header_filename_from_msg_name(spec.srv_name))__rosidl_typesupport_introspection_c.h"
#include "@(spec.pkg_name)/@(subfolder)/@(get_header_filename_from_msg_name(spec.request.base_type.type))__rosidl_typesupport_introspection_c.h"
#include "@(spec.pkg_name)/@(subfolder)/@(get_header_filename_from_msg_name(spec.response.base_type.type))__rosidl_typesupport_introspection_c.h"

#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers @(function_prefix)__@(spec.srv_name)_service_members = {
  "@(spec.pkg_name)",  // package name
  "@(spec.srv_name)",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // @(function_prefix)__@(spec.request.base_type.type)_message_type_support_handle,
  NULL  // response message
  // @(function_prefix)__@(spec.request.base_type.type)_message_type_support_handle
};

static rosidl_service_type_support_t @(function_prefix)__@(spec.srv_name)_service_type_support_handle = {
  0,
  &@(function_prefix)__@(spec.srv_name)_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, @(spec.pkg_name), @(subfolder), @(spec.srv_name)_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, @(spec.pkg_name), @(subfolder), @(spec.srv_name)_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_@(spec.pkg_name)
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, @(spec.pkg_name), @(subfolder), @(spec.srv_name))() {
  if (!@(function_prefix)__@(spec.srv_name)_service_type_support_handle.typesupport_identifier) {
    @(function_prefix)__@(spec.srv_name)_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)@(function_prefix)__@(spec.srv_name)_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, @(spec.pkg_name), @(subfolder), @(spec.srv_name)_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, @(spec.pkg_name), @(subfolder), @(spec.srv_name)_Response)()->data;
  }

  return &@(function_prefix)__@(spec.srv_name)_service_type_support_handle;
}

#endif  // @(header_guard_variable)
