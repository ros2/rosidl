// generated from rosidl_typesupport_introspection_cpp/resource/srv__type_support.cpp.em
// generated code does not contain a copyright notice

@#######################################################################
@# EmPy template for generating <srv>__type_support.cpp files
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
#include <rosidl_generator_c/service_type_support_struct.h>
#include <rosidl_typesupport_cpp/message_type_support.hpp>
#include <rosidl_typesupport_cpp/service_type_support.hpp>
#include "rosidl_typesupport_interface/macros.h"

#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

#include "@(spec.pkg_name)/@(subfolder)/@(get_header_filename_from_msg_name(spec.srv_name))__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/service_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/service_type_support_decl.hpp"

#include "@(spec.pkg_name)/@(subfolder)/@(get_header_filename_from_msg_name(spec.request.base_type.type))__struct.hpp"
#include "@(spec.pkg_name)/@(subfolder)/@(get_header_filename_from_msg_name(spec.response.base_type.type))__struct.hpp"

namespace @(spec.pkg_name)
{

namespace @(subfolder)
{

namespace rosidl_typesupport_introspection_cpp
{

// this is intentionally not const to allow initialization later to prevent an initialization race
static ::rosidl_typesupport_introspection_cpp::ServiceMembers @(spec.srv_name)_service_members = {
  "@(spec.pkg_name)",  // package name
  "@(spec.srv_name)",  // service name
  // these two fields are initialized below on the first access
  // see get_service_type_support_handle<@(spec.pkg_name)::@(spec.srv_name)>()
  nullptr,  // request message
  nullptr  // response message
};

static const rosidl_service_type_support_t @(spec.srv_name)_service_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &@(spec.srv_name)_service_members,
  get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace @(subfolder)

}  // namespace @(spec.pkg_name)


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<@(spec.pkg_name)::@(subfolder)::@(spec.srv_name)>()
{
  // get a handle to the value to be returned
  auto service_type_support =
    &::@(spec.pkg_name)::@(subfolder)::rosidl_typesupport_introspection_cpp::@(spec.srv_name)_service_type_support_handle;
  // get a non-const and properly typed version of the data void *
  auto service_members = const_cast<::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
    static_cast<const ::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
      service_type_support->data));
  // make sure that both the request_members_ and the response_members_ are initialized
  // if they are not, initialize them
  if (
    service_members->request_members_ == nullptr ||
    service_members->response_members_ == nullptr)
  {
    // initialize the request_members_ with the static function from the external library
    service_members->request_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::@(spec.pkg_name)::@(subfolder)::@(spec.request.base_type.type)
      >()->data
      );
    // initialize the response_members_ with the static function from the external library
    service_members->response_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::@(spec.pkg_name)::@(subfolder)::@(spec.response.base_type.type)
      >()->data
      );
  }
  // finally return the properly initialized service_type_support handle
  return service_type_support;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, @(spec.pkg_name), @(subfolder), @(spec.srv_name))() {
  return ::rosidl_typesupport_introspection_cpp::get_service_type_support_handle<@(spec.pkg_name)::@(subfolder)::@(spec.srv_name)>();
}

#ifdef __cplusplus
}
#endif
