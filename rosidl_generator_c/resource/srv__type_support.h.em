@# Included from rosidl_generator_c/resource/idl__type_support.h.em
@{
TEMPLATE(
    'msg__type_support.h.em',
    package_name=package_name, message=service.request_message,
    include_directives=include_directives)
}@

@{
TEMPLATE(
    'msg__type_support.h.em',
    package_name=package_name, message=service.response_message,
    include_directives=include_directives)
}@

@{
TEMPLATE(
    'msg__type_support.h.em',
    package_name=package_name, message=service.event_message,
    include_directives=include_directives)
}@

@{header_file = 'rosidl_runtime_c/service_type_support_struct.h'}@
@[if header_file in include_directives]@
// already included above
// @
@[else]@
@{include_directives.add(header_file)}@
@[end if]@
#include "@(header_file)"

// Forward declare the get type support functions for this type.
ROSIDL_GENERATOR_C_PUBLIC_@(package_name)
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(
  rosidl_typesupport_c,
  @(',\n  '.join(service.namespaced_type.namespaced_name()))
)(void);

// Forward declare the function to create a service event message for this type.
ROSIDL_GENERATOR_C_PUBLIC_@(package_name)
void *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
  rosidl_typesupport_c,
  @(',\n  '.join(service.namespaced_type.namespaced_name()))
)(
  const rosidl_service_introspection_info_t * info,
  rcutils_allocator_t * allocator,
  const void * request_message,
  const void * response_message);

// Forward declare the function to destroy a service event message for this type.
ROSIDL_GENERATOR_C_PUBLIC_@(package_name)
bool
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
  rosidl_typesupport_c,
  @(',\n  '.join(service.namespaced_type.namespaced_name()))
)(
  void * event_msg,
  rcutils_allocator_t * allocator);
