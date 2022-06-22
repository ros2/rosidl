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
    package_name=package_name, message=service.service_event_message,
    include_directives=include_directives)
}@

@{header_files = (
    'rosidl_runtime_c/service_type_support_struct.h',
    'rcl_interfaces/msg/ServiceEvent.h'
}@

@[for header_file in header_files]@
@[    if header_file in include_directives]@
// already included above
// @
@[    else]@
@{include_directives.add(header_file)}@
@[    end if]@
#include "@(header_file)"
@[end for]@

// Forward declare the get type support functions for this type.
ROSIDL_GENERATOR_C_PUBLIC_@(package_name)
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(
  rosidl_typesupport_c,
  @(',\n  '.join(service.namespaced_type.namespaced_name()))
)();
