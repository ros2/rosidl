@# Included from rosidl_generator_c/resource/idl__type_support.h.em
@{header_file = 'rosidl_generator_c/service_type_support_struct.h'}@
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
  @(',\n  '.join(service.structure_type.namespaces + [service.structure_type.name]))
)();