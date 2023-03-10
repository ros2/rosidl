@# Included from rosidl_generator_cpp/resource/idl__struct.hpp.em
@{
from rosidl_generator_c import type_hash_to_c_definition
}@
@{
TEMPLATE(
    'msg__struct.hpp.em',
    package_name=package_name, interface_path=interface_path,
    message=service.request_message, include_directives=include_directives,
    type_hash=type_hash['request_message'])
}@

@{
TEMPLATE(
    'msg__struct.hpp.em',
    package_name=package_name, interface_path=interface_path,
    message=service.response_message, include_directives=include_directives,
    type_hash=type_hash['response_message'])
}@

@{
TEMPLATE(
    'msg__struct.hpp.em',
    package_name=package_name, interface_path=interface_path,
    message=service.event_message, include_directives=include_directives,
    type_hash=type_hash['event_message'])
}@

@[for ns in service.namespaced_type.namespaces]@
namespace @(ns)
{

@[end for]@
@
struct @(service.namespaced_type.name)
{
@{
service_typename = '::'.join(service.namespaced_type.namespaced_name())
}@
  static constexpr const rosidl_type_hash_t TYPE_VERSION_HASH = @(type_hash_to_c_definition(type_hash['service']));

  using Request = @(service_typename)_Request;
  using Response = @(service_typename)_Response;
  using Event = @(service_typename)_Event;
};
@
@[for ns in reversed(service.namespaced_type.namespaces)]@

}  // namespace @(ns)
@[end for]@
