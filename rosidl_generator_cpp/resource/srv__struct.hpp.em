@# Included from rosidl_generator_cpp/resource/idl__struct.hpp.em
@{
service_includes = (
    'rcl_interfaces/msg/ServiceEvent.hpp',
)
}@

@{
TEMPLATE(
    'msg__struct.hpp.em',
    package_name=package_name, interface_path=interface_path,
    message=service.request_message, include_directives=include_directives)
}@

@{
TEMPLATE(
    'msg__struct.hpp.em',
    package_name=package_name, interface_path=interface_path,
    message=service.response_message, include_directives=include_directives)
}@

@[for header_file in service_includes]@
@[    if header_file in include_directives]@
// already included above
// @
@[    else]@
@{include_directives.add(header_file)}@
@[    end if]@
#include "@(header_file)"
@[end for]@

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
  using Request = @(service_typename)_Request;
  using Response = @(service_typename)_Response;
  using ServiceEvent = @(service_typename)_ServiceEvent;
};
@
@[for ns in reversed(service.namespaced_type.namespaces)]@

}  // namespace @(ns)
@[end for]@
