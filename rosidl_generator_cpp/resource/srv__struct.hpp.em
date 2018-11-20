@# Included from rosidl_generator_cpp/resource/idl__struct.hpp.em
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

@[for ns in service.structure_type.namespaces]@
namespace @(ns)
{

@[end for]@
@
struct @(service.structure_type.name)
{
@{
service_typename = '::'.join(service.structure_type.namespaces + [service.structure_type.name])
}@
  using Request = @(service_typename)_Request;
  using Response = @(service_typename)_Response;
};
@
@[for ns in reversed(service.structure_type.namespaces)]@

}  // namespace @(ns)
@[end for]@
