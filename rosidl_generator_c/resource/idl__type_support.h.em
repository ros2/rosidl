// generated from rosidl_generator_c/resource/idl__type_support.h.em
// generated code does not contain a copyright notice
@
@#######################################################################
@# EmPy template for generating <idl>__struct.h files
@#
@# Context:
@#  - package_name (string)
@#  - interface_path (Path relative to the directory named after the package)
@#  - interfaces (list of interfaces, either Messages or Services)
@#######################################################################
@{
from rosidl_cmake import convert_camel_case_to_lower_case_underscore
include_parts = [package_name] + list(interface_path.parents[0].parts) + \
    [convert_camel_case_to_lower_case_underscore(interface_path.stem)]
header_guard_variable = '__'.join([x.upper() for x in include_parts]) + \
    '__TYPE_SUPPORT_H_'
}@

#ifndef @(header_guard_variable)
#define @(header_guard_variable)

#ifdef __cplusplus
extern "C"
{
#endif

@#######################################################################
@# Handle message
@#######################################################################
@{
from rosidl_parser.definition import Message
}@
@[for message in content.get_elements_of_type(Message)]@
@{
TEMPLATE(
    'msg__type_support.h.em',
    package_name=package_name, message=message)
}@

@[end for]@
@
@#######################################################################
@# Handle service
@#######################################################################
@{
from rosidl_parser.definition import Service
}@
@[for service in content.get_elements_of_type(Service)]@
@{
TEMPLATE(
    'srv__type_support.h.em',
    package_name=package_name, service=service)
}@

@[end for]@
#ifdef __cplusplus
}
#endif

#endif  // @(header_guard_variable)
