// generated from rosidl_generator_c/resource/idl__struct.h.em
// generated code does not contain a copyright notice
@
@#######################################################################
@# EmPy template for generating <idl>__struct.h files
@#
@# Context:
@#  - package_name (string)
@#  - interface_path (Path relative to the directory named after the package)
@#  - interfaces (list of interfaces, either Messages or Services)
@#  - get_header_filename_from_msg_name (function)
@#######################################################################
@{
include_parts = [package_name] + list(interface_path.parents[0].parts) + \
    [get_header_filename_from_msg_name(interface_path.stem)]
header_guard_variable = '__'.join([x.upper() for x in include_parts]) + \
    '__STRUCT_H_'
}@

#ifndef @(header_guard_variable)
#define @(header_guard_variable)

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

@#######################################################################
@# Handle message
@#######################################################################
@{
from rosidl_parser.definition import Message
}@
@[for message in content.get_elements_of_type(Message)]@
@{
TEMPLATE(
    'msg__struct.h.em',
    package_name=package_name, interface_path=interface_path,
    message=message)
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
    'msg__struct.h.em',
    package_name=package_name, interface_path=interface_path,
    message=service.request_message)
}@

@{
TEMPLATE(
    'msg__struct.h.em',
    package_name=package_name, interface_path=interface_path,
    message=service.response_message)
}@

@[end for]@
#ifdef __cplusplus
}
#endif

#endif  // @(header_guard_variable)
