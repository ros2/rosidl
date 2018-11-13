// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// generated code does not contain a copyright notice
@
@#######################################################################
@# EmPy template for generating <idl>__struct.hpp files
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
    '__STRUCT_HPP_'

include_directives = set()
}@

#ifndef @(header_guard_variable)
#define @(header_guard_variable)

#include <rosidl_generator_cpp/bounded_vector.hpp>
#include <rosidl_generator_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

@#######################################################################
@# Handle message
@#######################################################################
@{
from rosidl_parser.definition import Message
}@
@[for message in content.get_elements_of_type(Message)]@
@{
TEMPLATE(
    'msg__struct.hpp.em',
    package_name=package_name, interface_path=interface_path,
    message=message, include_directives=include_directives)
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
    'srv__struct.hpp.em',
    package_name=package_name, interface_path=interface_path, service=service,
    include_directives=include_directives)
}@

@[end for]@
#endif  // @(header_guard_variable)
