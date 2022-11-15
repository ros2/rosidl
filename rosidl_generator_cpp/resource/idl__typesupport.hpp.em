// generated from rosidl_generator_cpp/resource/idl__typesupport.hpp.em
// with input from @(package_name):@(interface_path)
// generated code does not contain a copyright notice
@
@#######################################################################
@# EmPy template for generating <idl>__typesupport.hpp files
@#
@# Context:
@#  - package_name (string)
@#  - interface_path (Path relative to the directory named after the package)
@#  - content (IdlContent, list of elements, e.g. Messages or Services)
@#######################################################################
@{
from rosidl_pycommon import convert_camel_case_to_lower_case_underscore
include_parts = [package_name] + list(interface_path.parents[0].parts) + [
    'detail', convert_camel_case_to_lower_case_underscore(interface_path.stem)]
include_base = '/'.join(include_parts)

header_guard_variable = '__'.join([x.upper() for x in include_parts]) + \
    '__TYPESUPPORT_HPP_'

include_directives = set()

include_directives = {
  'rosidl_typesupport_cpp/service_type_support.hpp',
  include_base + '__struct.hpp'}
}@

#ifndef @(header_guard_variable)
#define @(header_guard_variable)

@[for header_file in include_directives]@
#include "@(header_file)"
@[end for]@

@#######################################################################
@# Handle service
@#######################################################################
@{
from rosidl_parser.definition import Service
}@
@[for service in content.get_elements_of_type(Service)]@
 @{
TEMPLATE(
    'srv__typesupport.hpp.em',
    package_name=package_name, interface_path=interface_path, service=service,
    include_directives=include_directives)
}@
@[end for]@

#endif  // @(header_guard_variable)
