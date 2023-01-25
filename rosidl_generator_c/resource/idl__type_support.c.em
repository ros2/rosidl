// generated from rosidl_generator_c/resource/idl__type_support.c.em
// with input from @(package_name):@(interface_path)
// generated code does not contain a copyright notice
@
@#######################################################################
@# EmPy template for generating <idl>__type_support.c files
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

include_directives = {
  'rosidl_typesupport_interface/macros.h',
  include_base + '__type_support.h',
  include_base + '__struct.h',
  include_base + '__functions.h'}
}@

#include <string.h>

@[for header_file in include_directives]@
#include "@(header_file)"
@[end for]@

#ifdef __cplusplus
extern "C"
{
#endif

@#######################################################################
@# Handle service
@#######################################################################
@{
from rosidl_parser.definition import Service
}@
@[for service in content.get_elements_of_type(Service)]@
@{
TEMPLATE(
    'srv__type_support.c.em',
    package_name=package_name, service=service,
    interface_path=interface_path, include_directives=include_directives)
}@

@[end for]@
@
@#######################################################################
@# Handle action
@#######################################################################
@{
from rosidl_parser.definition import Action
}@
@[for action in content.get_elements_of_type(Action)]@
@{
TEMPLATE(
    'action__type_support.c.em',
    package_name=package_name, action=action,
    interface_path=interface_path, include_directives=include_directives)
}@

@[end for]@
#ifdef __cplusplus
}
#endif
