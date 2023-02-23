// generated from rosidl_generator_cpp/resource/idl__description.hpp.em
// with input from @(package_name):@(interface_path)
// generated code does not contain a copyright notice
@
@#######################################################################
@# EmPy template for generating <idl>__description.hpp files
@#
@# Context:
@#  - package_name (string)
@#  - interface_path (Path relative to the directory named after the package)
@#  - content (IdlContent, list of elements, e.g. Messages or Services)
@#  - type_description_info (HashedTypeDescription.schema.json)
@#######################################################################
@{
from rosidl_parser.definition import Action
from rosidl_parser.definition import Message
from rosidl_parser.definition import Service
from rosidl_pycommon import convert_camel_case_to_lower_case_underscore

include_parts = [package_name] + list(interface_path.parents[0].parts) + [
    'detail', convert_camel_case_to_lower_case_underscore(interface_path.stem)]
include_base = '/'.join(include_parts)
header_guard_variable = '__'.join([x.upper() for x in include_parts]) + \
    '__DESCRIPTION_HPP_'
}@

#ifndef @(header_guard_variable)
#define @(header_guard_variable)

#include "@(include_base)__struct.hpp"

#include "rosidl_runtime_cpp/get_type_description_decl.hpp"
#include "rosidl_runtime_cpp/type_description/type_description__struct.hpp"

@[for message in content.get_elements_of_type(Message)]@
@{
TEMPLATE(
  'any__description.hpp.em',
  namespaced_type=message.structure.namespaced_type,
  type_description_msg=type_description_info['type_description_msg'])
}@

@[end for]@
@[for service in content.get_elements_of_type(Service)]@
@{
TEMPLATE(
  'srv__description.hpp.em',
  service=service,
  type_description_info=type_description_info)
}@

@[end for]@
@[for action in content.get_elements_of_type(Action)]@
@{
TEMPLATE(
  'action__description.hpp.em',
  action=action,
  type_description_info=type_description_info)
}@

@[end for]@

#endif  // @(header_guard_variable)
