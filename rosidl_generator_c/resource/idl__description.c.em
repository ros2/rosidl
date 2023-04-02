// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from @(package_name):@(interface_path)
// generated code does not contain a copyright notice
@
@#######################################################################
@# EmPy template for generating <idl>__description.c files
@#
@# Context:
@#  - package_name (string)
@#  - interface_path (Path relative to the directory named after the package)
@#  - content (IdlContent, list of elements, e.g. Messages or Services)
@#  - type_description_info (HashedTypeDescription.schema.json dict)
@#  - disable_description_codegen (bool)
@#######################################################################
@{
from rosidl_generator_type_description import extract_subinterface
from rosidl_parser.definition import Action
from rosidl_parser.definition import Service
from rosidl_pycommon import convert_camel_case_to_lower_case_underscore

type_description_msg = type_description_info['type_description_msg']

include_parts = [package_name] + list(interface_path.parents[0].parts) + [
    'detail', convert_camel_case_to_lower_case_underscore(interface_path.stem)]
include_base = '/'.join(include_parts)

implicit_type_descriptions = []
for service in content.get_elements_of_type(Service):
  implicit_type_descriptions.append(extract_subinterface(type_description_msg, 'request_message'))
  implicit_type_descriptions.append(extract_subinterface(type_description_msg, 'response_message'))
  implicit_type_descriptions.append(extract_subinterface(type_description_msg, 'event_message'))
for action in content.get_elements_of_type(Action):
  implicit_type_descriptions.append(extract_subinterface(type_description_msg, 'goal'))
  implicit_type_descriptions.append(extract_subinterface(type_description_msg, 'result'))
  implicit_type_descriptions.append(extract_subinterface(type_description_msg, 'feedback'))

  send_goal_service = extract_subinterface(type_description_msg, 'send_goal_service')
  implicit_type_descriptions.append(send_goal_service)
  implicit_type_descriptions.append(extract_subinterface(send_goal_service, 'request_message'))
  implicit_type_descriptions.append(extract_subinterface(send_goal_service, 'response_message'))
  implicit_type_descriptions.append(extract_subinterface(send_goal_service, 'event_message'))

  get_result_service = extract_subinterface(type_description_msg, 'get_result_service')
  implicit_type_descriptions.append(get_result_service)
  implicit_type_descriptions.append(extract_subinterface(get_result_service, 'request_message'))
  implicit_type_descriptions.append(extract_subinterface(get_result_service, 'response_message'))
  implicit_type_descriptions.append(extract_subinterface(get_result_service, 'event_message'))

  implicit_type_descriptions.append(extract_subinterface(type_description_msg, 'feedback_message'))
}@

#include "@(include_base)__functions.h"

@[if disable_description_codegen]@
@{
TEMPLATE(
  'empty__description.c.em',
  toplevel_type_description=type_description_msg,
  implicit_type_descriptions=implicit_type_descriptions)
}@
@[else]@
@{
TEMPLATE(
  'full__description.c.em',
  toplevel_type_description=type_description_msg,
  implicit_type_descriptions=implicit_type_descriptions)
}@
@[end if]@
