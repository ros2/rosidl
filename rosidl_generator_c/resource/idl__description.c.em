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
@#  - type_source_file (absolute Path to original source definition file for this interface)
@#  - disable_description_codegen (bool)
@#######################################################################
@{
from rosidl_generator_c import type_hash_to_c_definition
from rosidl_generator_type_description import extract_subinterface
from rosidl_generator_type_description import GET_HASH_FUNC
from rosidl_parser.definition import Action
from rosidl_parser.definition import Service
from rosidl_pycommon import convert_camel_case_to_lower_case_underscore

type_description_msg = type_description_info['type_description_msg']
hash_lookup = {
  val['type_name']: val['hash_string']
  for val in type_description_info['type_hashes']
}

include_parts = [package_name] + list(interface_path.parents[0].parts) + [
    'detail', convert_camel_case_to_lower_case_underscore(interface_path.stem)]
include_base = '/'.join(include_parts)

implicit_type_descriptions = []
toplevel_type_description = (type_description_msg, 'message')
for service in content.get_elements_of_type(Service):
  toplevel_type_description = (type_description_msg, 'service')
  implicit_type_descriptions.extend([
    (extract_subinterface(type_description_msg, 'request_message'), 'message'),
    (extract_subinterface(type_description_msg, 'response_message'), 'message'),
    (extract_subinterface(type_description_msg, 'event_message'), 'message'),
  ])
for action in content.get_elements_of_type(Action):
  toplevel_type_description = (type_description_msg, 'action')
  send_goal_service = extract_subinterface(type_description_msg, 'send_goal_service')
  get_result_service = extract_subinterface(type_description_msg, 'get_result_service')
  implicit_type_descriptions.extend([
    (extract_subinterface(type_description_msg, 'goal'), 'message'),
    (extract_subinterface(type_description_msg, 'result'), 'message'),
    (extract_subinterface(type_description_msg, 'feedback'), 'message'),

    (send_goal_service, 'service'),
    (extract_subinterface(send_goal_service, 'request_message'), 'message'),
    (extract_subinterface(send_goal_service, 'response_message'), 'message'),
    (extract_subinterface(send_goal_service, 'event_message'), 'message'),

    (get_result_service, 'service'),
    (extract_subinterface(get_result_service, 'request_message'), 'message'),
    (extract_subinterface(get_result_service, 'response_message'), 'message'),
    (extract_subinterface(get_result_service, 'event_message'), 'message'),

    (extract_subinterface(type_description_msg, 'feedback_message'), 'message'),
  ])
}@

#include "@(include_base)__functions.h"

@#<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
@# Define get_type_hash functions
@[for type_description_msg, interface_type in [toplevel_type_description] + implicit_type_descriptions]@
@{
typename = type_description_msg['type_description']['type_name']
c_typename = typename.replace('/', '__')
}@
ROSIDL_GENERATOR_C_PUBLIC_@(package_name)
const rosidl_type_hash_t *
@(c_typename)__@(GET_HASH_FUNC)(
  const rosidl_@(interface_type)_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = @(type_hash_to_c_definition(hash_lookup[typename], indent=4));
  return &hash;
}

@[end for]@
@#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
@
@#<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
@# Descriptions and sources functions (optionally disabled)
@[if disable_description_codegen]@
@{
TEMPLATE(
  'empty__description.c.em',
  toplevel_type_description=toplevel_type_description,
  implicit_type_descriptions=implicit_type_descriptions)
}@
@[else]@
@{
TEMPLATE(
  'full__description.c.em',
  toplevel_type_description=toplevel_type_description,
  implicit_type_descriptions=implicit_type_descriptions,
  hash_lookup=hash_lookup,
  type_source_file=type_source_file)
}@
@[end if]@
@#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
