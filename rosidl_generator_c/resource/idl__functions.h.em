// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from @(package_name):@(interface_path)
// generated code does not contain a copyright notice
@{from rosidl_pycommon import convert_camel_case_to_lower_case_underscore}
// IWYU pragma: private, include "@(package_name)/@(interface_path.parent)/@(convert_camel_case_to_lower_case_underscore(interface_path.stem)).h"

@
@#######################################################################
@# EmPy template for generating <idl>__struct.h files
@#
@# Context:
@#  - package_name (string)
@#  - interface_path (Path relative to the directory named after the package)
@#  - content (IdlContent, list of elements, e.g. Messages or Services)
@#######################################################################
@{
from rosidl_generator_c import idl_structure_type_to_c_typename
from rosidl_generator_type_description import GET_DESCRIPTION_FUNC
from rosidl_generator_type_description import GET_HASH_FUNC
from rosidl_generator_type_description import GET_INDIVIDUAL_SOURCE_FUNC
from rosidl_generator_type_description import GET_SOURCES_FUNC
from rosidl_pycommon import convert_camel_case_to_lower_case_underscore
include_parts = [package_name] + list(interface_path.parents[0].parts) + [
    'detail', convert_camel_case_to_lower_case_underscore(interface_path.stem)]
header_guard_variable = '__'.join([x.upper() for x in include_parts]) + \
    '__FUNCTIONS_H_'
include_base = '/'.join(include_parts)
}@

#ifndef @(header_guard_variable)
#define @(header_guard_variable)

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/action_type_support_struct.h"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_runtime_c/type_description/type_description__struct.h"
#include "rosidl_runtime_c/type_description/type_source__struct.h"
#include "rosidl_runtime_c/type_hash.h"
#include "rosidl_runtime_c/visibility_control.h"
#include "@(package_name)/msg/rosidl_generator_c__visibility_control.h"

#include "@(include_base)__struct.h"

@#######################################################################
@# Handle message
@#######################################################################
@{
from rosidl_parser.definition import Message
}@
@[for message in content.get_elements_of_type(Message)]@
@{
TEMPLATE(
    'msg__functions.h.em',
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
service_typename = idl_structure_type_to_c_typename(service.namespaced_type)
}@
/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_@(package_name)
const rosidl_type_hash_t *
@(service_typename)__@(GET_HASH_FUNC)(
  const rosidl_service_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_@(package_name)
const rosidl_runtime_c__type_description__TypeDescription *
@(service_typename)__@(GET_DESCRIPTION_FUNC)(
  const rosidl_service_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_@(package_name)
const rosidl_runtime_c__type_description__TypeSource *
@(service_typename)__@(GET_INDIVIDUAL_SOURCE_FUNC)(
  const rosidl_service_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_@(package_name)
const rosidl_runtime_c__type_description__TypeSource__Sequence *
@(service_typename)__@(GET_SOURCES_FUNC)(
  const rosidl_service_type_support_t * type_support);

@{
TEMPLATE(
    'msg__functions.h.em',
    package_name=package_name, interface_path=interface_path,
    message=service.request_message)
}@

@{
TEMPLATE(
    'msg__functions.h.em',
    package_name=package_name, interface_path=interface_path,
    message=service.response_message)
}@

@{
TEMPLATE(
    'msg__functions.h.em',
    package_name=package_name, interface_path=interface_path,
    message=service.event_message)
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
action_typename = idl_structure_type_to_c_typename(action.namespaced_type)
send_goal_srv_typename = idl_structure_type_to_c_typename(action.send_goal_service.namespaced_type)
get_result_srv_typename = idl_structure_type_to_c_typename(action.get_result_service.namespaced_type)
}@
/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_@(package_name)
const rosidl_type_hash_t *
@(action_typename)__@(GET_HASH_FUNC)(
  const rosidl_action_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_@(package_name)
const rosidl_runtime_c__type_description__TypeDescription *
@(action_typename)__@(GET_DESCRIPTION_FUNC)(
  const rosidl_action_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_@(package_name)
const rosidl_runtime_c__type_description__TypeSource *
@(action_typename)__@(GET_INDIVIDUAL_SOURCE_FUNC)(
  const rosidl_action_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_@(package_name)
const rosidl_runtime_c__type_description__TypeSource__Sequence *
@(action_typename)__@(GET_SOURCES_FUNC)(
  const rosidl_action_type_support_t * type_support);

@{
TEMPLATE(
    'msg__functions.h.em',
    package_name=package_name, interface_path=interface_path,
    message=action.goal)
}@

@{
TEMPLATE(
    'msg__functions.h.em',
    package_name=package_name, interface_path=interface_path,
    message=action.result)
}@

@{
TEMPLATE(
    'msg__functions.h.em',
    package_name=package_name, interface_path=interface_path,
    message=action.feedback)
}@

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_@(package_name)
const rosidl_type_hash_t *
@(send_goal_srv_typename)__@(GET_HASH_FUNC)(
  const rosidl_service_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_@(package_name)
const rosidl_runtime_c__type_description__TypeDescription *
@(send_goal_srv_typename)__@(GET_DESCRIPTION_FUNC)(
  const rosidl_service_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_@(package_name)
const rosidl_runtime_c__type_description__TypeSource *
@(send_goal_srv_typename)__@(GET_INDIVIDUAL_SOURCE_FUNC)(
  const rosidl_service_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_@(package_name)
const rosidl_runtime_c__type_description__TypeSource__Sequence *
@(send_goal_srv_typename)__@(GET_SOURCES_FUNC)(
  const rosidl_service_type_support_t * type_support);

@{
TEMPLATE(
    'msg__functions.h.em',
    package_name=package_name, interface_path=interface_path,
    message=action.send_goal_service.request_message)
}@

@{
TEMPLATE(
    'msg__functions.h.em',
    package_name=package_name, interface_path=interface_path,
    message=action.send_goal_service.response_message)
}@

@{
TEMPLATE(
    'msg__functions.h.em',
    package_name=package_name, interface_path=interface_path,
    message=action.send_goal_service.event_message)
}@

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_@(package_name)
const rosidl_type_hash_t *
@(get_result_srv_typename)__@(GET_HASH_FUNC)(
  const rosidl_service_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_@(package_name)
const rosidl_runtime_c__type_description__TypeDescription *
@(get_result_srv_typename)__@(GET_DESCRIPTION_FUNC)(
  const rosidl_service_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_@(package_name)
const rosidl_runtime_c__type_description__TypeSource *
@(get_result_srv_typename)__@(GET_INDIVIDUAL_SOURCE_FUNC)(
  const rosidl_service_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_@(package_name)
const rosidl_runtime_c__type_description__TypeSource__Sequence *
@(get_result_srv_typename)__@(GET_SOURCES_FUNC)(
  const rosidl_service_type_support_t * type_support);

@{
TEMPLATE(
    'msg__functions.h.em',
    package_name=package_name, interface_path=interface_path,
    message=action.get_result_service.request_message)
}@

@{
TEMPLATE(
    'msg__functions.h.em',
    package_name=package_name, interface_path=interface_path,
    message=action.get_result_service.response_message)
}@

@{
TEMPLATE(
    'msg__functions.h.em',
    package_name=package_name, interface_path=interface_path,
    message=action.get_result_service.event_message)
}@

@{
TEMPLATE(
    'msg__functions.h.em',
    package_name=package_name, interface_path=interface_path,
    message=action.feedback_message)
}@

@[end for]@
#ifdef __cplusplus
}
#endif

#endif  // @(header_guard_variable)
