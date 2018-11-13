// generated from rosidl_generator_c/resource/action.h.em
// generated code does not contain a copyright notice

@#######################################################################
@# EmPy template for generating <action>.hp files
@#
@# Context:
@#  - spec (rosidl_parser.ActionSpecification)
@#    Parsed specification of the .action file
@#  - subfolder (string)
@#    The subfolder / subnamespace of the message, usually 'action'
@#  - get_header_filename_from_msg_name (function)
@#######################################################################
@
@{
header_guard_parts = [
    spec.pkg_name, subfolder,
    get_header_filename_from_msg_name(spec.action_name) + '_h']
header_guard_variable = '__'.join([x.upper() for x in header_guard_parts]) + '_'
}@

#ifndef @(header_guard_variable)
#define @(header_guard_variable)

#ifdef __cplusplus
extern "C"
{
#endif

#include <action_msgs/msg/goal_info.h>
#include <action_msgs/msg/goal_status_array.h>
#include <action_msgs/srv/cancel_goal.h>

#include <@(spec.pkg_name)/@(subfolder)/@(get_header_filename_from_msg_name(spec.action_name))__feedback.h>
#include <@(spec.pkg_name)/@(subfolder)/@(get_header_filename_from_msg_name(spec.action_name))__goal.h>
#include <@(spec.pkg_name)/@(subfolder)/@(get_header_filename_from_msg_name(spec.action_name))__result.h>
#include "@(spec.pkg_name)/@(subfolder)/@(get_header_filename_from_msg_name(spec.action_name))__type_support.h"

#ifdef __cplusplus
}
#endif

#endif  // @(header_guard_variable)
