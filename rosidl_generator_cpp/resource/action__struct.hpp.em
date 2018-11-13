// generated from rosidl_generator_cpp/resource/action__struct.hpp.em
// generated code does not contain a copyright notice

@#######################################################################
@# EmPy template for generating <action>__struct.hpp files
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
    get_header_filename_from_msg_name(spec.action_name) + '__struct_hpp']
header_guard_variable = '__'.join([x.upper() for x in header_guard_parts]) + '_'
}@
#ifndef @(header_guard_variable)
#define @(header_guard_variable)

#include <action_msgs/srv/cancel_goal.hpp>
#include <action_msgs/msg/goal_info.hpp>
#include <action_msgs/msg/goal_status_array.hpp>
#include <@(spec.pkg_name)/@(subfolder)/@(get_header_filename_from_msg_name(spec.action_name))__feedback.hpp>
#include <@(spec.pkg_name)/@(subfolder)/@(get_header_filename_from_msg_name(spec.action_name))__goal.hpp>
#include <@(spec.pkg_name)/@(subfolder)/@(get_header_filename_from_msg_name(spec.action_name))__result.hpp>

namespace @(spec.pkg_name)
{

namespace @(subfolder)
{

struct @(spec.action_name)
{
  using CancelGoalService = action_msgs::srv::CancelGoal;
  using GoalStatusMessage = action_msgs::msg::GoalStatusArray;
  using GoalRequestService = @(spec.pkg_name)::@(subfolder)::@(spec.action_name)_Goal;
  using GoalResultService = @(spec.pkg_name)::@(subfolder)::@(spec.action_name)_Result;

  using Goal = GoalRequestService::Request;
  using Result = GoalResultService::Response;
  using Feedback = @(spec.pkg_name)::@(subfolder)::@(spec.action_name)_Feedback;
};

typedef struct @(spec.action_name) @(spec.action_name);

}  // namespace @(subfolder)

}  // namespace @(spec.pkg_name)

#endif  // @(header_guard_variable)
