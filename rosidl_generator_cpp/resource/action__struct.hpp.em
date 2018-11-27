@# Included from rosidl_generator_cpp/resource/idl__struct.hpp.em
@{
from rosidl_parser.definition import ACTION_FEEDBACK_MESSAGE_SUFFIX
from rosidl_parser.definition import ACTION_GOAL_SERVICE_SUFFIX
from rosidl_parser.definition import ACTION_RESULT_SERVICE_SUFFIX
from rosidl_parser.definition import ACTION_WRAPPER_TYPE_SUFFIX
from rosidl_parser.definition import SERVICE_REQUEST_MESSAGE_SUFFIX
from rosidl_parser.definition import SERVICE_RESPONSE_MESSAGE_SUFFIX
action_includes = (
    'action_msgs/srv/cancel_goal.hpp',
    'action_msgs/msg/goal_info.hpp',
    'action_msgs/msg/goal_status_array.hpp',
)
action_name = '::'.join(action.structure_type.namespaces + [action.structure_type.name])
}@
@[for header_file in action_includes]@
@[    if header_file in include_directives]@
// already included above
// @
@[    else]@
@{include_directives.add(header_file)}@
@[    end if]@
#include "@(header_file)"
@[end for]@

@[for ns in action.structure_type.namespaces]@
namespace @(ns)
{

@[end for]@
struct @(action.structure_type.name)
{
  using CancelGoalService = action_msgs::srv::CancelGoal;
  using GoalStatusMessage = action_msgs::msg::GoalStatusArray;
  using GoalRequestService = @(action_name)@(ACTION_WRAPPER_TYPE_SUFFIX)@(ACTION_GOAL_SERVICE_SUFFIX);
  using GoalResultService = @(action_name)@(ACTION_WRAPPER_TYPE_SUFFIX)@(ACTION_RESULT_SERVICE_SUFFIX);
  using FeedbackMessage = @(action_name)@(ACTION_WRAPPER_TYPE_SUFFIX)@(ACTION_FEEDBACK_MESSAGE_SUFFIX);

  using Goal = @(action_name)@(ACTION_GOAL_SERVICE_SUFFIX)@(SERVICE_REQUEST_MESSAGE_SUFFIX);
  using Result = @(action_name)@(ACTION_RESULT_SERVICE_SUFFIX)@(SERVICE_RESPONSE_MESSAGE_SUFFIX);
  using Feedback = @(action_name)@(ACTION_FEEDBACK_MESSAGE_SUFFIX);
};

typedef struct @(action.structure_type.name) @(action.structure_type.name);
@
@[for ns in reversed(action.structure_type.namespaces)]@

}  // namespace @(ns)
@[end for]@
