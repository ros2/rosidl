@# Included from rosidl_generator_cpp/resource/idl__description.hpp.em
@
@#######################################################################
@# EmPy template for generating get_type_description functions for actions
@#
@# Context:
@#  - action (Action)
@#  - type_description_msg (dict representation of TypeDescription.msg)
@#######################################################################
@{
from rosidl_generator_type_description import extract_subinterface
}@

@{
TEMPLATE(
    'any__description.hpp.em',
    namespaced_type=action.namespaced_type,
    type_description_msg=type_description_msg)
}@

@{
TEMPLATE(
    'any__description.hpp.em',
    namespaced_type=action.goal.structure.namespaced_type,
    type_description_msg=extract_subinterface(type_description_msg, 'goal'))
}@

@{
TEMPLATE(
    'any__description.hpp.em',
    namespaced_type=action.result.structure.namespaced_type,
    type_description_msg=extract_subinterface(type_description_msg, 'result'))
}@

@{
TEMPLATE(
    'any__description.hpp.em',
    namespaced_type=action.feedback.structure.namespaced_type,
    type_description_msg=extract_subinterface(type_description_msg, 'feedback'))
}@

@{
TEMPLATE(
    'srv__description.hpp.em',
    service=action.send_goal_service,
    type_description_msg=extract_subinterface(type_description_msg, 'send_goal_service'))
}@
@{
TEMPLATE(
    'srv__description.hpp.em',
    service=action.get_result_service,
    type_description_msg=extract_subinterface(type_description_msg, 'get_result_service'))
}@

@{
TEMPLATE(
    'any__description.hpp.em',
    namespaced_type=action.feedback_message.structure.namespaced_type,
    type_description_msg=extract_subinterface(type_description_msg, 'feedback_message'))
}@
