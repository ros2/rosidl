// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
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
include_directives = set()
}@
@#######################################################################
@# Handle message
@#######################################################################
@{
from rosidl_parser.definition import Message
}@
@[for message in content.get_elements_of_type(Message)]@

@{
TEMPLATE(
    'msg__type_support.c.em',
    package_name=package_name, interface_path=interface_path, message=message,
    include_directives=include_directives)
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
TEMPLATE(
    'srv__type_support.c.em',
    package_name=package_name, interface_path=interface_path, service=service,
    include_directives=include_directives)
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
    'msg__type_support.c.em',
    package_name=package_name, interface_path=interface_path, message=action.goal_request,
    include_directives=include_directives)
}@

@{
TEMPLATE(
    'msg__type_support.c.em',
    package_name=package_name, interface_path=interface_path, message=action.result_response,
    include_directives=include_directives)
}@

@{
TEMPLATE(
    'msg__type_support.c.em',
    package_name=package_name, interface_path=interface_path, message=action.feedback,
    include_directives=include_directives)
}@

@{
TEMPLATE(
    'srv__type_support.c.em',
    package_name=package_name, interface_path=interface_path, service=action.goal_service,
    include_directives=include_directives)
}@

@{
TEMPLATE(
    'srv__type_support.c.em',
    package_name=package_name, interface_path=interface_path, service=action.result_service,
    include_directives=include_directives)
}@

@{
TEMPLATE(
    'msg__type_support.c.em',
    package_name=package_name, interface_path=interface_path, message=action.feedback_message,
    include_directives=include_directives)
}@
@[end for]@
