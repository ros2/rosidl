@# Included from rosidl_generator_c/resource/idl__type_support.c.em
@{
TEMPLATE(
    'srv__type_support.c.em',
    package_name=package_name, service=action.send_goal_service,
    interface_path=interface_path, include_directives=include_directives)
}@

@{
TEMPLATE(
    'srv__type_support.c.em',
    package_name=package_name, service=action.get_result_service,
    interface_path=interface_path, include_directives=include_directives)
}@
