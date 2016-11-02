# generated from rosidl_generator_py/resource/_srv.py.em
# generated code does not contain a copyright notice


class @(spec.srv_name):
    from @(package_name).srv._@convert_camel_case_to_lower_case_underscore(spec.srv_name)__request import @(spec.srv_name)_Request as Request
    from @(package_name).srv._@convert_camel_case_to_lower_case_underscore(spec.srv_name)__response import @(spec.srv_name)_Response as Response
