@# Included from rosidl_generator_cpp/resource/idl__description.hpp.em
@
@#######################################################################
@# EmPy template for generating get_type_description functions for services
@#
@# Context:
@#  - service (Service)
@#  - type_description_msg (dict representation of TypeDescription.msg)
@#######################################################################
@{
from rosidl_generator_type_description import extract_subinterface
}@

@{
TEMPLATE(
    'any__description.hpp.em',
    namespaced_type=service.namespaced_type,
    type_description_msg=type_description_msg)
}@

@{
TEMPLATE(
    'any__description.hpp.em',
    namespaced_type=service.request_message.structure.namespaced_type,
    type_description_msg=extract_subinterface(type_description_msg, 'request_message'))
}@

@{
TEMPLATE(
    'any__description.hpp.em',
    namespaced_type=service.response_message.structure.namespaced_type,
    type_description_msg=extract_subinterface(type_description_msg, 'response_message'))
}@

@{
TEMPLATE(
    'any__description.hpp.em',
    namespaced_type=service.event_message.structure.namespaced_type,
    type_description_msg=extract_subinterface(type_description_msg, 'event_message'))
}@
