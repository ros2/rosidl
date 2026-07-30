// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from @(package_name):@(interface_path)
// generated code does not contain a copyright notice
@{from rosidl_pycommon import convert_camel_case_to_lower_case_underscore}
// IWYU pragma: private, include "@(package_name)/@(interface_path.parent)/@(convert_camel_case_to_lower_case_underscore(interface_path.stem)).hpp"

@
@#######################################################################
@# EmPy template for generating <idl>__struct.hpp files
@#
@# Context:
@#  - package_name (string)
@#  - interface_path (Path relative to the directory named after the package)
@#  - content (IdlContent, list of elements, e.g. Messages or Services)
@#######################################################################
@{
from rosidl_pycommon import convert_camel_case_to_lower_case_underscore
include_parts = [package_name] + list(interface_path.parents[0].parts) + [
    'detail', convert_camel_case_to_lower_case_underscore(interface_path.stem)]
header_guard_variable = '__'.join([x.upper() for x in include_parts]) + \
    '__STRUCT_HPP_'

include_directives = set()

# Scan all messages (including the ones implicitly defined by services and
# actions) to only emit the includes their members and constants actually
# need. This mirrors the TEMPLATE() expansion done below: services expand to
# request/response/event messages, actions to goal/result/feedback(+message)
# and two services.
from rosidl_generator_cpp import CPPLINT_ALGORITHM_NAMES
from rosidl_generator_cpp import get_all_messages
from rosidl_parser.definition import AbstractGenericString
from rosidl_parser.definition import AbstractNestedType
from rosidl_parser.definition import Array
from rosidl_parser.definition import BasicType
from rosidl_parser.definition import BoundedSequence
from rosidl_parser.definition import UnboundedSequence

all_messages = get_all_messages(content)

need_algorithm = False
need_array = False
need_string = False
need_vector = False
need_bounded_vector = False
need_buffer = False
for msg in all_messages:
    for constant in msg.constants:
        if isinstance(constant.type, AbstractGenericString):
            need_string = True
    for member in msg.structure.members:
        type_ = member.type
        if member.name in CPPLINT_ALGORITHM_NAMES:
            need_algorithm = True
        if member.name == 'string':
            # cpplint reads the bare word 'string' as std::string
            need_string = True
        if isinstance(type_, Array):
            need_array = True
            # std::fill is emitted for zero/default initialization of arrays
            # of primitive or string values
            if isinstance(
                type_.value_type, (BasicType, AbstractGenericString)
            ):
                need_algorithm = True
        elif isinstance(type_, UnboundedSequence):
            # unbounded uint8 sequences map to rosidl::Buffer,
            # everything else to std::vector (see msg_type_to_cpp)
            if (
                isinstance(type_.value_type, BasicType) and
                type_.value_type.typename == 'uint8'
            ):
                need_buffer = True
            else:
                need_vector = True
        elif isinstance(type_, BoundedSequence):
            need_bounded_vector = True
        value_type = type_.value_type \
            if isinstance(type_, AbstractNestedType) else type_
        if isinstance(value_type, AbstractGenericString):
            need_string = True
}@

#ifndef @(header_guard_variable)
#define @(header_guard_variable)

@[if need_algorithm]@
#include <algorithm>
@[end if]@
@[if need_array]@
#include <array>
@[end if]@
#include <cstdint>
#include <memory>
@[if need_string]@
#include <string>
@[end if]@
@[if need_vector]@
#include <vector>
@[end if]@

@[if need_bounded_vector]@
#include "rosidl_runtime_cpp/bounded_vector.hpp"
@[end if]@
@[if need_buffer]@
#include "rosidl_buffer/buffer.hpp"
@[end if]@
#include "rosidl_runtime_cpp/message_initialization.hpp"

@#######################################################################
@# Handle message
@#######################################################################
@{
from rosidl_parser.definition import Message
}@
@[for message in content.get_elements_of_type(Message)]@
@{
TEMPLATE(
    'msg__struct.hpp.em',
    package_name=package_name, interface_path=interface_path,
    message=message, include_directives=include_directives)
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
    'srv__struct.hpp.em',
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
    'action__struct.hpp.em',
    package_name=package_name, interface_path=interface_path, action=action,
    include_directives=include_directives)
}@

@[end for]@
#endif  // @(header_guard_variable)
