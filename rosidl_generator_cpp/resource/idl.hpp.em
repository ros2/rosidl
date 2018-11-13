// generated from rosidl_generator_cpp/resource/idl.hpp.em
// generated code does not contain a copyright notice

@#######################################################################
@# EmPy template for generating <idl>.hpp files
@#
@# Context:
@#  - package_name (string)
@#  - interface_path (Path relative to the directory named after the package)
@#  - interfaces (list of interfaces, either Messages or Services)
@#######################################################################
@
@{
from rosidl_cmake import convert_camel_case_to_lower_case_underscore
include_parts = [package_name] + list(interface_path.parents[0].parts) + \
    [convert_camel_case_to_lower_case_underscore(interface_path.stem)]
header_guard_variable = '__'.join([x.upper() for x in include_parts]) + '_HPP_'
include_base = '/'.join(include_parts)
}@
#ifndef @(header_guard_variable)
#define @(header_guard_variable)

#include "@(include_base)__struct.hpp"
#include "@(include_base)__traits.hpp"

#endif  // @(header_guard_variable)
