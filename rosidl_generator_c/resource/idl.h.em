// generated from rosidl_generator_c/resource/idl.h.em
// generated code does not contain a copyright notice

@#######################################################################
@# EmPy template for generating <idl>.h files
@#
@# Context:
@#  - package_name (string)
@#  - interface_path (Path relative to the directory named after the package)
@#  - interfaces (list of interfaces, either Messages or Services)
@#  - get_header_filename_from_msg_name (function)
@#######################################################################
@
@{
include_parts = [package_name] + list(interface_path.parents[0].parts) + \
    [get_header_filename_from_msg_name(interface_path.stem)]
header_guard_variable = '__'.join([x.upper() for x in include_parts]) + '_H_'
include_base = '/'.join(include_parts)
}@
#ifndef @(header_guard_variable)
#define @(header_guard_variable)

#include "@(include_base)__struct.h"
#include "@(include_base)__functions.h"
#include "@(include_base)__type_support.h"

#endif  // @(header_guard_variable)
