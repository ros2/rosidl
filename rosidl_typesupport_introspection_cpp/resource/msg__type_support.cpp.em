// generated from rosidl_typesupport_introspection_cpp/resource/msg__type_support.cpp.em
// generated code does not contain a copyright notice

@#######################################################################
@# EmPy template for generating <msg>__type_support.cpp files
@#
@# Context:
@#  - spec (rosidl_parser.MessageSpecification)
@#    Parsed specification of the .msg file
@#  - subfolder (string)
@#    The subfolder / subnamespace of the message
@#    Either 'msg', 'srv' or 'action'
@#  - get_header_filename_from_msg_name (function)
@#######################################################################
@
#include <array>
// providing offsetof()
#include <cstddef>
#include <string>
#include <vector>

#include "rosidl_generator_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"

#include "@(spec.base_type.pkg_name)/@(subfolder)/@(get_header_filename_from_msg_name(spec.base_type.type))__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace @(spec.base_type.pkg_name)
{

namespace @(subfolder)
{

namespace rosidl_typesupport_introspection_cpp
{

@[if spec.fields]@
@[  for field in spec.fields]@
@{
def is_vector_bool(field):
  return field.type.type == 'bool' and not (field.type.array_size and not field.type.is_upper_bound)
}@
@# exclude std::vector<bool> because of specialization in their API
@[    if field.type.is_array and not is_vector_bool(field)]@
@{
# from rosidl_generator_cpp import msg_type_only_to_cpp
# type = msg_type_only_to_cpp(field.type)
default_type = str(field.type.pkg_name) + '::msg::' + field.type.type
type_ = cpp_primitives.get(field.type.type, default_type)
if field.type.type == 'string':
    type_ = 'std::string'
}@
size_t size_function__@(spec.base_type.type)__@(field.name)(const void * untyped_member)
{
@[      if field.type.array_size and not field.type.is_upper_bound]@
  (void)untyped_member;
  return @(field.type.array_size);
@[      else]@
  const auto * member = reinterpret_cast<const std::vector<@(type_)> *>(untyped_member);
  return member->size();
@[      end if]@
}

const void * get_const_function__@(spec.base_type.type)__@(field.name)(const void * untyped_member, size_t index)
{
@[      if field.type.array_size and not field.type.is_upper_bound]@
  const auto & member =
    *reinterpret_cast<const std::array<@(type_), @(field.type.array_size)> *>(untyped_member);
@[      else]@
  const auto & member =
    *reinterpret_cast<const std::vector<@(type_)> *>(untyped_member);
@[      end if]@
  return &member[index];
}

void * get_function__@(spec.base_type.type)__@(field.name)(void * untyped_member, size_t index)
{
@[      if field.type.array_size and not field.type.is_upper_bound]@
  auto & member =
    *reinterpret_cast<std::array<@(type_), @(field.type.array_size)> *>(untyped_member);
@[      else]@
  auto & member =
    *reinterpret_cast<std::vector<@(type_)> *>(untyped_member);
@[      end if]@
  return &member[index];
}

@[      if not field.type.array_size or field.type.is_upper_bound]@
void resize_function__@(spec.base_type.type)__@(field.name)(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<@(type_)> *>(untyped_member);
  member->resize(size);
}

@[      end if]@
@[    end if]@
@[  end for]@
static const ::rosidl_typesupport_introspection_cpp::MessageMember @(spec.base_type.type)_message_member_array[@(len(spec.fields))] = {
@{
for index, field in enumerate(spec.fields):
    print('  {')

    # const char * name_
    print('    "%s",  // name' % field.name)
    if field.type.is_primitive_type():
        # uint8_t type_id_
        print('    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_%s,  // type' % field.type.type.upper())
        # size_t string_upper_bound
        print('    %u,  // upper bound of string' % (field.type.string_upper_bound if field.type.string_upper_bound is not None else 0))
        # const rosidl_generator_cpp::MessageTypeSupportHandle * members_
        print('    nullptr,  // members of sub message')
    else:
        # uint8_t type_id_
        print('    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type')
        # size_t string_upper_bound
        print('    0,  // upper bound of string')
        # const rosidl_message_type_support_t * members_
        print('    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<%s::msg::%s>(),  // members of sub message' % (field.type.pkg_name, field.type.type))
    # bool is_array_
    print('    %s,  // is array' % ('true' if field.type.is_array else 'false'))
    # size_t array_size_
    print('    %u,  // array size' % (field.type.array_size if field.type.array_size else 0))
    # bool is_upper_bound_
    print('    %s,  // is upper bound' % ('true' if field.type.is_upper_bound else 'false'))
    # unsigned long offset_
    print('    offsetof(%s::%s::%s, %s),  // bytes offset in struct' % (spec.base_type.pkg_name, subfolder, spec.base_type.type, field.name))
    # void * default_value_
    print('    nullptr,  // default value')  # TODO default value to be set

    function_suffix = '%s__%s' % (spec.base_type.type, field.name) if field.type.is_array and not is_vector_bool(field) else None

    # size_t(const void *) size_function
    print('    %s,  // size() function pointer' % ('size_function__%s' % function_suffix if function_suffix else 'nullptr'))
    # const void *(const void *, size_t) get_const_function
    print('    %s,  // get_const(index) function pointer' % ('get_const_function__%s' % function_suffix if function_suffix else 'nullptr'))
    # void *(void *, size_t) get_function
    print('    %s,  // get(index) function pointer' % ('get_function__%s' % function_suffix if function_suffix else 'nullptr'))
    # void(void *, size_t) resize_function
    print('    %s  // resize(index) function pointer' % ('resize_function__%s' % function_suffix if function_suffix and (not field.type.array_size or field.type.is_upper_bound) else 'nullptr'))

    if index < len(spec.fields) - 1:
        print('  },')
    else:
        print('  }')
}@
};

@[end if]@
static const ::rosidl_typesupport_introspection_cpp::MessageMembers @(spec.base_type.type)_message_members = {
  "@(spec.base_type.pkg_name)",  // package name
  "@(spec.base_type.type)",  // message name
  @(len(spec.fields)),  // number of fields
  sizeof(@(spec.base_type.pkg_name)::@(subfolder)::@(spec.base_type.type)),
@[if spec.fields]@
  @(spec.base_type.type)_message_member_array  // message members
@[else]@
  0  // message members
@[end if]@
};

static const rosidl_message_type_support_t @(spec.base_type.type)_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &@(spec.base_type.type)_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace @(subfolder)

}  // namespace @(spec.base_type.pkg_name)


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<@(spec.base_type.pkg_name)::@(subfolder)::@(spec.base_type.type)>()
{
  return &::@(spec.base_type.pkg_name)::@(subfolder)::rosidl_typesupport_introspection_cpp::@(spec.base_type.type)_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, @(spec.base_type.pkg_name), @(subfolder), @(spec.base_type.type))() {
  return &::@(spec.base_type.pkg_name)::@(subfolder)::rosidl_typesupport_introspection_cpp::@(spec.base_type.type)_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
