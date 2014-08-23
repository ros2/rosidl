
#ifndef __rosidl_typesupport_introspection_cpp__MessageIntrospection__h__
#define __rosidl_typesupport_introspection_cpp__MessageIntrospection__h__

#include <stdint.h>

namespace rosidl_typesupport_introspection_cpp
{

extern const char * typesupport_introspection_identifier;

struct MessageMembers;

typedef struct MessageMember
{
  const char * name_;
  uint8_t type_id_;
  size_t string_upper_bound_;
  const rosidl_generator_cpp::MessageTypeSupportHandle* members_;
  bool is_array_;
  size_t array_size_;
  bool is_upper_bound_;
  unsigned long offset_;
  const void * default_value_;
} MessageMember;

typedef struct MessageMembers
{
  const char * package_name_;
  const char * message_name_;
  unsigned long member_count_;
  const MessageMember * members_;
} MessageMembers;

template<typename T>
const rosidl_generator_cpp::MessageTypeSupportHandle& get_type_support_handle();

}  // namespace rosidl_typesupport_introspection_cpp

#endif  // __rosidl_typesupport_introspection_cpp__MessageIntrospection__h__
