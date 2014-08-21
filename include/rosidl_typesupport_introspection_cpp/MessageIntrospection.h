
#ifndef __rosidl_typesupport_introspection_cpp__MessageIntrospection__h__
#define __rosidl_typesupport_introspection_cpp__MessageIntrospection__h__

#include <stdint.h>

namespace rosidl_typesupport_introspection_cpp
{

extern const char * typesupport_introspection_identifier;

struct MessageMembers;
typedef struct MessageMember {
  const char * name_;
  uint8_t type_id_;
  const rosidl_generator_cpp::MessageTypeSupportHandle* members_;
  bool is_array;
  size_t array_size;
  size_t upper_bound;
  unsigned long offset_;
} MessageMember;

typedef struct MessageMembers {
  const char * package_name_;
  const char * message_name_;
  unsigned long member_count_;
  const MessageMember * members_;
} MessageMembers;

template<typename T>
const rosidl_generator_cpp::MessageTypeSupportHandle& get_type_support_handle();

}  // namespace rosidl_typesupport_introspection_cpp

#endif  // __rosidl_typesupport_introspection_cpp__MessageIntrospection__h__
