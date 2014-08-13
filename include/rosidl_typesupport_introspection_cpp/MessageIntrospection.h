
#ifndef __rosidl_typesupport_introspection_cpp__MessageTypeSupport__h__
#define __rosidl_typesupport_introspection_cpp__MessageTypeSupport__h__

namespace rosidl_typesupport_introspection_cpp
{

extern const char * typesupport_introspection_identifier;

typedef struct MessageMember {
  const char * name_;
  const char * type_;
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

#endif  // __rosidl_typesupport_introspection_cpp__MessageTypeSupport__h__
