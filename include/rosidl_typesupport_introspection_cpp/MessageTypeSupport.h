
#ifndef __rosidl_typesupport_introspection_cpp__MessageTypeSupport__h__
#define __rosidl_typesupport_introspection_cpp__MessageTypeSupport__h__

namespace rosidl_typesupport_introspection_cpp
{

const char * _dynamic_identifier = "dynamic";

typedef struct MessageTypeSupportMember {
  const char * _name;
  const char * _type;
  unsigned long _offset;
} MessageTypeSupportMember;

typedef struct MessageTypeSupportMembers {
  const char * _package_name;
  const char * _message_name;
  unsigned long _size;
  const MessageTypeSupportMember * _members;
} MessageTypeSupportMembers;

}  // namespace rosidl_typesupport_introspection_cpp

#endif  // __rosidl_typesupport_introspection_cpp__MessageTypeSupport__h__
