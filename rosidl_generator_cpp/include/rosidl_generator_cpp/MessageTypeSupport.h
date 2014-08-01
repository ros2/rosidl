
#ifndef __rosidl_generator_cpp__MessageTypeSupport__h__
#define __rosidl_generator_cpp__MessageTypeSupport__h__

namespace rosidl_generator_cpp
{

typedef struct MessageTypeSupportMember {
  const char * _name;
  const char * _type;
  unsigned long _offset;
} MessageTypeSupportMember;

typedef struct MessageTypeSupportMembers {
  unsigned long _size;
  const MessageTypeSupportMember * _members;
} MessageTypeSupportMembers;

template<typename T>
struct MessageTypeSupport
{
  static const MessageTypeSupportMembers& get_members();
};

}  // namespace rosidl_generator_cpp

#endif  // __rosidl_generator_cpp__MessageTypeSupport__h__
