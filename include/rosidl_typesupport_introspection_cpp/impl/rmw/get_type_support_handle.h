
#ifndef __rmw__get_type_support_handle__h__
#define __rmw__get_type_support_handle__h__

#include "rosidl_typesupport_introspection_cpp/MessageIntrospection.h"

namespace rmw
{
template<typename T>
const rosidl_message_type_support_t * get_type_support_handle()
{
  return rosidl_typesupport_introspection_cpp::get_type_support_handle<T>();
}

}  // namespace rmw

#endif  // __rmw__get_type_support_handle__h__
