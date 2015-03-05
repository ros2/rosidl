
#ifndef __rosidl_typesupport_introspection_cpp__ServiceIntrospection__h__
#define __rosidl_typesupport_introspection_cpp__ServiceIntrospection__h__

#include <rosidl_generator_c/service_type_support.h>
#include <rosidl_generator_cpp/ServiceTypeSupport.h>

namespace rosidl_typesupport_introspection_cpp
{

extern const char * typesupport_introspection_identifier;

template<typename T>
const rosidl_service_type_support_t * get_service_type_support_handle();

}  // namespace rosidl_typesupport_introspection_cpp

#endif  // __rosidl_typesupport_introspection_cpp__ServiceIntrospection__h__
