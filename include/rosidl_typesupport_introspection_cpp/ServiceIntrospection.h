
#ifndef __rosidl_typesupport_introspection_cpp__ServiceIntrospection__h__
#define __rosidl_typesupport_introspection_cpp__ServiceIntrospection__h__

#include <rosidl_generator_c/service_type_support.h>
#include <rosidl_generator_cpp/ServiceTypeSupport.h>

#include "visibility_control.h"

namespace rosidl_typesupport_introspection_cpp
{

template<typename T>
ROSIDL_TSI_CPP_PUBLIC
const rosidl_service_type_support_t * get_service_type_support_handle();

}  // namespace rosidl_typesupport_introspection_cpp

#endif  // __rosidl_typesupport_introspection_cpp__ServiceIntrospection__h__
