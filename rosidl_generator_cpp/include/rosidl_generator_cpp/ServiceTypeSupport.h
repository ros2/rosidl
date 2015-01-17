
#ifndef __rosidl_generator_cpp__ServiceTypeSupport__h__
#define __rosidl_generator_cpp__ServiceTypeSupport__h__

namespace rosidl_generator_cpp
{

typedef struct ServiceTypeSupportHandle {
  const char * _typesupport_identifier;
  const void * _data;
} ServiceTypeSupportHandle;

template<typename T>
const ServiceTypeSupportHandle& get_service_type_support_handle();

}  // namespace rosidl_generator_cpp

#endif  // __rosidl_generator_cpp__ServiceTypeSupport__h__
