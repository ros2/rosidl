// This header is connext implementation specific and is included by
// <rosidl_generator_cpp/MessageTypeSupport.h>

#ifndef __rosidl_generator_cpp__get_type_support_handle_impl__h__
#define __rosidl_generator_cpp__get_type_support_handle_impl__h__

// If evaluating just this file we need to include this header in order
// to get the prototype of rosidl_generator_cpp::get_type_support_handle
// with the appropriate dll import and export attributes set.
// Normally this header is included by the header below, so this will be
// skipped.
#include <rosidl_generator_cpp/MessageTypeSupport.h>

namespace rosidl_typesupport_introspection_cpp
{

// This is implemented in the shared library which goes with this header.
template<typename T>
ROSIDL_PUBLIC
const rosidl_message_type_support_t * get_type_support_handle_impl();

}  // namespace rosidl_typesupport_introspection_cpp

namespace rosidl_generator_cpp
{

template<typename T>
const rosidl_message_type_support_t * get_type_support_handle()
{
  // Hand off to implementation specific handle getter.
  // By using this implementation specific getter in this header, it makes
  // the user executable or library have a reference to an implementation
  // specific symbol, which is implemented in the implementation specific
  // message library. This is intentional to allow the linker to pick the
  // correct implementation library when being over linked with multiple
  // implementation options.
  return rosidl_typesupport_introspection_cpp::get_type_support_handle_impl<T>();
}

}  // namespace rosidl_generator_cpp

#endif  // __rosidl_generator_cpp__get_type_support_handle_impl__h__
