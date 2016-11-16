// Copyright 2015 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// This header contains the type support introspection specific implementation
// of the <rosidl_generator_cpp/service_type_support_decl.hpp> header.

#ifndef ROSIDL_TYPESUPPORT_INTROSPECTION_CPP__IMPL__ROSIDL_GENERATOR_CPP__SERVICE_TYPE_SUPPORT_HPP_
#define ROSIDL_TYPESUPPORT_INTROSPECTION_CPP__IMPL__ROSIDL_GENERATOR_CPP__SERVICE_TYPE_SUPPORT_HPP_

// Provides the definition of the rosidl_service_type_support_t struct.
#include "rosidl_generator_c/service_type_support.h"
// Provides the declaration of the function
// rosidl_generator_cpp::get_service_type_support_handle.
#include "rosidl_generator_cpp/service_type_support_decl.hpp"
// Provides the declaration of the function template
#include "rosidl_typesupport_introspection_cpp/service_type_support_decl.hpp"
// Provides visibility macros like ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC.
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace rosidl_generator_cpp
{

template<typename T>
const rosidl_service_type_support_t * get_service_type_support_handle()
{
  // Hand off to implementation specific handle getter.
  // By using this implementation specific getter in this header, it makes
  // the user executable or library have a reference to an implementation
  // specific symbol, which is implemented in the implementation specific
  // service library. This is intentional to allow the linker to pick the
  // correct implementation specific service library when being over linked
  // with multiple implementation options.
  return ::rosidl_typesupport_introspection_cpp::get_service_type_support_handle<T>();
}

}  // namespace rosidl_generator_cpp

#endif  // ROSIDL_TYPESUPPORT_INTROSPECTION_CPP__IMPL__ROSIDL_GENERATOR_CPP__SERVICE_TYPE_SUPPORT_HPP_
