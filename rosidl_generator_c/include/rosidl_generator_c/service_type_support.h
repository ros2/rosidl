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

#ifndef ROSIDL_GENERATOR_C__SERVICE_TYPE_SUPPORT_H_
#define ROSIDL_GENERATOR_C__SERVICE_TYPE_SUPPORT_H_

#include "rosidl_generator_c/visibility_control.h"

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct ROSIDL_PUBLIC rosidl_service_type_support_t
{
  const char * typesupport_identifier;
  const void * data;
} rosidl_service_type_support_t;

/* This macro is used to create the symbol of the get_service_type_support
 * function for a specific service type. The library of the message package
 * which defines a given service will provide the symbol to which this macro
 * expands.
 */
#define ROSIDL_GET_SERVICE_TYPE_SUPPORT(SrvPkgName, SrvName) \
  rosidl_get_service_type_support__ ## SrvPkgName ## __ ## SrvName()

#ifdef __cplusplus
}
#endif

#endif  // ROSIDL_GENERATOR_C__SERVICE_TYPE_SUPPORT_H_
