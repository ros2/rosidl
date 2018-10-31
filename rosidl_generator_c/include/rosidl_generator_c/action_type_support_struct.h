// Copyright 2018 Open Source Robotics Foundation, Inc.
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

#ifndef ROSIDL_GENERATOR_C__ACTION_TYPE_SUPPORT_STRUCT_H_
#define ROSIDL_GENERATOR_C__ACTION_TYPE_SUPPORT_STRUCT_H_

#include "rosidl_generator_c/message_type_support_struct.h"
#include "rosidl_generator_c/service_type_support_struct.h"
#include "rosidl_generator_c/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct rosidl_action_type_support_t rosidl_action_type_support_t;

struct rosidl_action_type_support_t
{
  const rosidl_service_type_support_t * goal_service_type_support;
  const rosidl_service_type_support_t * result_service_type_support;
  const rosidl_service_type_support_t * cancel_service_type_support;
  const rosidl_message_type_support_t * feedback_message_type_support;
  const rosidl_message_type_support_t * status_message_type_support;
};

#define ROSIDL_GET_ACTION_TYPE_SUPPORT(PkgName, Subfolder, Name) \
  ROSIDL_TYPESUPPORT_INTERFACE__ACTION_SYMBOL_NAME( \
    rosidl_typesupport_c, PkgName, Subfolder, Name)()

#ifdef __cplusplus
}
#endif

#endif  // ROSIDL_GENERATOR_C__ACTION_TYPE_SUPPORT_STRUCT_H_
