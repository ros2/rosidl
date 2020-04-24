// Copyright 2015-2018 Open Source Robotics Foundation, Inc.
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

#ifndef ROSIDL_RUNTIME_C__SEQUENCE_BOUND_H_
#define ROSIDL_RUNTIME_C__SEQUENCE_BOUND_H_

#include "rosidl_runtime_c/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct rosidl_runtime_c__Sequence__bound rosidl_runtime_c__Sequence__bound;

typedef const rosidl_runtime_c__Sequence__bound * (* rosidl_runtime_c__bound_handle_function)(
  const rosidl_runtime_c__Sequence__bound *, const char *);

struct rosidl_runtime_c__Sequence__bound
{
  const char * typesupport_identifier;
  const void * data;
  rosidl_runtime_c__bound_handle_function func;
};

ROSIDL_GENERATOR_C_PUBLIC
const rosidl_runtime_c__Sequence__bound * get_sequence_bound_handle(
  const rosidl_runtime_c__Sequence__bound * handle, const char * identifier);

ROSIDL_GENERATOR_C_PUBLIC
const rosidl_runtime_c__Sequence__bound * get_sequence_bound_handle_function(
  const rosidl_runtime_c__Sequence__bound * handle, const char * identifier);

#define ROSIDL_GET_SEQUENCE_BOUNDS(PkgName, MsgSubfolder, MsgName) \
  ROSIDL_BOUNDS_INTERFACE__MESSAGE_SYMBOL_NAME( \
    rosidl_typesupport_c, PkgName, MsgSubfolder, MsgName)()

#ifdef __cplusplus
}
#endif

#endif  // ROSIDL_RUNTIME_C__SEQUENCE_BOUND_H_
