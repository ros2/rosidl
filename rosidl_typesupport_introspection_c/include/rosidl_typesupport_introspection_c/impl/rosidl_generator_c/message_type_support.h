// Copyright 2016 Open Source Robotics Foundation, Inc.
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

// This header contains the opensplice specific macros used to generate the
// name of the function used to retrieve the type support for a message.

#ifndef ROSIDL_TYPESUPPORT_INTROSPECTION_C__IMPL__ROSIDL_GENERATOR_C__MESSAGE_TYPE_SUPPORT_H_
#define ROSIDL_TYPESUPPORT_INTROSPECTION_C__IMPL__ROSIDL_GENERATOR_C__MESSAGE_TYPE_SUPPORT_H_

// Provides the definition of the rosidl_message_type_support_t struct.
#include <rosidl_generator_c/message_type_support_struct.h>

/* These macros are used to create the symbols of the get_message_type_support
 * function for a specific message type. The library of the message package
 * which defines a given message will provide the symbol to which this macro
 * expands.
 *
 * These macros also generate introspection-specific symbols so they only match
 * the introspection C shared library symbols.
 */
#define ROSIDL_GET_MSG_TYPE_SUPPORT(PkgName, MsgName) \
  ROSIDL_GET_TYPE_SUPPORT(PkgName, msg, MsgName)

#define ROSIDL_GET_SRV_TYPE_SUPPORT(PkgName, MsgName) \
  ROSIDL_GET_TYPE_SUPPORT(PkgName, srv, MsgName)

#define ROSIDL_GET_TYPE_SUPPORT(PkgName, MsgSubfolder, MsgName) \
  ROSIDL_GET_TYPE_SUPPORT_FUNCTION(PkgName, MsgSubfolder, MsgName)()

#define ROSIDL_GET_TYPE_SUPPORT_FUNCTION(PkgName, MsgSubfolder, MsgName) \
  rosidl_typesupport_introspection_c_get_message__ ## PkgName ## __ ## MsgSubfolder ## __ ## MsgName

#endif  // ROSIDL_TYPESUPPORT_INTROSPECTION_C__IMPL__ROSIDL_GENERATOR_C__MESSAGE_TYPE_SUPPORT_H_
