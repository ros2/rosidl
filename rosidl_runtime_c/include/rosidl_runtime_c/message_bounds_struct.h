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

#ifndef ROSIDL_RUNTIME_C__MESSAGE_BOUNDS_STRUCT_H_
#define ROSIDL_RUNTIME_C__MESSAGE_BOUNDS_STRUCT_H_

#include "rosidl_runtime_c/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct rosidl_message_bounds_t rosidl_message_bounds_t;

typedef const rosidl_message_bounds_t * (* rosidl_message_bounds_handle_function)(
  const rosidl_message_bounds_t *, const char *);

/// Contains rosidl message bounds data
struct rosidl_message_bounds_t
{
  /// String identifier for the type_support.
  const char * typesupport_identifier;
  /// pointer to type support handle function
  const void * data;
  /// pointer to type support handle function.
  rosidl_message_bounds_handle_function func;
};

/// Get the message bounds handle specific to this identifier.
/**
 * If the identifier is the same as this handle's identifier, then the handle is
 * simply returned, otherwise it returns zero.
 *
 * \param handle Handle to service type support
 * \param identifier The identifier to get the handle for
 * \return The associated message bounds handle if found, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC
const rosidl_message_bounds_t * get_message_bounds_handle(
  const rosidl_message_bounds_t * handle, const char * identifier);

/// Get the message bounds handle function specific to this identifier.
/**
 * If the identifier is the same as this handle funtion's identifier, then the handle is
 * simply returned, otherwise it returns zero.
 *
 * \param handle Handle to service type support
 * \param identifier The identifier to get the handle function for
 * \return The associated message bounds handle function if found, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC
const rosidl_message_bounds_t * get_message_bounds_handle_function(
  const rosidl_message_bounds_t * handle, const char * identifier);

/// Macro to get the message bounds.
/*
 * \param PkgName Name of the package that contains the message
 * \param MsgSubfolder name of the subfolder (foe example: msg)
 * \param MsgName message name
 * \return a rosidl_message_bounds_t struct if founded, otherwise NULL.
 */
#define ROSIDL_GET_MSG_BOUNDS(PkgName, MsgSubfolder, MsgName) \
  ROSIDL_BOUNDS_INTERFACE__MESSAGE_SYMBOL_NAME( \
    rosidl_typesupport_c, PkgName, MsgSubfolder, MsgName)()

#ifdef __cplusplus
}
#endif

#endif  // ROSIDL_RUNTIME_C__MESSAGE_BOUNDS_STRUCT_H_
