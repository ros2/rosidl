// Copyright 2015-2016 Open Source Robotics Foundation, Inc.
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

#ifndef ROSIDL_RUNTIME_C__SERVICE_TYPE_SUPPORT_STRUCT_H_
#define ROSIDL_RUNTIME_C__SERVICE_TYPE_SUPPORT_STRUCT_H_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_runtime_c/visibility_control.h"

#include "rosidl_typesupport_interface/macros.h"
#include "stdint.h"

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct rosidl_service_type_support_t rosidl_service_type_support_t;

typedef const rosidl_service_type_support_t * (* rosidl_service_typesupport_handle_function)(
  const rosidl_service_type_support_t *, const char *);

typedef struct rosidl_service_introspection_info_s {
  uint8_t event_type;
  int32_t stamp_sec;
  uint32_t stamp_nanosec;
  uint32_t client_id[16];
  int64_t sequence_number;
} rosidl_service_introspection_info_t;

/// Fills in a service introspection request or response message
/**
 *
 *
 */
typedef void * (* rosidl_service_introspection_message_create_handle)(
    const rosidl_service_introspection_info_t * info,
    void * request_message,
    void * response_message);

typedef void (* rosidl_service_introspection_message_fini_handle)(
    void * event_message);

/// Contains rosidl service type support data
struct rosidl_service_type_support_t
{
  /// String identifier for the type_support.
  const char * typesupport_identifier;
  /// Pointer to the service type support library
  const void * data;
  /// Pointer to the service type support handler function
  rosidl_service_typesupport_handle_function func;
  /// Pointer to function to create the introspection message
  rosidl_service_introspection_message_create_handle introspection_message_create_handle;
  /// Pointer to function to finalize the introspection message
  rosidl_service_introspection_message_fini_handle introspection_message_fini_handle;
  /// Service event message typesupport
  const rosidl_message_type_support_t * event_typesupport;
};

/// Get the service type support handle specific to this identifier.
/**
 * The handle's message typesupport identifier function is returned or if the parameters are NULL
 * then an assert will happen.
 *
 * \param handle Handle to service type support
 * \param identifier The typesupport identifier to get the handle function for
 * \return The associated service typesupport handle function.
 */
ROSIDL_GENERATOR_C_PUBLIC
const rosidl_service_type_support_t * get_service_typesupport_handle(
  const rosidl_service_type_support_t * handle, const char * identifier);

/// Get the service type support handle function specific to this identifier.
/**
* If the identifier is the same as this handle's typesupport_identifier the handle is simply
* returned or if the parameters are NULL then an assert will happen.
 *
 * \param handle Handle to service type support
 * \param identifier The typesupport identifier to get the handle function for
 * \return if the identifier match's the handle's identifier then the handle's function
 *   is returned.
 */
ROSIDL_GENERATOR_C_PUBLIC
const rosidl_service_type_support_t * get_service_typesupport_handle_function(
  const rosidl_service_type_support_t * handle, const char * identifier);

/// Get the service type support given a provided action and package.
/*
 * \param PkgName Name of the package that contains the service
 * \param SrvSubfolder name of the subfolder (for example: srv)
 * \param SrvName service name
 * \return a rosidl_service_type_support_t struct if founded, otherwise NULL.
 */
#define ROSIDL_GET_SRV_TYPE_SUPPORT(PkgName, SrvSubfolder, SrvName) \
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME( \
    rosidl_typesupport_c, PkgName, SrvSubfolder, SrvName)()

#ifdef __cplusplus
}
#endif

#endif  // ROSIDL_RUNTIME_C__SERVICE_TYPE_SUPPORT_STRUCT_H_
