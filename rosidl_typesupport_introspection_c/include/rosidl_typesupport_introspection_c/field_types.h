// Copyright 2014-2015 Open Source Robotics Foundation, Inc.
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

#ifndef ROSIDL_TYPESUPPORT_INTROSPECTION_C__FIELD_TYPES_H_
#define ROSIDL_TYPESUPPORT_INTROSPECTION_C__FIELD_TYPES_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/// Possible types for message fields on a ROS message
enum rosidl_typesupport_introspection_c_field_types
{
  /// IEEE-754 binary32 format floating point number.
  rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT = 1,
  /// IEEE-754 binary64 format floating point number.
  rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE = 2,
  /// IEEE-754 binary128 format floating point number.
  rosidl_typesupport_introspection_c__ROS_TYPE_LONG_DOUBLE = 3,
  /// Unsigned char, 8 bits wide.
  rosidl_typesupport_introspection_c__ROS_TYPE_CHAR = 4,
  /// Wide character, 16 bits wide.
  rosidl_typesupport_introspection_c__ROS_TYPE_WCHAR = 5,
  /// Boolean value. The size is implementation defined.
  rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN = 6,
  /// A single unsigned byte in raw form.
  rosidl_typesupport_introspection_c__ROS_TYPE_OCTET = 7,
  /// Unsigned 8-bit integer.
  rosidl_typesupport_introspection_c__ROS_TYPE_UINT8 = 8,
  /// Signed 8-bit integer.
  rosidl_typesupport_introspection_c__ROS_TYPE_INT8 = 9,
  /// Unsigned 16-bit integer.
  rosidl_typesupport_introspection_c__ROS_TYPE_UINT16 = 10,
  /// Signed 16-bit integer.
  rosidl_typesupport_introspection_c__ROS_TYPE_INT16 = 11,
  /// Unsigned 32-bit integer.
  rosidl_typesupport_introspection_c__ROS_TYPE_UINT32 = 12,
  /// Signed 32-bit integer.
  rosidl_typesupport_introspection_c__ROS_TYPE_INT32 = 13,
  /// Unsigned 64-bit integer.
  rosidl_typesupport_introspection_c__ROS_TYPE_UINT64 = 14,
  /// Signed 64-bit integer.
  rosidl_typesupport_introspection_c__ROS_TYPE_INT64 = 15,
  /// String, represented as an array of char's.
  rosidl_typesupport_introspection_c__ROS_TYPE_STRING = 16,
  /// Wide string, represented as an array of elements at least 16-bits wide each.
  rosidl_typesupport_introspection_c__ROS_TYPE_WSTRING = 17,

  /// An embedded message type.
  rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE = 18,

  /// For backward compatibility only.
  rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT32 = 1,
  /// For backward compatibility only.
  rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT64 = 2,
  /// For backward compatibility only.
  rosidl_typesupport_introspection_c__ROS_TYPE_BOOL = 6,
  /// For backward compatibility only.
  rosidl_typesupport_introspection_c__ROS_TYPE_BYTE = 7
};

#ifdef __cplusplus
}
#endif

#endif  // ROSIDL_TYPESUPPORT_INTROSPECTION_C__FIELD_TYPES_H_
