// Copyright 2014 Open Source Robotics Foundation, Inc.
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

#ifndef ROSIDL_TYPESUPPORT_INTROSPECTION_CPP__FIELD_TYPES_HPP_
#define ROSIDL_TYPESUPPORT_INTROSPECTION_CPP__FIELD_TYPES_HPP_

#include <rosidl_typesupport_introspection_c/field_types.h>
#include <cstdint>

namespace rosidl_typesupport_introspection_cpp
{

/// Primitive (plain-old data) types supported by the ROS type system.

/// IEEE-754 binary32 format floating point number.
const uint8_t ROS_TYPE_FLOAT = rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT;
/// IEEE-754 binary64 format floating point number.
const uint8_t ROS_TYPE_DOUBLE = rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE;
/// IEEE-754 binary128 format floating point number.
const uint8_t ROS_TYPE_LONG_DOUBLE = rosidl_typesupport_introspection_c__ROS_TYPE_LONG_DOUBLE;
/// Unsigned char, 8 bits wide.
const uint8_t ROS_TYPE_CHAR = rosidl_typesupport_introspection_c__ROS_TYPE_CHAR;
/// Wide character, large enough to support Unicode code points.
const uint8_t ROS_TYPE_WCHAR = rosidl_typesupport_introspection_c__ROS_TYPE_WCHAR;
/// Boolean value. The size is implementation defined.
const uint8_t ROS_TYPE_BOOLEAN = rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN;
/// A single unsigned byte in raw form.
const uint8_t ROS_TYPE_OCTET = rosidl_typesupport_introspection_c__ROS_TYPE_OCTET;
/// Unsigned 8-bit integer.
const uint8_t ROS_TYPE_UINT8 = rosidl_typesupport_introspection_c__ROS_TYPE_UINT8;
/// Signed 8-bit integer.
const uint8_t ROS_TYPE_INT8 = rosidl_typesupport_introspection_c__ROS_TYPE_INT8;
/// Unsigned 16-bit integer.
const uint8_t ROS_TYPE_UINT16 = rosidl_typesupport_introspection_c__ROS_TYPE_UINT16;
/// Signed 16-bit integer.
const uint8_t ROS_TYPE_INT16 = rosidl_typesupport_introspection_c__ROS_TYPE_INT16;
/// Unsigned 32-bit integer.
const uint8_t ROS_TYPE_UINT32 = rosidl_typesupport_introspection_c__ROS_TYPE_UINT32;
/// Signed 32-bit integer.
const uint8_t ROS_TYPE_INT32 = rosidl_typesupport_introspection_c__ROS_TYPE_INT32;
/// Unsigned 64-bit integer.
const uint8_t ROS_TYPE_UINT64 = rosidl_typesupport_introspection_c__ROS_TYPE_UINT64;
/// Signed 64-bit integer.
const uint8_t ROS_TYPE_INT64 = rosidl_typesupport_introspection_c__ROS_TYPE_INT64;
/// String, represented as an array of char's.
const uint8_t ROS_TYPE_STRING = rosidl_typesupport_introspection_c__ROS_TYPE_STRING;
/// Wide string, represented as an array of elements at least 16-bits wide each.
const uint8_t ROS_TYPE_WSTRING = rosidl_typesupport_introspection_c__ROS_TYPE_WSTRING;

/// An embedded message type.
const uint8_t ROS_TYPE_MESSAGE = rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE;

/// For backward compatibility only.
const uint8_t ROS_TYPE_BOOL = rosidl_typesupport_introspection_c__ROS_TYPE_BOOL;
/// For backward compatibility only.
const uint8_t ROS_TYPE_BYTE = rosidl_typesupport_introspection_c__ROS_TYPE_BYTE;
/// For backward compatibility only.
const uint8_t ROS_TYPE_FLOAT32 = rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT32;
/// For backward compatibility only.
const uint8_t ROS_TYPE_FLOAT64 = rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT64;

}  // namespace rosidl_typesupport_introspection_cpp

#endif  // ROSIDL_TYPESUPPORT_INTROSPECTION_CPP__FIELD_TYPES_HPP_
