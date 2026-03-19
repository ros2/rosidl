// Copyright 2024 Open Source Robotics Foundation, Inc.
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

/// @file deprecated.h
/// Cross-compiler macro for marking message fields as deprecated.

#ifndef ROSIDL_RUNTIME_C__DEPRECATED_H_
#define ROSIDL_RUNTIME_C__DEPRECATED_H_

/// Emit a compiler deprecation warning with the given message string.
/// Usage:
///   ROSIDL_DEPRECATED_MSG("field 'foo' is deprecated")
///   int32_t foo;
#ifndef ROSIDL_DEPRECATED_MSG
#  if defined(__GNUC__) || defined(__clang__)
#    define ROSIDL_DEPRECATED_MSG(msg) __attribute__((deprecated(msg)))
#  elif defined(_MSC_VER)
#    define ROSIDL_DEPRECATED_MSG(msg) __declspec(deprecated(msg))
#  else
#    define ROSIDL_DEPRECATED_MSG(msg)
#  endif
#endif

#endif  // ROSIDL_RUNTIME_C__DEPRECATED_H_
