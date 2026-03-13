// Copyright 2026 Open Source Robotics Foundation, Inc.
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

#ifndef ROSIDL_BUFFER__C_HELPERS_H_
#define ROSIDL_BUFFER__C_HELPERS_H_

#include "rosidl_buffer/visibility_control.h"

#ifdef __cplusplus
extern "C" {
#endif

/// Throw std::runtime_error if the buffer is not CPU-backed.
/// @param buffer_ptr Opaque pointer to an rosidl::Buffer<uint8_t>
ROSIDL_BUFFER_PUBLIC
void rosidl_buffer_uint8_throw_if_not_cpu(const void * buffer_ptr);

#ifdef __cplusplus
}
#endif

#endif  // ROSIDL_BUFFER__C_HELPERS_H_
