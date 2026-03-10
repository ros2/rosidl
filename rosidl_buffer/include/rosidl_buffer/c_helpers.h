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

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/// Get the size of a Buffer<uint8_t>. Works for all backends.
/// @param buffer_ptr Opaque pointer to an rosidl::Buffer<uint8_t>
size_t rosidl_buffer_uint8_size(const void * buffer_ptr);

/// Get a const pointer to the underlying CPU data. Throws for non-CPU backends.
/// @param buffer_ptr Opaque pointer to an rosidl::Buffer<uint8_t>
const uint8_t * rosidl_buffer_uint8_data(const void * buffer_ptr);

/// Get a mutable pointer to the underlying CPU data. Throws for non-CPU backends.
/// @param buffer_ptr Opaque pointer to an rosidl::Buffer<uint8_t>
uint8_t * rosidl_buffer_uint8_data_mut(void * buffer_ptr);

/// Resize a Buffer<uint8_t>. Throws for non-CPU backends.
/// @param buffer_ptr Opaque pointer to an rosidl::Buffer<uint8_t>
/// @param size New size
void rosidl_buffer_uint8_resize(void * buffer_ptr, size_t size);

/// Throw std::runtime_error if the buffer is not CPU-backed.
/// @param buffer_ptr Opaque pointer to an rosidl::Buffer<uint8_t>
void rosidl_buffer_uint8_throw_if_not_cpu(const void * buffer_ptr);

#ifdef __cplusplus
}
#endif

#endif  // ROSIDL_BUFFER__C_HELPERS_H_
