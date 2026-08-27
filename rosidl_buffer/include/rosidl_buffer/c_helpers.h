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

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "rosidl_buffer/visibility_control.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum rosidl_buffer_ret_e
{
  ROSIDL_BUFFER_RET_OK = 0,
  ROSIDL_BUFFER_RET_INVALID_ARGUMENT = 1,
  ROSIDL_BUFFER_RET_NOT_CPU = 2,
  ROSIDL_BUFFER_RET_BAD_ALLOC = 3,
  ROSIDL_BUFFER_RET_ERROR = 4
} rosidl_buffer_ret_t;

ROSIDL_BUFFER_PUBLIC
rosidl_buffer_ret_t rosidl_buffer_uint8_create_cpu(
  const uint8_t * data, size_t size, void ** buffer_ptr);

ROSIDL_BUFFER_PUBLIC
rosidl_buffer_ret_t rosidl_buffer_uint8_clone(
  const void * source_ptr, void ** buffer_ptr);

ROSIDL_BUFFER_PUBLIC
rosidl_buffer_ret_t rosidl_buffer_uint8_size(
  const void * buffer_ptr, size_t * size);

ROSIDL_BUFFER_PUBLIC
rosidl_buffer_ret_t rosidl_buffer_uint8_backend_name(
  const void * buffer_ptr, char * output, size_t output_capacity, size_t * required_size);

ROSIDL_BUFFER_PUBLIC
rosidl_buffer_ret_t rosidl_buffer_uint8_copy_to(
  const void * buffer_ptr, uint8_t * output, size_t output_size);

ROSIDL_BUFFER_PUBLIC
rosidl_buffer_ret_t rosidl_buffer_uint8_are_equal(
  const void * lhs_ptr, const void * rhs_ptr, bool * are_equal);

ROSIDL_BUFFER_PUBLIC
rosidl_buffer_ret_t rosidl_buffer_uint8_equals_data(
  const void * buffer_ptr, const uint8_t * data, size_t size, bool * are_equal);

/// Throw std::runtime_error if the buffer is not CPU-backed.
/**
 * \param[in] buffer_ptr Opaque pointer to an rosidl::Buffer<uint8_t>
 */
ROSIDL_BUFFER_PUBLIC
void rosidl_buffer_uint8_throw_if_not_cpu(const void * buffer_ptr);

/// Destroy a heap-allocated rosidl::Buffer<uint8_t>.
/**
 * This is the canonical destruction function for Buffer pointers that were
 * created with `new rosidl::Buffer<uint8_t>()` during deserialization.
 * The virtual destructor chain in BufferImplBase ensures backend-specific
 * cleanup (e.g. GPU memory) happens automatically.
 *
 * Safe to call with NULL (no-op).
 *
 * \param[in] buffer_ptr Opaque pointer to a heap-allocated rosidl::Buffer<uint8_t>,
 *   or NULL.
 */
ROSIDL_BUFFER_PUBLIC
void rosidl_buffer_uint8_destroy(void * buffer_ptr);

#ifdef __cplusplus
}
#endif

#endif  // ROSIDL_BUFFER__C_HELPERS_H_
