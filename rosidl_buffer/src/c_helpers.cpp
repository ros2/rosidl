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

#include "rosidl_buffer/c_helpers.h"

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <new>
#include <string>
#include <vector>

#include "rosidl_buffer/buffer.hpp"

extern "C" {

rosidl_buffer_ret_t rosidl_buffer_uint8_create_cpu(
  const uint8_t * data, size_t size, void ** buffer_ptr)
{
  if (!buffer_ptr || (size != 0 && !data)) {
    return ROSIDL_BUFFER_RET_INVALID_ARGUMENT;
  }
  *buffer_ptr = nullptr;
  try {
    auto * buffer = new rosidl::Buffer<uint8_t>(size);
    if (size != 0) {
      std::copy_n(data, size, buffer->data());
    }
    *buffer_ptr = buffer;
    return ROSIDL_BUFFER_RET_OK;
  } catch (const std::bad_alloc &) {
    return ROSIDL_BUFFER_RET_BAD_ALLOC;
  } catch (...) {
    return ROSIDL_BUFFER_RET_ERROR;
  }
}

rosidl_buffer_ret_t rosidl_buffer_uint8_clone(
  const void * source_ptr, void ** buffer_ptr)
{
  if (!source_ptr || !buffer_ptr) {
    return ROSIDL_BUFFER_RET_INVALID_ARGUMENT;
  }
  *buffer_ptr = nullptr;
  try {
    const auto * source = static_cast<const rosidl::Buffer<uint8_t> *>(source_ptr);
    *buffer_ptr = new rosidl::Buffer<uint8_t>(*source);
    return ROSIDL_BUFFER_RET_OK;
  } catch (const std::bad_alloc &) {
    return ROSIDL_BUFFER_RET_BAD_ALLOC;
  } catch (...) {
    return ROSIDL_BUFFER_RET_ERROR;
  }
}

rosidl_buffer_ret_t rosidl_buffer_uint8_size(
  const void * buffer_ptr, size_t * size)
{
  if (!buffer_ptr || !size) {
    return ROSIDL_BUFFER_RET_INVALID_ARGUMENT;
  }
  try {
    *size = static_cast<const rosidl::Buffer<uint8_t> *>(buffer_ptr)->size();
    return ROSIDL_BUFFER_RET_OK;
  } catch (...) {
    return ROSIDL_BUFFER_RET_ERROR;
  }
}

rosidl_buffer_ret_t rosidl_buffer_uint8_backend_name(
  const void * buffer_ptr, char * output, size_t output_capacity, size_t * required_size)
{
  if (!buffer_ptr || !required_size || (!output && output_capacity != 0)) {
    return ROSIDL_BUFFER_RET_INVALID_ARGUMENT;
  }
  try {
    const std::string backend =
      static_cast<const rosidl::Buffer<uint8_t> *>(buffer_ptr)->get_backend_type();
    *required_size = backend.size() + 1;
    if (!output) {
      return ROSIDL_BUFFER_RET_OK;
    }
    if (output_capacity < *required_size) {
      return ROSIDL_BUFFER_RET_INVALID_ARGUMENT;
    }
    std::memcpy(output, backend.c_str(), *required_size);
    return ROSIDL_BUFFER_RET_OK;
  } catch (...) {
    return ROSIDL_BUFFER_RET_ERROR;
  }
}

rosidl_buffer_ret_t rosidl_buffer_uint8_copy_to(
  const void * buffer_ptr, uint8_t * output, size_t output_size)
{
  if (!buffer_ptr || (!output && output_size != 0)) {
    return ROSIDL_BUFFER_RET_INVALID_ARGUMENT;
  }
  try {
    const auto * buffer = static_cast<const rosidl::Buffer<uint8_t> *>(buffer_ptr);
    if (output_size < buffer->size()) {
      return ROSIDL_BUFFER_RET_INVALID_ARGUMENT;
    }
    const std::vector<uint8_t> data = buffer->to_vector();
    if (!data.empty()) {
      std::copy(data.begin(), data.end(), output);
    }
    return ROSIDL_BUFFER_RET_OK;
  } catch (const std::bad_alloc &) {
    return ROSIDL_BUFFER_RET_BAD_ALLOC;
  } catch (...) {
    return ROSIDL_BUFFER_RET_ERROR;
  }
}

rosidl_buffer_ret_t rosidl_buffer_uint8_are_equal(
  const void * lhs_ptr, const void * rhs_ptr, bool * are_equal)
{
  if (!lhs_ptr || !rhs_ptr || !are_equal) {
    return ROSIDL_BUFFER_RET_INVALID_ARGUMENT;
  }
  try {
    const auto * lhs = static_cast<const rosidl::Buffer<uint8_t> *>(lhs_ptr);
    const auto * rhs = static_cast<const rosidl::Buffer<uint8_t> *>(rhs_ptr);
    *are_equal = lhs->to_vector() == rhs->to_vector();
    return ROSIDL_BUFFER_RET_OK;
  } catch (const std::bad_alloc &) {
    return ROSIDL_BUFFER_RET_BAD_ALLOC;
  } catch (...) {
    return ROSIDL_BUFFER_RET_ERROR;
  }
}

rosidl_buffer_ret_t rosidl_buffer_uint8_equals_data(
  const void * buffer_ptr, const uint8_t * data, size_t size, bool * are_equal)
{
  if (!buffer_ptr || (!data && size != 0) || !are_equal) {
    return ROSIDL_BUFFER_RET_INVALID_ARGUMENT;
  }
  try {
    const auto * buffer = static_cast<const rosidl::Buffer<uint8_t> *>(buffer_ptr);
    if (buffer->size() != size) {
      *are_equal = false;
      return ROSIDL_BUFFER_RET_OK;
    }
    const std::vector<uint8_t> buffer_data = buffer->to_vector();
    *are_equal = std::equal(buffer_data.begin(), buffer_data.end(), data);
    return ROSIDL_BUFFER_RET_OK;
  } catch (const std::bad_alloc &) {
    return ROSIDL_BUFFER_RET_BAD_ALLOC;
  } catch (...) {
    return ROSIDL_BUFFER_RET_ERROR;
  }
}

void rosidl_buffer_uint8_throw_if_not_cpu(const void * buffer_ptr)
{
  const auto * buf = static_cast<const rosidl::Buffer<uint8_t> *>(buffer_ptr);
  buf->throw_if_not_cpu_backend();
}

void rosidl_buffer_uint8_destroy(void * buffer_ptr)
{
  if (buffer_ptr) {
    delete static_cast<rosidl::Buffer<uint8_t> *>(buffer_ptr);
  }
}

}  // extern "C"
