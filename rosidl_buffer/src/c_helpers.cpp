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

#include <cstdint>

#include "rosidl_buffer/buffer.hpp"

extern "C" {

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
