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

// Loaded by test_buffer_dso with dlopen(RTLD_LOCAL) and built with hidden
// visibility, so its CpuBufferImpl<uint8_t> RTTI is not shared with the host.

#include <cstdint>
#include <memory>
#include <stdexcept>

#include "rosidl_buffer/buffer.hpp"
#include "rosidl_buffer/buffer_impl_base.hpp"
#include "rosidl_buffer/cpu_buffer_impl.hpp"

#define PLUGIN_EXPORT extern "C" __attribute__((visibility("default")))

/// Create a CPU impl holding {9, 9, 9, 9}; ownership passes to the caller.
PLUGIN_EXPORT rosidl::BufferImplBase<uint8_t> * buffer_dso_plugin_make_cpu_impl()
{
  auto impl = std::make_unique<rosidl::CpuBufferImpl<uint8_t>>();
  impl->get_storage().assign(4, 9);
  return impl.release();
}

/// Take ownership of an impl created by the host, wrap it in a Buffer here and
/// push_back(42). Returns the new size, or -1 if the Buffer did not treat it as CPU.
PLUGIN_EXPORT int64_t buffer_dso_plugin_push_back_on(rosidl::BufferImplBase<uint8_t> * raw)
{
  rosidl::Buffer<uint8_t> buffer{std::unique_ptr<rosidl::BufferImplBase<uint8_t>>(raw)};
  try {
    buffer.push_back(42);
  } catch (const std::runtime_error &) {
    return -1;
  }
  return static_cast<int64_t>(buffer.size());
}
