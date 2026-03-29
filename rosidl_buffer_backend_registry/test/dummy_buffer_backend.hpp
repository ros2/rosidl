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

#ifndef DUMMY_BUFFER_BACKEND_HPP_
#define DUMMY_BUFFER_BACKEND_HPP_

#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_buffer_backend/buffer_backend.hpp"
#include "rosidl_buffer/buffer_impl_base.hpp"
#include "rosidl_buffer/cpu_buffer_impl.hpp"

// For pluginlib export
#include <pluginlib/class_list_macros.hpp>

namespace rosidl_buffer_backend_registry
{
namespace test
{

/// Dummy backend implementation for testing.
/// Acts like CPU backend but adds a marker to verify it's being used.
template<typename T>
class DummyBufferImpl : public rosidl::BufferImplBase<T>
{
public:
  DummyBufferImpl()
  : marker_(0xDEADBEEF) {}

  explicit DummyBufferImpl(size_t size)
  : marker_(0xDEADBEEF)
  {
    data_.resize(size);
  }

  std::string get_backend_type() const override {return "dummy";}

  size_t size() const override {return data_.size();}

  std::unique_ptr<rosidl::BufferImplBase<T>> to_cpu() const override
  {
    auto cpu = std::make_unique<rosidl::CpuBufferImpl<T>>();
    cpu->get_storage() = data_;
    return cpu;
  }

  std::unique_ptr<rosidl::BufferImplBase<T>> clone() const override
  {
    auto cloned = std::make_unique<DummyBufferImpl<T>>();
    cloned->data_ = data_;
    cloned->marker_ = marker_;
    return cloned;
  }

  std::vector<T> & get_data() {return data_;}
  const std::vector<T> & get_data() const {return data_;}

  uint32_t get_marker() const {return marker_;}

private:
  std::vector<T> data_;
  uint32_t marker_;  // Marker to identify this backend
};

/// Dummy backend for testing the BufferBackend interface.
/// Implements all 4+4 methods with test markers.
class DummyBufferBackend : public rosidl::BufferBackend
{
public:
  DummyBufferBackend()
  : call_count_(0) {}

  // Track which methods were called
  mutable size_t call_count_;
  mutable std::string last_method_;

  std::string get_backend_type() const override
  {
    return "dummy";
  }

  const rosidl_message_type_support_t * get_descriptor_type_support() const override
  {
    return nullptr;
  }

  std::shared_ptr<void> create_empty_descriptor() const override
  {
    return nullptr;
  }

  std::shared_ptr<void> create_descriptor_with_endpoint(
    const void * impl,
    const rmw_topic_endpoint_info_t & endpoint_info) const override
  {
    (void)impl;
    (void)endpoint_info;
    return nullptr;
  }

  std::unique_ptr<void, void (*)(void *)> from_descriptor_with_endpoint(
    const void * descriptor,
    const rmw_topic_endpoint_info_t & endpoint_info) const override
  {
    (void)descriptor;
    (void)endpoint_info;
    return {nullptr, [](void *) {}};
  }
};

}  // namespace test
}  // namespace rosidl_buffer_backend_registry

// Export the DummyBufferBackend as a pluginlib plugin
PLUGINLIB_EXPORT_CLASS(
  rosidl_buffer_backend_registry::test::DummyBufferBackend,
  rosidl::BufferBackend)

#endif  // DUMMY_BUFFER_BACKEND_HPP_
