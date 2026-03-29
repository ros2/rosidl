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

#ifndef ROSIDL_BUFFER_BACKEND__BUFFER_DESCRIPTOR_OPS_HPP_
#define ROSIDL_BUFFER_BACKEND__BUFFER_DESCRIPTOR_OPS_HPP_

#include <functional>
#include <memory>

#include "rmw/topic_endpoint_info.h"

namespace rosidl
{

/// Type-erased descriptor conversion callbacks used by the serialization layer.
struct BufferDescriptorOps
{
  /// Create descriptor with endpoint awareness.
  /// Input pointer (`impl`) is a non-owning, read-only, type-erased
  /// `BufferImplBase<T>` instance.
  /// Return value is a type-erased backend descriptor message instance.
  std::function<std::shared_ptr<void>(const void *,
    const rmw_topic_endpoint_info_t &)> create_descriptor_with_endpoint;

  /// Create buffer impl from descriptor with endpoint awareness.
  /// Input pointer (`descriptor`) is a non-owning, type-erased backend descriptor
  /// message (read-only).
  /// Return value is a type-erased unique pointer to a newly created
  /// `BufferImplBase<T>` instance.
  std::function<std::unique_ptr<void, void (*)(void *)>(const void *,
    const rmw_topic_endpoint_info_t &)> from_descriptor_with_endpoint;
};

}  // namespace rosidl

#endif  // ROSIDL_BUFFER_BACKEND__BUFFER_DESCRIPTOR_OPS_HPP_
