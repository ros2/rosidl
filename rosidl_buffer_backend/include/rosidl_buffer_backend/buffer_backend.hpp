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

#ifndef ROSIDL_BUFFER_BACKEND__BUFFER_BACKEND_HPP_
#define ROSIDL_BUFFER_BACKEND__BUFFER_BACKEND_HPP_

#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <set>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rmw/topic_endpoint_info.h"
#include "rosidl_runtime_c/message_type_support_struct.h"

namespace rosidl
{

/// Upper bound (in bytes) on the serialized size of any buffer backend
/// descriptor message. Backends must ensure that every descriptor produced
/// by create_descriptor_with_endpoint() serializes to no more than this
/// many bytes.
inline constexpr size_t kMaxBufferDescriptorSize = 4096;

/// Abstract interface for vendor-specific buffer backend implementations.
class BufferBackend
{
public:
  virtual ~BufferBackend() = default;

  /// Get the backend type name
  virtual std::string get_backend_type() const = 0;

  /// Get the backend metadata string.
  /// This is used by the backend to pass backend-specific information to other endpoints.
  virtual std::string get_backend_metadata() const
  {
    return "";
  }

  /// Get the message type support handle for this backend's descriptor type.
  /// Implementations should return the generic aggregate handle, typically via
  /// `rosidl_typesupport_cpp::get_message_type_support_handle<YourDescriptor>()`.
  /// The consuming RMW resolves this aggregate to the concrete per-typesupport
  /// library handle it needs at runtime (e.g. rosidl_typesupport_fastrtps_cpp),
  /// so the plugin itself stays RMW-agnostic.
  virtual const rosidl_message_type_support_t * get_descriptor_type_support() const = 0;

  /// Create an empty descriptor message instance for this backend.
  /// The returned object must match get_descriptor_type_support().
  virtual std::shared_ptr<void> create_empty_descriptor() const = 0;

  /// Create a descriptor message with endpoint awareness.
  /// @param impl Non-owning, type-erased BufferImplBase pointer (read-only).
  ///        Backends that need internal bookkeeping (e.g. IPC handle setup,
  ///        memory pinning) should use `mutable` members in their impl class.
  /// @param endpoint_info Endpoint info for the peer.
  /// @return Type-erased descriptor message, or nullptr if the backend cannot
  ///         handle this endpoint (e.g., the peer does not support this backend).
  ///         Returning nullptr signals the serialization layer to fall back to
  ///         CPU-based serialization. The descriptor must not exceed
  ///         kMaxBufferDescriptorSize bytes.
  virtual std::shared_ptr<void> create_descriptor_with_endpoint(
    const void * impl,
    const rmw_topic_endpoint_info_t & endpoint_info) const = 0;

  /// Create a BufferImpl from descriptor with endpoint awareness.
  /// @param descriptor Non-owning, type-erased descriptor message pointer (read-only).
  /// @param endpoint_info Endpoint info for the peer.
  /// @return Type-erased unique pointer to a newly created BufferImplBase instance.
  ///         The custom deleter ensures correct destruction across the plugin boundary.
  virtual std::unique_ptr<void, void (*)(void *)> from_descriptor_with_endpoint(
    const void * descriptor,
    const rmw_topic_endpoint_info_t & endpoint_info) const = 0;

  /// Hook invoked when creating a local endpoint.
  virtual void on_creating_endpoint(
    const rmw_topic_endpoint_info_t & endpoint_info) const
  {
    (void)endpoint_info;
  }

  /// Hook invoked when discovering a remote endpoint.
  /// Returns {compatible, groups of endpoint GID hashes}.
  /// @param endpoint_info Information about the discovered endpoint
  /// @param existing_endpoints List of existing endpoints for grouping decisions
  /// @param endpoint_supported_backends Map of backend type to backend metadata string
  ///        for the discovered endpoint
  virtual std::pair<bool, std::vector<std::set<uint32_t>>> on_discovering_endpoint(
    const rmw_topic_endpoint_info_t & endpoint_info,
    const std::vector<rmw_topic_endpoint_info_t> & existing_endpoints,
    const std::unordered_map<std::string, std::string> & endpoint_supported_backends)
  {
    (void)endpoint_info;
    (void)existing_endpoints;
    (void)endpoint_supported_backends;
    return {true, {}};
  }
};

}  // namespace rosidl

#endif  // ROSIDL_BUFFER_BACKEND__BUFFER_BACKEND_HPP_
