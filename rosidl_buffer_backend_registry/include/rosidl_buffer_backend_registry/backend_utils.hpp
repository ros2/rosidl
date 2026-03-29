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

#ifndef ROSIDL_BUFFER_BACKEND_REGISTRY__BACKEND_UTILS_HPP_
#define ROSIDL_BUFFER_BACKEND_REGISTRY__BACKEND_UTILS_HPP_

#include <algorithm>
#include <cstdint>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rosidl_buffer_backend/buffer_backend.hpp"
#include "rmw/topic_endpoint_info.h"

namespace rosidl_buffer_backend_registry
{

/// Collect metadata strings from every loaded backend instance.
/// @param[in] backend_instances Map of backend name to backend instance.
/// @return Map of backend name to its metadata string (null backends are skipped).
inline std::unordered_map<std::string, std::string> get_all_backend_metadata(
  const std::unordered_map<std::string, std::shared_ptr<rosidl::BufferBackend>> & backend_instances)
{
  std::unordered_map<std::string, std::string> backend_metadata;
  for (const auto & [backend_name, backend] : backend_instances) {
    if (backend) {
      backend_metadata[backend_name] = backend->get_backend_metadata();
    }
  }
  return backend_metadata;
}

/// Notify every loaded backend that a local endpoint has been created.
/// @param[in] backend_instances Map of backend name to backend instance.
/// @param[in] endpoint_info Endpoint that was just created.
inline void notify_endpoint_created(
  const std::unordered_map<std::string, std::shared_ptr<rosidl::BufferBackend>> & backend_instances,
  const rmw_topic_endpoint_info_t & endpoint_info)
{
  for (const auto & [_, backend] : backend_instances) {
    if (backend) {
      backend->on_creating_endpoint(endpoint_info);
    }
  }
}

/// Notify every loaded backend that a remote endpoint has been discovered and
/// collect compatibility / grouping results.
/// @param[in]  backend_instances Map of backend name to backend instance.
/// @param[in]  endpoint_info Information about the discovered endpoint.
/// @param[in]  existing_endpoints Endpoints already known on this topic.
/// @param[out] backend_endpoint_groups Updated per-backend endpoint GID-hash
///             groupings (cleared to empty for null backends).
/// @param[in]  endpoint_supported_backends Backend-type-to-metadata map
///             advertised by the discovered endpoint.
/// @return Map of backend name to compatibility flag (false for null backends).
inline std::unordered_map<std::string, bool> notify_endpoint_discovered(
  const std::unordered_map<std::string, std::shared_ptr<rosidl::BufferBackend>> & backend_instances,
  const rmw_topic_endpoint_info_t & endpoint_info,
  const std::vector<rmw_topic_endpoint_info_t> & existing_endpoints,
  std::unordered_map<std::string, std::vector<std::set<uint32_t>>> & backend_endpoint_groups,
  const std::unordered_map<std::string, std::string> & endpoint_supported_backends)
{
  std::unordered_map<std::string, bool> backend_compatibility;
  for (const auto & [backend_name, backend] : backend_instances) {
    if (!backend) {
      backend_compatibility[backend_name] = false;
      backend_endpoint_groups[backend_name] = {};
      continue;
    }
    auto result = backend->on_discovering_endpoint(
      endpoint_info, existing_endpoints, endpoint_supported_backends);
    backend_compatibility[backend_name] = result.first;
    backend_endpoint_groups[backend_name] = std::move(result.second);
  }
  return backend_compatibility;
}

/// Check whether two backend name lists share at least one common entry.
/// @return true if any backend name appears in both \p a and \p b.
inline bool backends_compatible(
  const std::vector<std::string> & a,
  const std::vector<std::string> & b)
{
  for (const auto & entry : a) {
    if (std::find(b.begin(), b.end(), entry) != b.end()) {
      return true;
    }
  }
  return false;
}

/// Return the de-duplicated intersection of two backend name lists,
/// preserving the order in which names appear in \p a.
inline std::vector<std::string> get_common_backends(
  const std::vector<std::string> & a,
  const std::vector<std::string> & b)
{
  std::vector<std::string> common;
  for (const auto & entry : a) {
    if (std::find(b.begin(), b.end(), entry) != b.end() &&
      std::find(common.begin(), common.end(), entry) == common.end())
    {
      common.push_back(entry);
    }
  }
  return common;
}

}  // namespace rosidl_buffer_backend_registry

#endif  // ROSIDL_BUFFER_BACKEND_REGISTRY__BACKEND_UTILS_HPP_
