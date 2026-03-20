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

#ifndef ROSIDL_BUFFER_BACKEND_REGISTRY__BUFFER_BACKEND_REGISTRY_HPP_
#define ROSIDL_BUFFER_BACKEND_REGISTRY__BUFFER_BACKEND_REGISTRY_HPP_

#include <map>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include "rosidl_buffer_backend/buffer_backend.hpp"
#include "rosidl_buffer_backend_registry/visibility_control.h"

#include "rmw/topic_endpoint_info.h"

// Forward declare pluginlib ClassLoader to avoid header dependency
namespace pluginlib
{
template<class T>
class ClassLoader;
}  // namespace pluginlib

namespace rosidl_buffer_backend_registry
{

/// Singleton registry for discovering and managing buffer backend plugins.
/// Uses pluginlib for dynamic plugin discovery and loading.
///
/// Thread-safety model: load_plugins() is guarded by std::call_once and
/// register_backend() is mutex-protected, so initialization is thread-safe.
/// After load_plugins() completes, the backends map is effectively immutable
/// and all read-only accessors (get_backend, get_backend_names, etc.) are
/// safe for concurrent use without additional synchronization.
class BufferBackendRegistry
{
public:
  /// Get the singleton instance.
  ROSIDL_BUFFER_BACKEND_REGISTRY_PUBLIC
  static BufferBackendRegistry & get_instance();

  /// Get a registered backend by name (e.g., "cpu", "cuda").
  /// @return The backend, or nullptr if not found.
  ROSIDL_BUFFER_BACKEND_REGISTRY_PUBLIC
  std::shared_ptr<rosidl::BufferBackend> get_backend(const std::string & name);

  /// Manually register a backend (for built-in backends or testing).
  /// Thread-safe (mutex-protected).
  ROSIDL_BUFFER_BACKEND_REGISTRY_PUBLIC
  void register_backend(const std::string & name, std::shared_ptr<rosidl::BufferBackend> backend);

  /// Load all available backend plugins via pluginlib.
  /// Called automatically on first get_backend() if not already loaded.
  /// Thread-safe (guarded by std::call_once).
  ROSIDL_BUFFER_BACKEND_REGISTRY_PUBLIC
  void load_plugins();

  /// Get names of all registered backends.
  ROSIDL_BUFFER_BACKEND_REGISTRY_PUBLIC
  std::vector<std::string> get_backend_names() const;

  /// Get backend type strings for all registered backends, always including "cpu".
  /// Each backend's get_backend_type() is queried (e.g., "cuda", "rocm").
  ROSIDL_BUFFER_BACKEND_REGISTRY_PUBLIC
  std::vector<std::string> get_backend_types();

  /// Collect aux info from all registered backends.
  /// @return Map of backend name to aux info string.
  ROSIDL_BUFFER_BACKEND_REGISTRY_PUBLIC
  std::unordered_map<std::string, std::string> get_all_aux_info();

  /// Notify all backends that a local endpoint is being created.
  ROSIDL_BUFFER_BACKEND_REGISTRY_PUBLIC
  void notify_endpoint_created(const rmw_topic_endpoint_info_t & endpoint_info);

  /// Notify all backends that a remote endpoint has been discovered.
  /// @param endpoint_info Information about the discovered endpoint.
  /// @param existing_endpoints List of existing endpoints for grouping decisions.
  /// @param backend_endpoint_groups In/out parameter for backend-specific grouping information.
  /// @param endpoint_supported_backends Backend aux info from the discovered endpoint.
  /// @return Map of backend name to compatibility boolean.
  ROSIDL_BUFFER_BACKEND_REGISTRY_PUBLIC
  std::unordered_map<std::string, bool> notify_endpoint_discovered(
    const rmw_topic_endpoint_info_t & endpoint_info,
    const std::vector<rmw_topic_endpoint_info_t> & existing_endpoints,
    std::unordered_map<std::string, std::vector<std::set<uint32_t>>> & backend_endpoint_groups,
    const std::unordered_map<std::string, std::string> & endpoint_supported_backends);

  /// Check if two backend type lists have at least one common entry.
  ROSIDL_BUFFER_BACKEND_REGISTRY_PUBLIC
  static bool backends_compatible(
    const std::vector<std::string> & a,
    const std::vector<std::string> & b);

  /// Get the intersection of two backend type lists.
  ROSIDL_BUFFER_BACKEND_REGISTRY_PUBLIC
  static std::vector<std::string> get_common_backends(
    const std::vector<std::string> & a,
    const std::vector<std::string> & b);

  /// Clear all backends and release plugin instances.
  /// Called automatically in destructor before ClassLoader is destroyed.
  ROSIDL_BUFFER_BACKEND_REGISTRY_PUBLIC
  void clear_global_state();

  // Non-copyable, non-movable
  BufferBackendRegistry(const BufferBackendRegistry &) = delete;
  BufferBackendRegistry & operator=(const BufferBackendRegistry &) = delete;
  BufferBackendRegistry(BufferBackendRegistry &&) = delete;
  BufferBackendRegistry & operator=(BufferBackendRegistry &&) = delete;

private:
  BufferBackendRegistry();
  ~BufferBackendRegistry();

  std::map<std::string, std::shared_ptr<rosidl::BufferBackend>> backends_;
  std::unique_ptr<pluginlib::ClassLoader<rosidl::BufferBackend>> loader_;
  bool plugins_loaded_;
};

}  // namespace rosidl_buffer_backend_registry

#endif  // ROSIDL_BUFFER_BACKEND_REGISTRY__BUFFER_BACKEND_REGISTRY_HPP_
