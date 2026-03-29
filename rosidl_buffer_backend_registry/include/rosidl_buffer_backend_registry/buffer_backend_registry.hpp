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

#include <memory>
#include <string>
#include <vector>

#include "rosidl_buffer_backend/buffer_backend.hpp"
#include "rosidl_buffer_backend_registry/visibility_control.h"

// Forward declare pluginlib ClassLoader to avoid header dependency
namespace pluginlib
{
template<class T>
class ClassLoader;
}  // namespace pluginlib

namespace rosidl_buffer_backend_registry
{

/// Registry for discovering and managing buffer backend plugins.
/// Uses pluginlib for dynamic plugin discovery and loading.
class BufferBackendRegistry
{
public:
  ROSIDL_BUFFER_BACKEND_REGISTRY_PUBLIC
  BufferBackendRegistry();

  ROSIDL_BUFFER_BACKEND_REGISTRY_PUBLIC
  ~BufferBackendRegistry();

  /// Create a backend instance by plugin class name.
  /// Backends loaded through pluginlib are instantiated per call.
  ROSIDL_BUFFER_BACKEND_REGISTRY_PUBLIC
  std::shared_ptr<rosidl::BufferBackend> create_backend_instance(const std::string & name);

  /// Get names of all registered backends.
  ROSIDL_BUFFER_BACKEND_REGISTRY_PUBLIC
  std::vector<std::string> get_backend_names() const;

  // Non-copyable, non-movable
  BufferBackendRegistry(const BufferBackendRegistry &) = delete;
  BufferBackendRegistry & operator=(const BufferBackendRegistry &) = delete;
  BufferBackendRegistry(BufferBackendRegistry &&) = delete;
  BufferBackendRegistry & operator=(BufferBackendRegistry &&) = delete;

private:
  /// Query pluginlib for declared backend classes and populate plugin_backend_classes_.
  void load_plugins();

  std::vector<std::string> plugin_backend_classes_;
  std::unique_ptr<pluginlib::ClassLoader<rosidl::BufferBackend>> loader_;
};

}  // namespace rosidl_buffer_backend_registry

#endif  // ROSIDL_BUFFER_BACKEND_REGISTRY__BUFFER_BACKEND_REGISTRY_HPP_
