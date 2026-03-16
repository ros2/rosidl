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

#include "rosidl_buffer_backend_registry/buffer_backend_registry.hpp"

#include <mutex>
#include <stdexcept>

#include <pluginlib/class_loader.hpp>

namespace rosidl_buffer_backend_registry
{

BufferBackendRegistry::BufferBackendRegistry()
: plugins_loaded_(false)
{
  try {
    loader_ = std::make_unique<pluginlib::ClassLoader<rosidl::BufferBackend>>(
      "rosidl_buffer_backend",
      "rosidl::BufferBackend");
  } catch (const std::exception & e) {
    // Silently ignore if pluginlib isn't available - will work without plugins
    loader_ = nullptr;
  }
}

BufferBackendRegistry::~BufferBackendRegistry()
{
  // CRITICAL: Clear all global state before ClassLoader is destroyed
  clear_global_state();
}

BufferBackendRegistry & BufferBackendRegistry::get_instance()
{
  static BufferBackendRegistry instance;
  return instance;
}

std::shared_ptr<rosidl::BufferBackend> BufferBackendRegistry::get_backend(const std::string & name)
{
  // Ensure plugins are loaded on first access
  if (!plugins_loaded_) {
    load_plugins();
  }

  auto it = backends_.find(name);
  if (it != backends_.end()) {
    return it->second;
  }

  return nullptr;
}

void BufferBackendRegistry::register_backend(
  const std::string & name,
  std::shared_ptr<rosidl::BufferBackend> backend)
{
  static std::mutex mutex;
  std::lock_guard<std::mutex> lock(mutex);
  backends_[name] = backend;
}

void BufferBackendRegistry::load_plugins()
{
  static std::once_flag load_flag;
  std::call_once(
    load_flag, [this]() {
      if (!loader_) {
        plugins_loaded_ = true;
        return;
      }

      try {
        auto declared_classes = loader_->getDeclaredClasses();
        if (declared_classes.empty()) {
          RCUTILS_LOG_INFO_NAMED(
            "rosidl_buffer_backend_registry",
            "No buffer backend plugins found");
        } else {
          RCUTILS_LOG_INFO_NAMED(
            "rosidl_buffer_backend_registry",
            "Discovered %zu buffer backend plugin(s)",
            declared_classes.size());
          for (const auto & class_name : declared_classes) {
            try {
              auto backend = loader_->createSharedInstance(class_name);
              register_backend(class_name, backend);
              RCUTILS_LOG_INFO_NAMED(
                "rosidl_buffer_backend_registry",
                "Loaded buffer backend plugin: %s",
                class_name.c_str());
            } catch (const std::exception & e) {
              RCUTILS_LOG_ERROR_NAMED(
                "rosidl_buffer_backend_registry",
                "Failed to load %s: %s",
                class_name.c_str(), e.what());
              continue;
            }
          }
        }
      } catch (const std::exception & e) {
        RCUTILS_LOG_ERROR_NAMED(
          "rosidl_buffer_backend_registry",
          "Buffer backend plugin discovery error: %s",
          e.what());
      }

      plugins_loaded_ = true;
    });
}

std::vector<std::string> BufferBackendRegistry::get_backend_names() const
{
  std::vector<std::string> names;
  names.reserve(backends_.size());
  for (const auto & pair : backends_) {
    names.push_back(pair.first);
  }
  return names;
}

std::vector<std::string> BufferBackendRegistry::get_backend_types()
{
  // Ensure plugins are loaded
  if (!plugins_loaded_) {
    load_plugins();
  }

  std::vector<std::string> backend_types;

  // Always include CPU backend
  backend_types.push_back("cpu");

  // Get additional backends from loaded plugins
  for (const auto & pair : backends_) {
    auto & backend = pair.second;
    if (backend) {
      std::string backend_type = backend->get_backend_type();
      // Avoid duplicating CPU if it's in the registry
      if (backend_type != "cpu") {
        backend_types.push_back(backend_type);
      }
    }
  }

  return backend_types;
}

std::unordered_map<std::string, std::string> BufferBackendRegistry::get_all_aux_info()
{
  // Ensure plugins are loaded
  if (!plugins_loaded_) {
    load_plugins();
  }

  std::unordered_map<std::string, std::string> aux_info;
  for (const auto & pair : backends_) {
    if (pair.second) {
      aux_info[pair.first] = pair.second->get_backend_aux_info();
    }
  }
  return aux_info;
}

void BufferBackendRegistry::notify_endpoint_created(
  const rmw_topic_endpoint_info_t & endpoint_info)
{
  for (const auto & pair : backends_) {
    if (pair.second) {
      pair.second->on_creating_endpoint(endpoint_info);
    }
  }
}

std::unordered_map<std::string, bool> BufferBackendRegistry::notify_endpoint_discovered(
  const rmw_topic_endpoint_info_t & endpoint_info,
  const std::vector<rmw_topic_endpoint_info_t> & existing_endpoints,
  std::unordered_map<std::string, std::vector<std::set<uint32_t>>> & backend_endpoint_groups,
  const std::unordered_map<std::string, std::string> & endpoint_supported_backends)
{
  std::unordered_map<std::string, bool> backend_compatibility;
  for (const auto & pair : backends_) {
    const auto & backend_name = pair.first;
    const auto & backend = pair.second;
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

bool BufferBackendRegistry::backends_compatible(
  const std::vector<std::string> & a,
  const std::vector<std::string> & b)
{
  for (const auto & backend_a : a) {
    for (const auto & backend_b : b) {
      if (backend_a == backend_b) {
        return true;
      }
    }
  }
  return false;
}

std::vector<std::string> BufferBackendRegistry::get_common_backends(
  const std::vector<std::string> & a,
  const std::vector<std::string> & b)
{
  std::vector<std::string> common;
  for (const auto & backend_a : a) {
    for (const auto & backend_b : b) {
      if (backend_a == backend_b) {
        bool already_added = false;
        for (const auto & c : common) {
          if (c == backend_a) {
            already_added = true;
            break;
          }
        }
        if (!already_added) {
          common.push_back(backend_a);
        }
      }
    }
  }
  return common;
}

void BufferBackendRegistry::clear_global_state()
{
  // CRITICAL: Clear backends_ map to release shared_ptr to plugin instances
  // before ClassLoader is destroyed.
  backends_.clear();

  // Note: Global serialization maps in rosidl_typesupport_fastrtps_cpp are cleared
  // by rmw_shutdown() which is called before this destructor runs.
}

}  // namespace rosidl_buffer_backend_registry
