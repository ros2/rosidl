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

#include <algorithm>
#include <stdexcept>

#include <pluginlib/class_loader.hpp>
#include "rcutils/logging_macros.h"

namespace rosidl_buffer_backend_registry
{

BufferBackendRegistry::BufferBackendRegistry()
{
  try {
    loader_ = std::make_unique<pluginlib::ClassLoader<rosidl::BufferBackend>>(
      "rosidl_buffer_backend",
      "rosidl::BufferBackend");
  } catch (const std::exception & e) {
    RCUTILS_LOG_ERROR_NAMED(
      "rosidl_buffer_backend_registry",
      "Failed to create ClassLoader: %s", e.what());
    loader_ = nullptr;
  }
  load_plugins();
}

BufferBackendRegistry::~BufferBackendRegistry() = default;

void BufferBackendRegistry::load_plugins()
{
  if (!loader_) {
    return;
  }

  try {
    auto declared_classes = loader_->getDeclaredClasses();
    if (declared_classes.empty()) {
      RCUTILS_LOG_DEBUG_NAMED(
        "rosidl_buffer_backend_registry",
        "No buffer backend plugins found");
    } else {
      RCUTILS_LOG_DEBUG_NAMED(
        "rosidl_buffer_backend_registry",
        "Discovered %zu buffer backend plugin(s)",
        declared_classes.size());
      for (const auto & class_name : declared_classes) {
        plugin_backend_classes_.push_back(class_name);
        RCUTILS_LOG_DEBUG_NAMED(
          "rosidl_buffer_backend_registry",
          "Discovered buffer backend plugin class: %s",
          class_name.c_str());
      }
    }
  } catch (const std::exception & e) {
    RCUTILS_LOG_ERROR_NAMED(
      "rosidl_buffer_backend_registry",
      "Buffer backend plugin discovery error: %s",
      e.what());
  }
}

std::shared_ptr<rosidl::BufferBackend> BufferBackendRegistry::create_backend_instance(
  const std::string & name)
{
  if (!loader_) {
    return nullptr;
  }

  auto plugin_it = std::find(
    plugin_backend_classes_.begin(), plugin_backend_classes_.end(), name);
  if (plugin_it == plugin_backend_classes_.end()) {
    return nullptr;
  }

  try {
    return loader_->createSharedInstance(name);
  } catch (const std::exception & e) {
    RCUTILS_LOG_ERROR_NAMED(
      "rosidl_buffer_backend_registry",
      "Failed to instantiate backend '%s': %s",
      name.c_str(), e.what());
    return nullptr;
  }
}

std::vector<std::string> BufferBackendRegistry::get_backend_names() const
{
  std::vector<std::string> names;
  names.reserve(plugin_backend_classes_.size());
  for (const auto & class_name : plugin_backend_classes_) {
    if (std::find(names.begin(), names.end(), class_name) == names.end()) {
      names.push_back(class_name);
    }
  }
  return names;
}

}  // namespace rosidl_buffer_backend_registry
