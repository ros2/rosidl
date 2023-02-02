// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#ifndef ROSIDL_TYPESUPPORT_CPP__SERVICE_TYPE_SUPPORT_HPP_
#define ROSIDL_TYPESUPPORT_CPP__SERVICE_TYPE_SUPPORT_HPP_

#include <cstring>
#include <stdexcept>

#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_runtime_c/visibility_control.h"

namespace rosidl_typesupport_cpp
{

template<typename T>
const rosidl_service_type_support_t * get_service_type_support_handle();

template<typename T>
void * service_create_event_message(
  const rosidl_service_introspection_info_t * info,
  rcutils_allocator_t * allocator,
  const void * request_message,
  const void * response_message)
{
  if (nullptr == info) {
    throw std::invalid_argument("service introspection info struct cannot be null");
  }
  if (nullptr == allocator) {
    throw std::invalid_argument("allocator cannot be null");
  }
  auto * event_msg = static_cast<typename T::Event *>(
    allocator->allocate(sizeof(typename T::Event), allocator->state));
  if (nullptr == event_msg) {
    throw std::invalid_argument("allocation failed for service event message");
  }
  event_msg = new(event_msg) typename T::Event();

  event_msg->info.set__event_type(info->event_type);
  event_msg->info.set__sequence_number(info->sequence_number);
  event_msg->info.stamp.set__sec(info->stamp_sec);
  event_msg->info.stamp.set__nanosec(info->stamp_nanosec);

  std::memcpy(&event_msg->info.client_gid[0], info->client_gid, 16);

  if (nullptr != request_message) {
    event_msg->request.push_back(*static_cast<const typename T::Request *>(request_message));
  }
  if (nullptr != response_message) {
    event_msg->response.push_back(*static_cast<const typename T::Response *>(response_message));
  }
  return event_msg;
}

template<typename T>
bool service_destroy_event_message(
  void * event_msg,
  rcutils_allocator_t * allocator)
{
  auto * event_msg_ = static_cast<typename T::Event *>(event_msg);
  using EventT = typename T::Event;
  event_msg_->~EventT();
  allocator->deallocate(event_msg, allocator->state);
  return true;
}

}  // namespace rosidl_typesupport_cpp

#endif  // ROSIDL_TYPESUPPORT_CPP__SERVICE_TYPE_SUPPORT_HPP_
