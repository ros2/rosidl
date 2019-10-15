// Copyright 2019 Open Source Robotics Foundation, Inc.
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

#include <gtest/gtest.h>
#include "rosidl_generator_cpp/msg/empty.hpp"
#include "rosidl_generator_cpp/msg/strings.hpp"
#include "rosidl_generator_cpp/srv/empty.hpp"

using rosidl_generator_traits::is_message;
using rosidl_generator_traits::is_service;
using rosidl_generator_traits::is_service_request;
using rosidl_generator_traits::is_service_response;

// Empty testing struct
struct Message {};

// Empty testing struct, with template instantiation
struct Message2 {};

template<>
struct rosidl_generator_traits::is_message<Message2>: std::true_type {};

TEST(Test_rosidl_generator_traits, is_message) {
  // A message is not a service
  using Empty = rosidl_generator_cpp::msg::Empty;
  EXPECT_TRUE(is_message<Empty>());
  EXPECT_FALSE(is_service<Empty>());
  EXPECT_FALSE(is_service_request<Empty>());
  EXPECT_FALSE(is_service_response<Empty>());

  // A message is not a service
  using Strings = rosidl_generator_cpp::msg::Strings;
  EXPECT_TRUE(is_message<Strings>());
  EXPECT_FALSE(is_service<Strings>());
  EXPECT_FALSE(is_service_request<Strings>());
  EXPECT_FALSE(is_service_response<Strings>());

  // Other datatypes should have is_message == false
  EXPECT_FALSE(is_message<double>());
  EXPECT_FALSE(is_message<Message>());

  // Unless the template has been specifically instantiated for the type
  EXPECT_TRUE(is_message<Message2>());
}

TEST(Test_rosidl_generator_traits, is_service) {
  using Service = rosidl_generator_cpp::srv::Empty;
  using ServiceReq = Service::Request;
  using ServiceResp = Service::Response;

  EXPECT_TRUE(is_service<Service>());
  EXPECT_FALSE(is_message<Service>());
  EXPECT_FALSE(is_service_request<Service>());
  EXPECT_FALSE(is_service_response<Service>());

  // Requests are additionally messages
  EXPECT_FALSE(is_service<ServiceReq>());
  EXPECT_TRUE(is_message<ServiceReq>());
  EXPECT_TRUE(is_service_request<ServiceReq>());
  EXPECT_FALSE(is_service_response<ServiceReq>());

  // Responses are additionally messages
  EXPECT_FALSE(is_service<ServiceResp>());
  EXPECT_TRUE(is_message<ServiceResp>());
  EXPECT_FALSE(is_service_request<ServiceResp>());
  EXPECT_TRUE(is_service_response<ServiceResp>());
}
