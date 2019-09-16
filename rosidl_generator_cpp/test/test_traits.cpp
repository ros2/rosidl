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
#include "rosidl_generator_cpp/msg/string.hpp"
#include "rosidl_generator_cpp/srv/string.hpp"

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
  ASSERT_TRUE(is_message<rosidl_generator_cpp::msg::Empty>());
  ASSERT_FALSE(is_service<rosidl_generator_cpp::msg::Empty>());
  ASSERT_FALSE(is_service_request<rosidl_generator_cpp::msg::Empty>());
  ASSERT_FALSE(is_service_response<rosidl_generator_cpp::msg::Empty>());

  // A message is not a service
  ASSERT_TRUE(is_message<rosidl_generator_cpp::msg::String>());
  ASSERT_FALSE(is_service<rosidl_generator_cpp::msg::String>());
  ASSERT_FALSE(is_service_request<rosidl_generator_cpp::msg::String>());
  ASSERT_FALSE(is_service_response<rosidl_generator_cpp::msg::String>());

  // Other datatypes should have is_message == false
  ASSERT_FALSE(is_message<double>());
  ASSERT_FALSE(is_message<Message>());

  // Unless the template has been specifically instantiated for the type
  ASSERT_TRUE(is_message<Message2>());
}

TEST(Test_rosidl_generator_traits, is_service) {
  using String = rosidl_generator_cpp::srv::String;
  using StringReq = String::Request;
  using StringResp = String::Response;

  ASSERT_TRUE(is_service<String>());
  ASSERT_FALSE(is_message<String>());
  ASSERT_FALSE(is_service_request<String>());
  ASSERT_FALSE(is_service_response<String>());

  // Requests are additionally messages
  ASSERT_FALSE(is_service<StringReq>());
  ASSERT_TRUE(is_message<StringReq>());
  ASSERT_TRUE(is_service_request<StringReq>());
  ASSERT_FALSE(is_service_response<StringReq>());

  // Responses are additionally messages
  ASSERT_FALSE(is_service<StringResp>());
  ASSERT_TRUE(is_message<StringResp>());
  ASSERT_FALSE(is_service_request<StringResp>());
  ASSERT_TRUE(is_service_response<StringResp>());
}
