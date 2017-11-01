// Copyright 2017 Open Source Robotics Foundation, Inc.
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

#include "rosidl_generator_cpp/srv/complex_type_zero_fill.hpp"

TEST(Test_srv_initialization, no_arg_request_constructor) {
  rosidl_generator_cpp::srv::ComplexTypeZeroFill::Request def;

  ASSERT_EQ(2UL, def.req.size());
  ASSERT_EQ(0UL, def.req[0].x);
  ASSERT_EQ(0UL, def.req[1].x);
  ASSERT_EQ(45UL, def.req[0].y);
  ASSERT_EQ(45UL, def.req[1].y);
  ASSERT_EQ(0UL, def.req[0].z);
  ASSERT_EQ(0UL, def.req[1].z);
}

TEST(Test_srv_initialization, no_arg_response_constructor) {
  rosidl_generator_cpp::srv::ComplexTypeZeroFill::Response def;

  ASSERT_EQ(2UL, def.reply.size());
  ASSERT_EQ(0UL, def.reply[0].x);
  ASSERT_EQ(0UL, def.reply[1].x);
  ASSERT_EQ(45UL, def.reply[0].y);
  ASSERT_EQ(45UL, def.reply[1].y);
  ASSERT_EQ(0UL, def.reply[0].z);
  ASSERT_EQ(0UL, def.reply[1].z);
}

// Note that we very specifically don't add tests for some of the other
// possibilities here (more primitive types, unbounded arrays, other
// initialization parameters, etc).  That's because that is all tested in
// test_msg_initialization.cpp, so we don't need to repeat it here.
