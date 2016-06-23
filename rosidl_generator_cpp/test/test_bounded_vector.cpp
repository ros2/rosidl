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

#include <gtest/gtest.h>

#include "rosidl_generator_cpp/bounded_vector.hpp"


TEST(rosidl_generator_cpp, bounded_vector) {
  rosidl_generator_cpp::BoundedVector<int, 2> v;
  ASSERT_EQ(v.size(), 0u);
  ASSERT_EQ(v.max_size(), 2u);
  v.push_back(1);
  v.push_back(2);
  ASSERT_THROW(v.push_back(3), std::length_error);
  ASSERT_THROW(v.resize(3), std::length_error);
  v.resize(1);
  ASSERT_EQ(v.size(), 1u);
  v = {1, 2};
  ASSERT_EQ(v.size(), 2u);
  auto l = {1, 2, 3};
  ASSERT_THROW(v = l, std::length_error);
}
