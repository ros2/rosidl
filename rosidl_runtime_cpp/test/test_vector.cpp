// Copyright 2022 Open Source Robotics Foundation, Inc.
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
#include <cstring>
#include <vector>

#include "rosidl_runtime_cpp/vector.hpp"

namespace
{

template<typename T>
class VectorTest : public ::testing::Test
{
};

using SomeLiteralTypes =
  testing::Types<bool, char, int, unsigned int>;
TYPED_TEST_SUITE(VectorTest, SomeLiteralTypes);

// cppcheck-suppress syntaxError
TYPED_TEST(VectorTest, CanBeListInitialized) {
  rosidl_runtime_cpp::Vector<TypeParam> vector;
  EXPECT_TRUE(vector.empty());
  vector = {TypeParam{0}};
  EXPECT_EQ(vector.size(), 1u);
  vector = {{TypeParam{0}, TypeParam{1}, TypeParam{0}}};
  EXPECT_EQ(vector.size(), 3u);
}

TYPED_TEST(VectorTest, UsesContiguousStorage) {
  rosidl_runtime_cpp::Vector<TypeParam> vector;
  EXPECT_TRUE(vector.empty());
  vector.push_back(TypeParam{0});
  EXPECT_EQ(
    0, std::memcmp(
      &vector[0], &vector[0],
      sizeof(TypeParam) * vector.size()));
}

TYPED_TEST(VectorTest, CanInteractWithStdVector) {
  std::vector<TypeParam> std_vector{{TypeParam{0}}};

  rosidl_runtime_cpp::Vector<TypeParam>
  vector_constructed_from_std_vector{std_vector};
  EXPECT_EQ(TypeParam{0}, vector_constructed_from_std_vector[0]);

  rosidl_runtime_cpp::Vector<TypeParam>
  vector_assigned_to_std_vector;
  vector_assigned_to_std_vector = std_vector;
  EXPECT_EQ(TypeParam{0}, vector_assigned_to_std_vector[0]);

  rosidl_runtime_cpp::Vector<TypeParam> vector{{TypeParam{1}}};

  std::vector<TypeParam> std_vector_constructed_from_vector{vector};
  EXPECT_EQ(TypeParam{1}, std_vector_constructed_from_vector[0]);

  std::vector<TypeParam> std_vector_assigned_to_vector;
  std_vector_assigned_to_vector = vector;
  EXPECT_EQ(TypeParam{1}, std_vector_assigned_to_vector[0]);
}

}  // namespace
