// Copyright 2025 Open Source Robotics Foundation, Inc.
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

#include <array>
#include <cstddef>
#include <type_traits>

#include "rosidl_runtime_cpp/deprecated_helper_array.hpp"


// Suppress deprecation notices to allow unit testing of deprecated functions.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"


TEST(rosidl_generator_cpp, deprecated_helper_array_constructor) {
  rosidl_runtime_cpp::DeprecatedHelperArray<3> dep_array{};
  std::array<std::byte, 3> empty_byte_array{};
  std::array<unsigned char, 3> empty_uchar_array{};

  ASSERT_EQ(dep_array, empty_byte_array);
  ASSERT_EQ(dep_array, empty_uchar_array);

  std::array<std::byte, 3> byte_array = {std::byte{6}, std::byte{1}, std::byte{11}};
  std::array<unsigned char, 3> char_array = {6, 1, 11};

  dep_array = byte_array;
  EXPECT_EQ(dep_array, byte_array);
  EXPECT_EQ(dep_array, char_array);

  // Clear
  dep_array = {0, 0, 0};
  dep_array = char_array;
  EXPECT_EQ(dep_array, byte_array);
  EXPECT_EQ(dep_array, char_array);

  std::array<std::byte, 3> byte_array2 = {std::byte{1}, std::byte{2}, std::byte{3}};
  std::array<unsigned char, 3> char_array2 = {1, 2, 3};

  dep_array = {std::byte{1}, std::byte{2}, std::byte{3}};
  EXPECT_EQ(dep_array, byte_array2);
  EXPECT_EQ(dep_array, char_array2);

  dep_array = {0, 0, 0};
  dep_array = {1, 2, 3};
  EXPECT_EQ(dep_array, byte_array2);
  EXPECT_EQ(dep_array, char_array2);
}

TEST(rosidl_generator_cpp, deprecated_helper_array_operator_index) {
  rosidl_runtime_cpp::DeprecatedHelperArray<3> dep_array{};
  dep_array.fill(std::byte{5});
  EXPECT_EQ(static_cast<std::byte>(dep_array[0]), std::byte{5});
  EXPECT_EQ(static_cast<std::byte>(dep_array[1]), std::byte{5});
  EXPECT_EQ(static_cast<std::byte>(dep_array[2]), std::byte{5});

  dep_array[0] = 10;
  dep_array[1] = std::byte{10};
  dep_array[2] = 10;
  EXPECT_EQ(static_cast<std::byte>(dep_array[0]), std::byte{10});
  EXPECT_EQ(static_cast<std::byte>(dep_array[1]), std::byte{10});
  EXPECT_EQ(dep_array[0], 10);
  EXPECT_EQ(dep_array[1], 10);
}

// TEST(rosidl_generator_cpp, deprecated_helper_array_at) {
//   rosidl_runtime_cpp::DeprecatedHelperArray<3> dep_array{};
//   dep_array.fill(std::byte{7});

//   EXPECT_EQ(static_cast<std::byte>(dep_array.at(0)), std::byte{7});
//   EXPECT_EQ(static_cast<std::byte>(dep_array.at(2)), std::byte{7});

//   dep_array.at(1) = std::byte{11};
//   EXPECT_EQ(static_cast<std::byte>(dep_array.at(1)), std::byte{11});
//   EXPECT_THROW(dep_array.at(3), std::out_of_range);
// }

// TEST(rosidl_generator_cpp, deprecated_helper_array_data) {
//   rosidl_runtime_cpp::DeprecatedHelperArray<3> dep_array{};
//   dep_array.fill(std::byte{9});

//   std::byte * data_ptr = dep_array.data();
//   EXPECT_EQ(*data_ptr, std::byte{9});

//   data_ptr[2] = std::byte{15};
//   EXPECT_EQ(static_cast<std::byte>(dep_array[2]), std::byte{15});
// }

// TEST(rosidl_generator_cpp, deprecated_helper_array_begin_end) {
//   rosidl_runtime_cpp::DeprecatedHelperArray<3> dep_array{};
//   dep_array.fill(std::byte{3});

//   auto it = dep_array.begin();
//   auto end_it = dep_array.end();

//   EXPECT_NE(it, end_it);
//   EXPECT_EQ(static_cast<std::byte>(*it), std::byte{3});

//   int count = 0;
//   for (auto iter = dep_array.begin(); iter != dep_array.end(); ++iter) {
//     EXPECT_EQ(static_cast<std::byte>(*iter), std::byte{3});
//     ++count;
//   }
//   EXPECT_EQ(count, 3);
// }

// TEST(rosidl_generator_cpp, deprecated_helper_array_cbegin_cend) {
//   rosidl_runtime_cpp::DeprecatedHelperArray<3> dep_array{};
//   dep_array.fill(std::byte{4});

//   auto cit = dep_array.cbegin();
//   auto cend_it = dep_array.cend();

//   EXPECT_NE(cit, cend_it);
//   EXPECT_EQ(static_cast<std::byte>(*cit), std::byte{4});
// }

TEST(rosidl_generator_cpp, deprecated_helper_array_empty) {
  rosidl_runtime_cpp::DeprecatedHelperArray<0> empty_array{};
  rosidl_runtime_cpp::DeprecatedHelperArray<3> non_empty_array{};

  EXPECT_TRUE(empty_array.empty());
  EXPECT_FALSE(non_empty_array.empty());
}

// TEST(rosidl_generator_cpp, deprecated_helper_array_front_back) {
//   rosidl_runtime_cpp::DeprecatedHelperArray<3> dep_array{};
//   dep_array[0] = std::byte{7};
//   dep_array[1] = std::byte{8};
//   dep_array[2] = std::byte{9};

//   EXPECT_EQ(static_cast<std::byte>(dep_array.front()), std::byte{7});
//   EXPECT_EQ(static_cast<std::byte>(dep_array.back()), std::byte{9});

//   dep_array.front() = std::byte{10};
//   dep_array.back() = std::byte{11};

//   EXPECT_EQ(static_cast<std::byte>(dep_array[0]), std::byte{10});
//   EXPECT_EQ(static_cast<std::byte>(dep_array[2]), std::byte{11});
// }

TEST(rosidl_generator_cpp, deprecated_helper_array_swap) {
  rosidl_runtime_cpp::DeprecatedHelperArray<3> dep_array1{};
  rosidl_runtime_cpp::DeprecatedHelperArray<3> dep_array2{};

  dep_array1 = {std::byte{1}, std::byte{2}, std::byte{3}};
  dep_array2 = {std::byte{4}, std::byte{5}, std::byte{6}};

  dep_array1.swap(dep_array2);

  EXPECT_EQ(dep_array1, (std::array<std::byte, 3>{std::byte{4}, std::byte{5}, std::byte{6}}));
  EXPECT_EQ(dep_array2, (std::array<std::byte, 3>{std::byte{1}, std::byte{2}, std::byte{3}}));
}


TEST(rosidl_generator_cpp, deprecated_helper_array_size) {
  rosidl_runtime_cpp::DeprecatedHelperArray<3> dep_array;
  EXPECT_EQ(dep_array.size(), 3);
}

TEST(rosidl_generator_cpp, deprecated_helper_array_fill) {
  rosidl_runtime_cpp::DeprecatedHelperArray<3> dep_array;
  std::array<std::byte, 3> byte_array = {std::byte{10}, std::byte{10}, std::byte{10}};
  std::array<unsigned char, 3> char_array = {10, 10, 10};
  dep_array.fill(std::byte{10});
  EXPECT_EQ(dep_array, byte_array);
  EXPECT_EQ(dep_array, char_array);

  dep_array.fill(35);
  byte_array.fill(std::byte{35});
  char_array.fill(35);

  EXPECT_EQ(dep_array, byte_array);
  EXPECT_EQ(dep_array, char_array);
}

TEST(rosidl_generator_cpp, deprecated_helper_array_tuple_element) {
  rosidl_runtime_cpp::DeprecatedHelperArray<3> dep_array;
  using T = std::tuple_element<0, decltype(dep_array)>::type;
  static_assert(std::is_same_v<T, unsigned char>);

  using B = std::tuple_element<0, std::array<std::byte, 10>>::type;
  static_assert(std::is_same_v<B, std::byte>);
}

#pragma GCC diagnostic pop
