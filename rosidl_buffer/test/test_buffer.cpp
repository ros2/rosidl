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

#include <gtest/gtest.h>

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <vector>

#include "rosidl_buffer/buffer.hpp"
#include "rosidl_buffer/buffer_impl_base.hpp"
#include "rosidl_buffer/cpu_buffer_impl.hpp"

using rosidl::Buffer;
using rosidl::BufferImplBase;
using rosidl::CpuBufferImpl;

// Test default construction
TEST(TestBuffer, default_construction) {
  Buffer<int> buffer;
  EXPECT_EQ(0u, buffer.size());
  EXPECT_TRUE(buffer.empty());
  EXPECT_EQ("cpu", buffer.get_backend_type());
}

// Test construction with size
TEST(TestBuffer, sized_construction) {
  Buffer<int> buffer(10);
  EXPECT_EQ(10u, buffer.size());
  EXPECT_FALSE(buffer.empty());
  EXPECT_EQ("cpu", buffer.get_backend_type());
}

// Test construction with size and value
TEST(TestBuffer, sized_value_construction) {
  Buffer<int> buffer(5, 42);
  EXPECT_EQ(5u, buffer.size());
  for (size_t i = 0; i < buffer.size(); ++i) {
    EXPECT_EQ(42, buffer[i]);
  }
}

// Test copy construction
TEST(TestBuffer, copy_construction) {
  Buffer<int> buffer1(3, 100);
  Buffer<int> buffer2(buffer1);

  EXPECT_EQ(buffer1.size(), buffer2.size());
  EXPECT_EQ(buffer1.get_backend_type(), buffer2.get_backend_type());
  for (size_t i = 0; i < buffer1.size(); ++i) {
    EXPECT_EQ(buffer1[i], buffer2[i]);
  }
}

// Test move construction
TEST(TestBuffer, move_construction) {
  Buffer<int> buffer1(3, 100);
  Buffer<int> buffer2(std::move(buffer1));

  EXPECT_EQ(3u, buffer2.size());
  EXPECT_EQ("cpu", buffer2.get_backend_type());
}

// Test copy assignment
TEST(TestBuffer, copy_assignment) {
  Buffer<int> buffer1(3, 100);
  Buffer<int> buffer2;
  buffer2 = buffer1;

  EXPECT_EQ(buffer1.size(), buffer2.size());
  for (size_t i = 0; i < buffer1.size(); ++i) {
    EXPECT_EQ(buffer1[i], buffer2[i]);
  }
}

// Test move assignment
TEST(TestBuffer, move_assignment) {
  Buffer<int> buffer1(3, 100);
  Buffer<int> buffer2;
  buffer2 = std::move(buffer1);

  EXPECT_EQ(3u, buffer2.size());
}

// Test element access via operator[]
TEST(TestBuffer, element_access) {
  Buffer<int> buffer(5);
  for (size_t i = 0; i < buffer.size(); ++i) {
    buffer[i] = static_cast<int>(i * 10);
  }

  for (size_t i = 0; i < buffer.size(); ++i) {
    EXPECT_EQ(static_cast<int>(i * 10), buffer[i]);
  }
}

// Test const element access
TEST(TestBuffer, const_element_access) {
  Buffer<int> buffer(3, 42);
  const Buffer<int> & const_buffer = buffer;

  EXPECT_EQ(42, const_buffer[0]);
  EXPECT_EQ(42, const_buffer[1]);
  EXPECT_EQ(42, const_buffer[2]);
}

// Test at() with bounds checking
TEST(TestBuffer, at_access) {
  Buffer<int> buffer(3, 99);
  EXPECT_EQ(99, buffer.at(0));
  EXPECT_EQ(99, buffer.at(2));
  EXPECT_THROW(buffer.at(3), std::out_of_range);
}

// Test front() and back()
TEST(TestBuffer, front_back_access) {
  Buffer<int> buffer;
  buffer.push_back(10);
  buffer.push_back(20);
  buffer.push_back(30);

  EXPECT_EQ(10, buffer.front());
  EXPECT_EQ(30, buffer.back());
}

// Test data() pointer access
TEST(TestBuffer, data_access) {
  Buffer<int> buffer(5, 7);
  int * ptr = buffer.data();
  ASSERT_NE(nullptr, ptr);
  EXPECT_EQ(7, ptr[0]);
  EXPECT_EQ(7, ptr[4]);

  ptr[2] = 100;
  EXPECT_EQ(100, buffer[2]);
}

// Test iterators
TEST(TestBuffer, iterators) {
  Buffer<int> buffer;
  buffer.push_back(1);
  buffer.push_back(2);
  buffer.push_back(3);

  int sum = 0;
  for (auto it = buffer.begin(); it != buffer.end(); ++it) {
    sum += *it;
  }
  EXPECT_EQ(6, sum);

  // Test const iterators
  const Buffer<int> & const_buffer = buffer;
  int const_sum = 0;
  for (auto it = const_buffer.begin(); it != const_buffer.end(); ++it) {
    const_sum += *it;
  }
  EXPECT_EQ(6, const_sum);
}

// Test resize
TEST(TestBuffer, resize) {
  Buffer<int> buffer(5, 10);
  EXPECT_EQ(5u, buffer.size());

  buffer.resize(10);
  EXPECT_EQ(10u, buffer.size());
  EXPECT_EQ(10, buffer[4]);  // Old values preserved

  buffer.resize(3);
  EXPECT_EQ(3u, buffer.size());
}

// Test resize with value
TEST(TestBuffer, resize_with_value) {
  Buffer<int> buffer(2, 5);
  buffer.resize(5, 99);

  EXPECT_EQ(5u, buffer.size());
  EXPECT_EQ(5, buffer[0]);
  EXPECT_EQ(5, buffer[1]);
  EXPECT_EQ(99, buffer[2]);
  EXPECT_EQ(99, buffer[4]);
}

// Test push_back
TEST(TestBuffer, push_back) {
  Buffer<int> buffer;
  EXPECT_TRUE(buffer.empty());

  buffer.push_back(10);
  EXPECT_EQ(1u, buffer.size());
  EXPECT_EQ(10, buffer[0]);

  buffer.push_back(20);
  EXPECT_EQ(2u, buffer.size());
  EXPECT_EQ(20, buffer[1]);
}

// Test push_back with rvalue
TEST(TestBuffer, push_back_rvalue) {
  Buffer<std::string> buffer;
  std::string str = "hello";
  buffer.push_back(std::move(str));

  EXPECT_EQ(1u, buffer.size());
  EXPECT_EQ("hello", buffer[0]);
}

// Test pop_back
TEST(TestBuffer, pop_back) {
  Buffer<int> buffer;
  buffer.push_back(1);
  buffer.push_back(2);
  buffer.push_back(3);

  EXPECT_EQ(3u, buffer.size());
  buffer.pop_back();
  EXPECT_EQ(2u, buffer.size());
  EXPECT_EQ(2, buffer.back());
}

// Test emplace_back
TEST(TestBuffer, emplace_back) {
  Buffer<std::string> buffer;
  buffer.emplace_back("test");
  buffer.emplace_back(5, 'x');  // Creates "xxxxx"

  EXPECT_EQ(2u, buffer.size());
  EXPECT_EQ("test", buffer[0]);
  EXPECT_EQ("xxxxx", buffer[1]);
}

// Test clear
TEST(TestBuffer, clear) {
  Buffer<int> buffer(10, 42);
  EXPECT_EQ(10u, buffer.size());
  EXPECT_FALSE(buffer.empty());

  buffer.clear();
  EXPECT_EQ(0u, buffer.size());
  EXPECT_TRUE(buffer.empty());
}

// Test reserve and capacity (CPU backend only)
TEST(TestBuffer, reserve_capacity) {
  Buffer<int> buffer;
  buffer.reserve(100);

  EXPECT_EQ(0u, buffer.size());
  EXPECT_GE(buffer.capacity(), 100u);
}

// Test shrink_to_fit
TEST(TestBuffer, shrink_to_fit) {
  Buffer<int> buffer;
  buffer.reserve(100);
  buffer.push_back(1);
  buffer.push_back(2);

  EXPECT_GE(buffer.capacity(), 100u);
  buffer.shrink_to_fit();
  EXPECT_EQ(2u, buffer.size());
  EXPECT_LE(buffer.capacity(), 100u);
}

// Test implicit conversion to std::vector&
TEST(TestBuffer, implicit_conversion_to_vector) {
  Buffer<int> buffer;
  buffer.push_back(1);
  buffer.push_back(2);
  buffer.push_back(3);

  // Implicit conversion
  std::vector<int> & vec_ref = buffer;
  EXPECT_EQ(3u, vec_ref.size());
  EXPECT_EQ(1, vec_ref[0]);
  EXPECT_EQ(2, vec_ref[1]);
  EXPECT_EQ(3, vec_ref[2]);

  // Modify through vector reference
  vec_ref[1] = 99;
  EXPECT_EQ(99, buffer[1]);
}

// Test const implicit conversion
TEST(TestBuffer, const_implicit_conversion) {
  Buffer<int> buffer(3, 42);
  const Buffer<int> & const_buffer = buffer;

  const std::vector<int> & vec_ref = const_buffer;
  EXPECT_EQ(3u, vec_ref.size());
  EXPECT_EQ(42, vec_ref[0]);
}

// Test to_vector() escape hatch
TEST(TestBuffer, to_vector_escape_hatch) {
  Buffer<int> buffer;
  buffer.push_back(10);
  buffer.push_back(20);
  buffer.push_back(30);

  std::vector<int> copied = buffer.to_vector();
  EXPECT_EQ(buffer.size(), copied.size());
  EXPECT_EQ(10, copied[0]);
  EXPECT_EQ(20, copied[1]);
  EXPECT_EQ(30, copied[2]);

  // Verify it's a copy
  copied[0] = 999;
  EXPECT_EQ(10, buffer[0]);  // Original unchanged
}

// Test backend type
TEST(TestBuffer, backend_type) {
  Buffer<int> buffer;
  EXPECT_EQ("cpu", buffer.get_backend_type());
}

// Test using Buffer with algorithms
TEST(TestBuffer, with_algorithms) {
  Buffer<int> buffer;
  for (int i = 0; i < 10; ++i) {
    buffer.push_back(i);
  }

  // Test std::find
  auto it = std::find(buffer.begin(), buffer.end(), 5);
  ASSERT_NE(buffer.end(), it);
  EXPECT_EQ(5, *it);

  // Test std::count
  buffer.push_back(5);
  int count = std::count(buffer.begin(), buffer.end(), 5);
  EXPECT_EQ(2, count);

  // Test std::sort
  Buffer<int> buffer2;
  buffer2.push_back(3);
  buffer2.push_back(1);
  buffer2.push_back(2);
  std::sort(buffer2.begin(), buffer2.end());
  EXPECT_EQ(1, buffer2[0]);
  EXPECT_EQ(2, buffer2[1]);
  EXPECT_EQ(3, buffer2[2]);
}

// Test CpuBufferImpl directly
TEST(TestCpuBufferImpl, basic_operations) {
  CpuBufferImpl<int> impl;
  EXPECT_EQ(0u, impl.get_storage().size());

  impl.get_storage().resize(5);
  EXPECT_EQ(5u, impl.get_storage().size());

  auto & storage = impl.get_storage();
  storage[0] = 100;
  EXPECT_EQ(100, storage[0]);

  impl.get_storage().clear();
  EXPECT_EQ(0u, impl.get_storage().size());
}

// Test CpuBufferImpl::to_cpu()
TEST(TestCpuBufferImpl, to_cpu) {
  CpuBufferImpl<int> impl;
  impl.get_storage().resize(3);
  auto & storage = impl.get_storage();
  storage[0] = 1;
  storage[1] = 2;
  storage[2] = 3;

  auto cpu_copy = impl.to_cpu();
  ASSERT_NE(nullptr, cpu_copy);

  auto * cpu_impl = static_cast<CpuBufferImpl<int> *>(cpu_copy.get());
  EXPECT_EQ(3u, cpu_impl->get_storage().size());
  EXPECT_EQ(1, cpu_impl->get_storage()[0]);
  EXPECT_EQ(2, cpu_impl->get_storage()[1]);
  EXPECT_EQ(3, cpu_impl->get_storage()[2]);
}

// Test CpuBufferImpl storage data pointer
TEST(TestCpuBufferImpl, storage_data_pointer) {
  CpuBufferImpl<int> impl;

  EXPECT_TRUE(impl.get_storage().empty());

  impl.get_storage().resize(5);
  const int * data_ptr = impl.get_storage().data();
  ASSERT_NE(nullptr, data_ptr);

  impl.get_storage()[0] = 42;
  EXPECT_EQ(42, data_ptr[0]);
}

// Test Buffer copy creates independent clone (deep copy via unique_ptr)
TEST(TestBuffer, deep_copy_semantics) {
  Buffer<int> buffer1;
  buffer1.push_back(10);
  buffer1.push_back(20);

  Buffer<int> buffer2 = buffer1;  // Deep copy via clone()

  // Verify they have the same values
  EXPECT_EQ(buffer1.size(), buffer2.size());
  EXPECT_EQ(buffer1[0], buffer2[0]);
  EXPECT_EQ(buffer1[1], buffer2[1]);

  // Modify buffer2 and verify buffer1 is unchanged (independent copies)
  buffer2[0] = 999;
  EXPECT_EQ(10, buffer1[0]);  // Original unchanged
  EXPECT_EQ(999, buffer2[0]);  // Copy modified
}

// Test Buffer::set_impl() for backend injection
TEST(TestBuffer, set_impl) {
  Buffer<int> buffer;

  // Create a custom impl
  auto custom_impl = std::make_unique<CpuBufferImpl<int>>();
  custom_impl->get_storage().resize(3);
  custom_impl->get_storage()[0] = 100;
  custom_impl->get_storage()[1] = 200;
  custom_impl->get_storage()[2] = 300;

  // Inject the impl
  buffer.set_impl(std::move(custom_impl), "cpu");

  EXPECT_EQ(3u, buffer.size());
  EXPECT_EQ(100, buffer[0]);
  EXPECT_EQ(200, buffer[1]);
  EXPECT_EQ(300, buffer[2]);
}

// Test that Buffer works with complex types
TEST(TestBuffer, with_string) {
  Buffer<std::string> buffer;
  buffer.push_back("hello");
  buffer.push_back("world");

  EXPECT_EQ(2u, buffer.size());
  EXPECT_EQ("hello", buffer[0]);
  EXPECT_EQ("world", buffer[1]);

  std::vector<std::string> copied = buffer.to_vector();
  EXPECT_EQ("hello", copied[0]);
  EXPECT_EQ("world", copied[1]);
}

// Test Buffer with struct types
TEST(TestBuffer, with_struct)
{
  struct Point
  {
    double x;
    double y;
    double z;
  };

  Buffer<Point> buffer;
  buffer.push_back({1.0, 2.0, 3.0});
  buffer.push_back({4.0, 5.0, 6.0});

  EXPECT_EQ(2u, buffer.size());
  EXPECT_DOUBLE_EQ(1.0, buffer[0].x);
  EXPECT_DOUBLE_EQ(5.0, buffer[1].y);

  std::vector<Point> copied = buffer.to_vector();
  EXPECT_DOUBLE_EQ(3.0, copied[0].z);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
