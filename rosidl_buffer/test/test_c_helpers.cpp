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

#include <cstdint>
#include <cstring>
#include <memory>
#include <stdexcept>

#include "rosidl_buffer/buffer.hpp"
#include "rosidl_buffer/c_helpers.h"

#include "non_cpu_buffer_impl.hpp"

using rosidl::Buffer;

// -- rosidl_buffer_uint8_throw_if_not_cpu --

TEST(TestCHelpers, throw_if_not_cpu_does_not_throw_for_cpu) {
  Buffer<uint8_t> buf(5, 1);
  EXPECT_NO_THROW(rosidl_buffer_uint8_throw_if_not_cpu(&buf));
}

TEST(TestCHelpers, throw_if_not_cpu_does_not_throw_for_empty_cpu) {
  Buffer<uint8_t> buf;
  EXPECT_NO_THROW(rosidl_buffer_uint8_throw_if_not_cpu(&buf));
}

TEST(TestCHelpers, throw_if_not_cpu_throws_for_non_cpu_backend) {
  auto impl = std::make_unique<NonCpuBufferImpl<uint8_t>>(4);
  Buffer<uint8_t> buf(std::move(impl));
  EXPECT_THROW(rosidl_buffer_uint8_throw_if_not_cpu(&buf), std::runtime_error);
}

// -- rosidl_buffer_uint8_destroy --

TEST(TestCHelpers, destroy_null_is_noop) {
  EXPECT_NO_THROW(rosidl_buffer_uint8_destroy(nullptr));
}

TEST(TestCHelpers, destroy_cpu_buffer) {
  auto * buf = new Buffer<uint8_t>({1, 2, 3, 4, 5});
  EXPECT_EQ(buf->size(), 5u);
  rosidl_buffer_uint8_destroy(buf);
}

TEST(TestCHelpers, destroy_non_cpu_buffer) {
  auto * buf = new Buffer<uint8_t>(std::make_unique<NonCpuBufferImpl<uint8_t>>(10));
  EXPECT_EQ(buf->get_backend_type(), "non_cpu_test");
  rosidl_buffer_uint8_destroy(buf);
}

TEST(TestCHelpers, create_clone_and_query_cpu_buffer) {
  const uint8_t source[] = {1, 2, 3, 4};
  void * buffer = nullptr;
  ASSERT_EQ(
    ROSIDL_BUFFER_RET_OK,
    rosidl_buffer_uint8_create_cpu(source, sizeof(source), &buffer));
  ASSERT_NE(buffer, nullptr);

  size_t size = 0;
  EXPECT_EQ(ROSIDL_BUFFER_RET_OK, rosidl_buffer_uint8_size(buffer, &size));
  EXPECT_EQ(sizeof(source), size);

  size_t required_size = 0;
  EXPECT_EQ(
    ROSIDL_BUFFER_RET_OK,
    rosidl_buffer_uint8_backend_name(buffer, nullptr, 0, &required_size));
  std::string backend(required_size, '\0');
  EXPECT_EQ(
    ROSIDL_BUFFER_RET_OK,
    rosidl_buffer_uint8_backend_name(
      buffer, backend.data(), backend.size(), &required_size));
  EXPECT_STREQ("cpu", backend.c_str());

  uint8_t copied[sizeof(source)] = {};
  EXPECT_EQ(
    ROSIDL_BUFFER_RET_OK,
    rosidl_buffer_uint8_copy_to(buffer, copied, sizeof(copied)));
  EXPECT_EQ(0, std::memcmp(source, copied, sizeof(source)));

  void * clone = nullptr;
  ASSERT_EQ(ROSIDL_BUFFER_RET_OK, rosidl_buffer_uint8_clone(buffer, &clone));
  bool are_equal = false;
  EXPECT_EQ(
    ROSIDL_BUFFER_RET_OK,
    rosidl_buffer_uint8_are_equal(buffer, clone, &are_equal));
  EXPECT_TRUE(are_equal);

  rosidl_buffer_uint8_destroy(clone);
  rosidl_buffer_uint8_destroy(buffer);
}

TEST(TestCHelpers, rejects_invalid_arguments) {
  EXPECT_EQ(
    ROSIDL_BUFFER_RET_INVALID_ARGUMENT,
    rosidl_buffer_uint8_create_cpu(nullptr, 1, nullptr));
  EXPECT_EQ(
    ROSIDL_BUFFER_RET_INVALID_ARGUMENT,
    rosidl_buffer_uint8_clone(nullptr, nullptr));
  EXPECT_EQ(
    ROSIDL_BUFFER_RET_INVALID_ARGUMENT,
    rosidl_buffer_uint8_size(nullptr, nullptr));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
