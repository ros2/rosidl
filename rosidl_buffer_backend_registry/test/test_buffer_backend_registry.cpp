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

#include <memory>
#include <vector>

#include "rosidl_buffer_backend_registry/buffer_backend_registry.hpp"
#include "dummy_buffer_backend.hpp"

using rosidl_buffer_backend_registry::BufferBackendRegistry;
using rosidl_buffer_backend_registry::test::DummyBufferBackend;
using rosidl_buffer_backend_registry::test::DummyBufferImpl;

// Test getting singleton instance
TEST(TestBufferBackendRegistry, get_instance) {
  auto & registry1 = BufferBackendRegistry::get_instance();
  auto & registry2 = BufferBackendRegistry::get_instance();

  // Should be the same instance
  EXPECT_EQ(&registry1, &registry2);
}

// Test registering and getting a backend
TEST(TestBufferBackendRegistry, register_and_get_backend) {
  auto & registry = BufferBackendRegistry::get_instance();

  auto dummy_backend = std::make_shared<DummyBufferBackend>();
  registry.register_backend("dummy_test", dummy_backend);

  auto retrieved = registry.get_backend("dummy_test");
  ASSERT_NE(nullptr, retrieved);
  EXPECT_EQ(dummy_backend.get(), retrieved.get());
}

// Test getting non-existent backend
TEST(TestBufferBackendRegistry, get_nonexistent_backend) {
  auto & registry = BufferBackendRegistry::get_instance();

  auto backend = registry.get_backend("nonexistent_backend_12345");
  EXPECT_EQ(nullptr, backend);
}

// Test DummyBufferImpl to_cpu() conversion
TEST(TestDummyBufferImpl, to_cpu_conversion) {
  DummyBufferImpl<int> dummy(4);
  dummy.get_data()[0] = 10;
  dummy.get_data()[1] = 20;
  dummy.get_data()[2] = 30;
  dummy.get_data()[3] = 40;

  auto cpu_impl = dummy.to_cpu();
  ASSERT_NE(nullptr, cpu_impl);

  auto * cpu = static_cast<rosidl::CpuBufferImpl<int> *>(cpu_impl.get());
  EXPECT_EQ(4u, cpu->get_storage().size());
  EXPECT_EQ(10, cpu->get_storage()[0]);
  EXPECT_EQ(20, cpu->get_storage()[1]);
  EXPECT_EQ(30, cpu->get_storage()[2]);
  EXPECT_EQ(40, cpu->get_storage()[3]);
}

// Test marker preservation
TEST(TestDummyBufferImpl, marker_verification) {
  DummyBufferImpl<uint8_t> impl1;
  DummyBufferImpl<uint8_t> impl2;

  EXPECT_EQ(0xDEADBEEFu, impl1.get_marker());
  EXPECT_EQ(0xDEADBEEFu, impl2.get_marker());
}

// Test data access via get_data()
TEST(TestDummyBufferImpl, data_access) {
  DummyBufferImpl<int> impl;

  EXPECT_TRUE(impl.get_data().empty());

  impl.get_data().resize(3);
  impl.get_data()[0] = 100;

  const int * ptr = impl.get_data().data();
  ASSERT_NE(nullptr, ptr);
  EXPECT_EQ(100, ptr[0]);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
