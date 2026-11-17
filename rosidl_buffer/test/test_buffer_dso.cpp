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

// Buffer must recognize a CpuBufferImpl created in a shared library loaded with
// RTLD_LOCAL. With libc++, dynamic_cast across such a boundary fails because each
// side has its own CpuBufferImpl RTTI and type_info is compared by address.

#include <dlfcn.h>
#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

#include "rosidl_buffer/buffer.hpp"
#include "rosidl_buffer/buffer_impl_base.hpp"
#include "rosidl_buffer/cpu_buffer_impl.hpp"

using rosidl::Buffer;
using rosidl::BufferImplBase;
using rosidl::CpuBufferImpl;

class TestBufferDso : public ::testing::Test
{
protected:
  using MakeFn = BufferImplBase<uint8_t> * (*)();
  using PushBackFn = int64_t (*)(BufferImplBase<uint8_t> *);

  void SetUp() override
  {
    handle_ = dlopen(BUFFER_DSO_PLUGIN_PATH, RTLD_NOW | RTLD_LOCAL);
    ASSERT_NE(nullptr, handle_) << dlerror();
    make_ = reinterpret_cast<MakeFn>(dlsym(handle_, "buffer_dso_plugin_make_cpu_impl"));
    ASSERT_NE(nullptr, make_) << dlerror();
    push_back_on_ = reinterpret_cast<PushBackFn>(dlsym(handle_, "buffer_dso_plugin_push_back_on"));
    ASSERT_NE(nullptr, push_back_on_) << dlerror();
  }

  void TearDown() override
  {
    if (handle_) {
      dlclose(handle_);
    }
  }

  Buffer<uint8_t> make_plugin_buffer()
  {
    return Buffer<uint8_t>(std::unique_ptr<BufferImplBase<uint8_t>>(make_()));
  }

  void * handle_ = nullptr;
  MakeFn make_ = nullptr;
  PushBackFn push_back_on_ = nullptr;
};

TEST_F(TestBufferDso, plugin_cpu_impl_is_cpu)
{
  Buffer<uint8_t> buffer = make_plugin_buffer();
  ASSERT_NO_THROW(buffer.push_back(1));
  EXPECT_EQ(5u, buffer.size());
  EXPECT_EQ(9, buffer[0]);
  EXPECT_EQ(1, buffer[4]);
  std::vector<uint8_t> & vec = buffer;
  EXPECT_EQ(5u, vec.size());
}

TEST_F(TestBufferDso, plugin_cpu_impl_stays_cpu_after_move)
{
  Buffer<uint8_t> buffer = make_plugin_buffer();
  Buffer<uint8_t> moved(std::move(buffer));
  EXPECT_NO_THROW(moved.push_back(1));
  Buffer<uint8_t> assigned;
  assigned = std::move(moved);
  EXPECT_NO_THROW(assigned.push_back(2));
  EXPECT_EQ(6u, assigned.size());
}

TEST_F(TestBufferDso, plugin_cpu_impl_stays_cpu_after_copy)
{
  // clone() runs in the plugin, so the copy's impl also comes from the plugin.
  Buffer<uint8_t> buffer = make_plugin_buffer();
  Buffer<uint8_t> copied(buffer);
  EXPECT_NO_THROW(copied.push_back(1));
  Buffer<uint8_t> assigned;
  assigned = buffer;
  EXPECT_NO_THROW(assigned.push_back(2));
  EXPECT_EQ(buffer, Buffer<uint8_t>(std::vector<uint8_t>(4, 9)));
}

TEST_F(TestBufferDso, host_cpu_impl_is_cpu_in_plugin)
{
  auto impl = std::make_unique<CpuBufferImpl<uint8_t>>();
  impl->get_storage().assign(2, 3);
  EXPECT_EQ(3, push_back_on_(impl.release()));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
