// Copyright 2014-2015 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
// // Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROSIDL_GENERATOR_CPP__TEST__NULL_ALLOCATOR__HPP
#define ROSIDL_GENERATOR_CPP__TEST__NULL_ALLOCATOR__HPP

#include <memory>

namespace rosidl_generator_cpp {
namespace test {

template<typename T>
struct null_allocator {
  using value_type = T;

  null_allocator() {}

  template<typename U>
  null_allocator(const null_allocator<U>&) {}

  T* allocate(size_t size) {
    (void) size;
    return NULL;
  }

  void deallocate(T* ptr, size_t size) {
    (void) ptr;
    (void) size;
  }
};

template<typename T, typename U>
constexpr bool operator== (const null_allocator<T>&, const null_allocator<U>&) noexcept {
  return true;
}

template<typename T, typename U>
constexpr bool operator!= (const null_allocator<T>&, const null_allocator<U>&) noexcept {
  return false;
}

}
}

#endif  // ROSIDL_GENERATOR_CPP__TEST__NULL_ALLOCATOR__HPP
