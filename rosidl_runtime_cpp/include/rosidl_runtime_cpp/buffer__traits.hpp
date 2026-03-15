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

#ifndef ROSIDL_RUNTIME_CPP__BUFFER__TRAITS_HPP_
#define ROSIDL_RUNTIME_CPP__BUFFER__TRAITS_HPP_

#include <type_traits>

#include "rosidl_buffer/buffer.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace rosidl_generator_traits
{

// Trait specializations for rosidl::Buffer<T>
// Buffer is not a message itself
template<typename T, typename Allocator>
struct is_message<rosidl::Buffer<T, Allocator>>
  : std::false_type {};

// Buffer has dynamic size
template<typename T, typename Allocator>
struct has_fixed_size<rosidl::Buffer<T, Allocator>>
  : std::false_type {};

// Buffer is unbounded
template<typename T, typename Allocator>
struct has_bounded_size<rosidl::Buffer<T, Allocator>>
  : std::false_type {};

}  // namespace rosidl_generator_traits

#endif  // ROSIDL_RUNTIME_CPP__BUFFER__TRAITS_HPP_
