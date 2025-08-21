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

#ifndef ROSIDL_RUNTIME_CPP__DEPRECATED_HELPER_VECTOR_HPP_
#define ROSIDL_RUNTIME_CPP__DEPRECATED_HELPER_VECTOR_HPP_

#include <cstdint>
#include <iostream>
#include <cstddef>
#include <memory>
#include <vector>


namespace rosidl_runtime_cpp
{
// using T = unsigned char;
// using T = std::byte;
// template<typename Allocator = std::allocator<T>>
// using DeprecatedHelperVector = std::vector<T, Allocator>;

template<typename Allocator = std::allocator<std::byte>>
struct DeprecatedHelperVector : std::vector<std::byte, Allocator>
{};
}  // namespace rosidl_runtime_cpp

#endif  // ROSIDL_RUNTIME_CPP__DEPRECATED_HELPER_VECTOR_HPP_
