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
#include <vector>


namespace rosidl_runtime_cpp
{

  template <typename Allocator = std::allocator<unsigned char>>
  using DeprecatedHelperVector = std::vector<unsigned char, Allocator>;

// struct DeprecatedHelperVector
// {
//   // Constructor
//   constexpr DeprecatedHelperVector() noexcept = default;

//   [[deprecated("Initialize with std::byte instead")]]
//   constexpr DeprecatedHelperVector(unsigned char value) noexcept  // NOLINT(runtime/explicit)
//   : b(value) {}

//   constexpr DeprecatedHelperVector(std::byte value) noexcept  // NOLINT(runtime/explicit)
//   : b(std::to_integer<unsigned char>(value)) {}

//   // Getter
//   [[deprecated("Use static_cast<std::byte&> instead")]]
//   constexpr operator unsigned char&() noexcept
//   {
//     return b;
//   }

//   // Used for comparisons with const int
//   [[deprecated("Use static_cast<std::byte> instead")]]
//   constexpr operator unsigned char() const noexcept
//   {
//     return b;
//   }

//   // To avoid collisions with unsigned char 
//   // Explicit needing a static_cast<> until unsigned char is removed.
//   explicit constexpr operator std::byte&() noexcept
//   {
//     return reinterpret_cast<std::byte&>(b);
//   }

//   explicit constexpr operator std::byte() const noexcept
//   {
//     return std::byte{b};
//   }

//   // Setter
//   [[deprecated("Set std::byte instead")]]
//   constexpr DeprecatedHelperVector & operator=(unsigned char value) noexcept
//   {
//     b = std::move(value);
//     return *this;
//   }

//   constexpr DeprecatedHelperVector & operator=(std::byte value) noexcept
//   {
//     b = std::to_integer<unsigned char>(value);
//     return *this;
//   }


// private:
//   unsigned char b{};
// };
}  // namespace rosidl_runtime_cpp

#endif  // ROSIDL_RUNTIME_CPP__DEPRECATED_HELPER_VECTOR_HPP_
