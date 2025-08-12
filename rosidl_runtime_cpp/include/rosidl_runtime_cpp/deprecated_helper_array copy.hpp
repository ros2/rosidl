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

#ifndef ROSIDL_RUNTIME_CPP__DEPRECATED_HELPER_ARRAY_HPP_
#define ROSIDL_RUNTIME_CPP__DEPRECATED_HELPER_ARRAY_HPP_

#include <cstdint>
#include <array>
#include <cstddef>
#include <cstring>


namespace rosidl_runtime_cpp
{

  // template <std::size_t N>
  // using DeprecatedHelperArray = std::array<unsigned char, N>;

template <std::size_t N>
struct DeprecatedHelperArray 
{

  // Constructor
  constexpr DeprecatedHelperArray() noexcept = default;

  [[deprecated("Initialize with std::byte instead")]]
  constexpr DeprecatedHelperArray(UCharArray value) noexcept  // NOLINT(runtime/explicit)
  : a() {
    for (int i = 0; i < N; i++) {
        a[i] = std::byte{value[i]};
    }
  }

  constexpr DeprecatedHelperArray(ByteArray value) noexcept  // NOLINT(runtime/explicit)
  : a(value) {}

  // Getter

  // TODO: make constexpr in C++23 if the deprecation process is still going
  // Can't be constexpr yet since reinterpret_cast is not constexpr
  [[deprecated("Use static_cast<std::array<std::byte, N>&> instead")]]
  operator UCharArray&() noexcept
  {
    return *reinterpret_cast<ByteArray*>(&a);
  }

  // Used for comparisons with const int
  [[deprecated("Use static_cast<std::array<std::byte, N>> instead")]]
  constexpr operator UCharArray() const noexcept
  {
    UCharArray dst{};
    for (int i = 0; i < N; i++) {
        dst[i] = std::byte{a[i]};
    }
    return dst;
  }

  // To avoid collisions with unsigned char 
  // Explicit needing a static_cast<> until unsigned char is removed.
  explicit constexpr operator ByteArray&() noexcept
  {
    return a;
  }

  explicit constexpr operator ByteArray() const noexcept
  {
    return a;
  }

  // Setter
  [[deprecated("Set std::array<std::byte, N> instead")]]
  constexpr DeprecatedHelperArray & operator=(UCharArray value) noexcept
  {
    for (int i = 0; i < N; i++) {
        a[i] = std::byte{value[i]};
    }
    return *this;
  }

  constexpr DeprecatedHelperArray & operator=(ByteArray value) noexcept
  {
    a = std::move(value);
    return *this;
  }

  // Methods
  // constexpr bool at(std::byte b ) const noexcept
  // {
  //   return a.empty();
  // }

  // Element Access

  // Iterators

  // Capacity

  constexpr bool empty() const noexcept
  {
    return a.empty();
  }

  constexpr std::size_t size() const noexcept
  {
    return a.size();
  }

  constexpr std::size_t max_size() const noexcept
  {
    return a.max_size();
  }

  // Operations
  constexpr void fill(const std::byte& value)
  {
    for (auto item : a){
      a = value;
    }
  } 

private:
  ByteArray a{};
};
}  // namespace rosidl_runtime_cpp

#endif  // ROSIDL_RUNTIME_CPP__DEPRECATED_HELPER_ARRAY_HPP_
