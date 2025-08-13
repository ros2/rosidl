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
#include <algorithm>
#include <array>
#include <iterator>
#include <cstddef>
#include <cstring>
#include <utility>

#include "byte_ref.hpp"
#include "deprecated_helper_byte.hpp"

// // For testing
// namespace rosidl_runtime_cpp
// {
// // using T = unsigned char;
// using T = std::byte;
// template<std::size_t N>
// using DeprecatedHelperArray = std::array<std::byte, N>;

// }  // namespace rosidl_runtime_cpp

namespace rosidl_runtime_cpp
{

template<std::size_t N>
struct DeprecatedHelperArray : std::array<std::byte, N>
{
  // Override Defaults to match old types
  using value_type = unsigned char;
  using reference = value_type &;
  using const_reference = const reference;
  using pointer = value_type *;
  using const_pointer = const pointer;
  using iterator = pointer;
  using const_iterator = const_pointer;
  using reverse_iterator = std::reverse_iterator<iterator>;
  using reverse_const_iterator = std::reverse_iterator<const_iterator>;

  // Helper Types
  using ByteArray = std::array<std::byte, N>;
  using UCharArray = std::array<unsigned char, N>;


  // Member Functions
  // Constructor
  [[deprecated("Use std::array<std::byte, N> instead of unsigned char array")]]
  constexpr DeprecatedHelperArray(const UCharArray & arr) noexcept  // NOLINT(runtime/explicit)
  {
    for (std::size_t i = 0; i < N; ++i) {
      ByteArray::operator[](i) = std::byte{arr[i]};
    }
  }

  [[deprecated("Use initializer_list of bytes instead of unsigned chars")]]
  constexpr DeprecatedHelperArray(
    std::initializer_list<unsigned char> init)  // NOLINT(runtime/explicit)
  {
    std::transform(init.begin(), init.end(), ByteArray::begin(),
      [](unsigned char v) {return std::byte{v};});
  }

  constexpr DeprecatedHelperArray(
    std::initializer_list<std::byte> init)  // NOLINT(runtime/explicit)
  {
    std::copy(init.begin(), init.end(), ByteArray::begin());
  }

  // Assignment
  [[deprecated("Use std::array<std::byte, N> instead")]]
  constexpr UCharArray & operator=(const UCharArray & arr) noexcept
  {
    for (std::size_t i = 0; i < N; ++i) {
      ByteArray::operator[](i) = std::byte{arr[i]};
    }
    return to_uchar_array_ref(*this);
  }

  [[deprecated("Use initializer_list<std::byte> instead of unsigned chars")]]
  constexpr DeprecatedHelperArray & operator=(std::initializer_list<unsigned char> init) noexcept
  {
    std::transform(init.begin(), init.end(), ByteArray::begin(),
      [](unsigned char v) {return std::byte{v};});
    return *this;
  }

  constexpr DeprecatedHelperArray & operator=(std::initializer_list<std::byte> init) noexcept
  {
    std::copy(init.begin(), init.end(), ByteArray::begin());
    return *this;
  }


  // Implicitly-defined Member Functions

  // Element Access

  constexpr ByteRef at(std::size_t pos)
  {
    return ByteRef{ByteArray::at(pos)};
  }

  constexpr DeprecatedHelperByte at(std::size_t pos) const
  {
    return DeprecatedHelperByte{ByteArray::at(pos)};
  }

  constexpr ByteRef operator[](std::size_t pos)
  {
    return ByteRef{ByteArray::operator[](pos)};
  }

  constexpr DeprecatedHelperByte operator[](std::size_t pos) const
  {
    return DeprecatedHelperByte{ByteArray::operator[](pos)};
  }

  // Iterators

  // begin, cbegin
  constexpr DeprecatedHelperByte * begin() noexcept
  {
    return reinterpret_cast<DeprecatedHelperByte *>(ByteArray::begin());
  }

  constexpr const DeprecatedHelperByte * begin() const noexcept
  {
    return reinterpret_cast<const DeprecatedHelperByte *>(ByteArray::begin());
  }

  constexpr const DeprecatedHelperByte * cbegin() const noexcept
  {
    return reinterpret_cast<const DeprecatedHelperByte *>(ByteArray::cbegin());
  }

  // end, cend
  constexpr DeprecatedHelperByte * end() noexcept
  {
    return reinterpret_cast<DeprecatedHelperByte *>(ByteArray::end());
  }

  constexpr const DeprecatedHelperByte * end() const noexcept
  {
    return reinterpret_cast<const DeprecatedHelperByte *>(ByteArray::end());
  }

  constexpr const DeprecatedHelperByte * cend() const noexcept
  {
    return reinterpret_cast<const DeprecatedHelperByte *>(ByteArray::cend());
  }

  // Operations

  [[deprecated("Switch to calling fill with std::byte")]]
  constexpr void fill(const unsigned char & value)
  {
    ByteArray::fill(std::byte{value});
  }

  [[deprecated("Switch to calling fill with std::array<std::byte, N>&")]]
  constexpr void swap(UCharArray & other)
  {
    ByteArray::swap(to_byte_array_ref(other));
  }

  // Non-member Functions

  // Equals

  // Implicitly-defined member functions
  using ByteArray::ByteArray;
  using ByteArray::operator=;

  // Element Access
  using ByteArray::at;
  using ByteArray::operator[];
  using ByteArray::front;
  using ByteArray::back;
  using ByteArray::data;

  // Iterators
  using ByteArray::begin;
  using ByteArray::cbegin;
  using ByteArray::end;
  using ByteArray::cend;
  using ByteArray::rbegin;
  using ByteArray::crbegin;
  using ByteArray::rend;
  using ByteArray::crend;

  // Capacity
  using ByteArray::empty;
  using ByteArray::size;
  using ByteArray::max_size;

  // Operations
  using ByteArray::fill;
  using ByteArray::swap;

private:
  // Helper Functions
  constexpr UCharArray & to_uchar_array_ref(ByteArray & in)
  {
    return reinterpret_cast<UCharArray &>(in);
  }
  constexpr ByteArray & to_byte_array_ref(UCharArray & in)
  {
    return reinterpret_cast<ByteArray &>(in);
  }
};

template<std::size_t N>
constexpr bool operator==(
  const std::array<std::byte, N> & lhs,
  const std::array<unsigned char, N> & rhs)
{
  for (std::size_t i = 0; i < N; ++i) {
    if (std::byte{rhs[i]} != lhs[i]) {
      return false;
    }
  }
  return true;
}

template<std::size_t N>
constexpr bool operator==(
  const std::array<unsigned char, N> & lhs,
  const std::array<std::byte, N> & rhs)
{
  return operator==(rhs, lhs);
}

}  // namespace rosidl_runtime_cpp

namespace std
{
template<std::size_t I, std::size_t N>
struct tuple_element<I, rosidl_runtime_cpp::DeprecatedHelperArray<N>>
{
  static_assert(I < N, "Index out of range for DeprecatedHelperArray");
  using type = unsigned char;
};
}  // namespace std

#endif  // ROSIDL_RUNTIME_CPP__DEPRECATED_HELPER_ARRAY_HPP_
