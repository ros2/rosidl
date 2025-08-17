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

#include "byte_conversion_helpers.hpp"
#include "deprecated_helper_byte.hpp"

// // For testing
// namespace rosidl_runtime_cpp
// {
// // using T = unsigned char;
// using T = std::byte;
// template<std::size_t N>
// using DeprecatedHelperArray = std::array<std::byte, N>;

// }  // namespace rosidl_runtime_cpp


// Anonymous namespace for private Helpers
namespace
{
template<std::size_t N>
constexpr std::array<unsigned  char, N> & to_uchar_array_ref(std::array<std::byte, N> & in)
{
  return reinterpret_cast<std::array<unsigned char, N> &>(in);
}

template<std::size_t N>
constexpr std::array<std::byte, N> & to_byte_array_ref(std::array<unsigned  char, N> & in)
{
  return reinterpret_cast<std::array<std::byte, N> &>(in);
}

template<std::size_t N>
constexpr std::array<std::byte, N> to_byte_array(const std::array<unsigned char, N> & arr)
{
  std::array<std::byte, N> byte_array{};

  for (std::size_t i = 0; i < N; i++) {
    byte_array[i] = std::byte{arr[i]};
  }
  return byte_array;
}
}  // namespace


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

  // Getter

  [[deprecated("Convert to std::array<std::byte, N> instead")]]
  constexpr operator UCharArray &() noexcept
  {
    return to_uchar_array_ref(*this);
  }

  // Implicitly-defined Member Functions

  // Element Access

  // at
  constexpr detail::ByteRef at(std::size_t pos)
  {
    return detail::ByteRef{ByteArray::at(pos)};
  }

  constexpr DeprecatedHelperByte at(std::size_t pos) const
  {
    return DeprecatedHelperByte{ByteArray::at(pos)};
  }

  // operator[]
  constexpr detail::ByteRef operator[](std::size_t pos)
  {
    return detail::ByteRef{ByteArray::operator[](pos)};
  }

  constexpr DeprecatedHelperByte operator[](std::size_t pos) const
  {
    return DeprecatedHelperByte{ByteArray::operator[](pos)};
  }

  // front
  constexpr detail::ByteRef front()
  {
    return detail::ByteRef{ByteArray::front()};
  }

  constexpr DeprecatedHelperByte front() const
  {
    return DeprecatedHelperByte{ByteArray::front()};
  }

  // back
  constexpr detail::ByteRef back()
  {
    return detail::ByteRef{ByteArray::back()};
  }

  constexpr DeprecatedHelperByte back() const
  {
    return DeprecatedHelperByte{ByteArray::back()};
  }

  // data()
  constexpr detail::BytePtr data()
  {
    return detail::BytePtr{ByteArray::data()};
  }

  constexpr detail::ConstBytePtr data() const
  {
    return detail::ConstBytePtr{ByteArray::data()};
  }

  // Iterators

  // begin, cbegin
  constexpr detail::BytePtr begin() noexcept
  {
    return detail::BytePtr{ByteArray::begin()};
  }

  constexpr detail::ConstBytePtr begin() const noexcept
  {
    return detail::ConstBytePtr{ByteArray::begin()};
  }

  constexpr detail::ConstBytePtr cbegin() const noexcept
  {
    return detail::ConstBytePtr{ByteArray::cbegin()};
  }

  // end, cend
  constexpr detail::BytePtr end() noexcept
  {
    return detail::BytePtr{ByteArray::end()};
  }

  constexpr detail::ConstBytePtr end() const noexcept
  {
    return detail::ConstBytePtr{ByteArray::end()};
  }

  constexpr detail::ConstBytePtr cend() const noexcept
  {
    return detail::ConstBytePtr{ByteArray::cend()};
  }

  // rbegin, crbegin
  constexpr detail::BytePtr rbegin() noexcept
  {
    return detail::BytePtr{ByteArray::rbegin()};
  }

  constexpr detail::ConstBytePtr rbegin() const noexcept
  {
    return detail::ConstBytePtr{ByteArray::rbegin()};
  }

  constexpr detail::ConstBytePtr crbegin() const noexcept
  {
    return detail::ConstBytePtr{ByteArray::crbegin()};
  }

  // rend, crend
  constexpr detail::BytePtr rend() noexcept
  {
    return detail::BytePtr{ByteArray::rend()};
  }

  constexpr detail::ConstBytePtr rend() const noexcept
  {
    return detail::ConstBytePtr{ByteArray::rend()};
  }

  constexpr detail::ConstBytePtr crend() const noexcept
  {
    return detail::ConstBytePtr{ByteArray::crend()};
  }

  // Operations

  [[deprecated("Switch to calling fill with std::byte")]]
  constexpr void fill(const unsigned char & value)
  {
    ByteArray::fill(std::byte{value});
  }

  [[deprecated("Switch to calling swap with std::array<std::byte, N>&")]]
  constexpr void swap(UCharArray & other)
  {
    ByteArray::swap(to_byte_array_ref(other));
  }

  // Non-member Functions

  // Comparisons defined below

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
};

// ==
template<std::size_t N>
[[deprecated("Do comparisons with std::array<std::byte, N>")]]
constexpr bool operator==(
  const std::array<unsigned char, N> & lhs,
  const DeprecatedHelperArray<N> & rhs)
{
  return to_byte_array(lhs) == rhs;
}

template<std::size_t N>
[[deprecated("Do comparisons with std::array<std::byte, N>")]]
constexpr bool operator==(
  const DeprecatedHelperArray<N> & lhs,
  const std::array<unsigned char, N> & rhs)
{
  return lhs == to_byte_array(rhs);
}

// !=
template<std::size_t N>
[[deprecated("Do comparisons with std::array<std::byte, N>")]]
constexpr bool operator!=(
  const std::array<unsigned char, N> & lhs,
  const DeprecatedHelperArray<N> & rhs)
{
  return to_byte_array(lhs) != rhs;
}

template<std::size_t N>
[[deprecated("Do comparisons with std::array<std::byte, N>")]]
constexpr bool operator!=(
  const DeprecatedHelperArray<N> & lhs,
  const std::array<unsigned char, N> & rhs)
{
  return lhs != to_byte_array(rhs);
}

// <
template<std::size_t N>
[[deprecated("Do comparisons with std::array<std::byte, N>")]]
constexpr bool operator<(
  const std::array<unsigned char, N> & lhs,
  const DeprecatedHelperArray<N> & rhs)
{
  return to_byte_array(lhs) < rhs;
}

template<std::size_t N>
[[deprecated("Do comparisons with std::array<std::byte, N>")]]
constexpr bool operator<(
  const DeprecatedHelperArray<N> & lhs,
  const std::array<unsigned char, N> & rhs)
{
  return lhs < to_byte_array(rhs);
}

// <=
template<std::size_t N>
[[deprecated("Do comparisons with std::array<std::byte, N>")]]
constexpr bool operator<=(
  const std::array<unsigned char, N> & lhs,
  const DeprecatedHelperArray<N> & rhs)
{
  return to_byte_array(lhs) <= rhs;
}

template<std::size_t N>
[[deprecated("Do comparisons with std::array<std::byte, N>")]]
constexpr bool operator<=(
  const DeprecatedHelperArray<N> & lhs,
  const std::array<unsigned char, N> & rhs)
{
  return lhs <= to_byte_array(rhs);
}

// >
template<std::size_t N>
[[deprecated("Do comparisons with std::array<std::byte, N>")]]
constexpr bool operator>(
  const std::array<unsigned char, N> & lhs,
  const DeprecatedHelperArray<N> & rhs)
{
  return to_byte_array(lhs) > rhs;
}

template<std::size_t N>
[[deprecated("Do comparisons with std::array<std::byte, N>")]]
constexpr bool operator>(
  const DeprecatedHelperArray<N> & lhs,
  const std::array<unsigned char, N> & rhs)
{
  return lhs > to_byte_array(rhs);
}

// >=
template<std::size_t N>
[[deprecated("Do comparisons with std::array<std::byte, N>")]]
constexpr bool operator>=(
  const std::array<unsigned char, N> & lhs,
  const DeprecatedHelperArray<N> & rhs)
{
  return to_byte_array(lhs) >= rhs;
}

template<std::size_t N>
[[deprecated("Do comparisons with std::array<std::byte, N>")]]
constexpr bool operator>=(
  const DeprecatedHelperArray<N> & lhs,
  const std::array<unsigned char, N> & rhs)
{
  return lhs >= to_byte_array(rhs);
}

}  // namespace rosidl_runtime_cpp

namespace std
{

template<std::size_t I, std::size_t N>
constexpr rosidl_runtime_cpp::detail::ByteRef
get(rosidl_runtime_cpp::DeprecatedHelperArray<N> & arr) noexcept
{
  static_assert(I < N, "Index out of range for DeprecatedHelperArray");
  return rosidl_runtime_cpp::detail::ByteRef{arr[I]};
}

template<std::size_t I, std::size_t N>
constexpr rosidl_runtime_cpp::detail::ConstByteRef
get(const rosidl_runtime_cpp::DeprecatedHelperArray<N> & arr) noexcept
{
  static_assert(I < N, "Index out of range for DeprecatedHelperArray");
  return rosidl_runtime_cpp::detail::ConstByteRef{arr[I]};
}

template<std::size_t I, std::size_t N>
constexpr rosidl_runtime_cpp::detail::ByteRef
get(rosidl_runtime_cpp::DeprecatedHelperArray<N> && arr) noexcept
{
  static_assert(I < N, "Index out of range for DeprecatedHelperArray");
  return rosidl_runtime_cpp::detail::ByteRef{arr[I]};
}

template<std::size_t I, std::size_t N>
constexpr rosidl_runtime_cpp::detail::ConstByteRef
get(const rosidl_runtime_cpp::DeprecatedHelperArray<N> && arr) noexcept
{
  static_assert(I < N, "Index out of range for DeprecatedHelperArray");
  return rosidl_runtime_cpp::detail::ConstByteRef{arr[I]};
}

// tuple_element
template<std::size_t I, std::size_t N>
struct tuple_element<I, rosidl_runtime_cpp::DeprecatedHelperArray<N>>
{
  static_assert(I < N, "Index out of range for DeprecatedHelperArray");
  using type = unsigned char;
};
}  // namespace std

#endif  // ROSIDL_RUNTIME_CPP__DEPRECATED_HELPER_ARRAY_HPP_
