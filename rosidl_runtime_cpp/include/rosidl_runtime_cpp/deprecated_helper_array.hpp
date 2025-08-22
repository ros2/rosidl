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
#include <cstddef>
#include <cstring>

#include "byte_conversion_helpers.hpp"
#include "byte_container_mixin.hpp"


// Anonymous namespace for private Helpers
namespace
{
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
struct DeprecatedHelperArray : ByteContainerMixin<std::array<std::byte, N>>
{
  using Mixin = ByteContainerMixin<std::array<std::byte, N>>;
  using Mixin::value_type;
  using Mixin::reference;
  using Mixin::const_reference;
  using Mixin::pointer;
  using Mixin::const_pointer;
  using Mixin::iterator;
  using Mixin::const_iterator;
  using Mixin::reverse_iterator;
  using Mixin::reverse_const_iterator;

  // Helper Types
  using ByteArray = std::array<std::byte, N>;
  using UCharArray = std::array<unsigned char, N>;

  // Implicitly-defined member functions

  // Suppress warning about inherited deprecated Constructor
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wdeprecated-declarations"

  using Mixin::Mixin;

  #pragma GCC diagnostic pop
  using Mixin::operator=;

  // Element Access
  using Mixin::at;
  using Mixin::operator[];
  using Mixin::front;
  using Mixin::back;
  using Mixin::data;

  // Iterators
  using Mixin::begin;
  using Mixin::cbegin;
  using Mixin::end;
  using Mixin::cend;
  using Mixin::rbegin;
  using Mixin::crbegin;
  using Mixin::rend;
  using Mixin::crend;

  // Capacity
  using Mixin::empty;
  using Mixin::size;
  using Mixin::max_size;

  // Getter
  using Mixin::operator UCharArray &;

  // Operations
  using ByteArray::fill;

  [[deprecated("Switch to calling fill with std::byte")]]
  constexpr void fill(const unsigned char & value)
  {
    ByteArray::fill(std::byte{value});
  }

  using Mixin::swap;
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

// Non-member Functions

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
