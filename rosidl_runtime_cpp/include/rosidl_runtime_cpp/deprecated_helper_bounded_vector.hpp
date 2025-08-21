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

#ifndef ROSIDL_RUNTIME_CPP__DEPRECATED_HELPER_BOUNDED_VECTOR_HPP_
#define ROSIDL_RUNTIME_CPP__DEPRECATED_HELPER_BOUNDED_VECTOR_HPP_

#include <cstdint>
#include <cstddef>
#include <memory>

#include "byte_conversion_helpers.hpp"
#include "bounded_vector.hpp"

namespace rosidl_runtime_cpp
{
template<std::size_t UpperBound, typename Alloc = std::allocator<std::byte>>
struct DeprecatedHelperBoundedVector : BoundedVector<std::byte, UpperBound, Alloc>
{
    // // Override Defaults to match old types
    // using value_type = unsigned char;
    // using allocator_type = std::allocator<value_type>;
    // using reference = value_type &;
    // using const_reference = const reference;
    // using pointer = value_type *;
    // using const_pointer = const pointer;
    // using iterator = pointer;
    // using const_iterator = const_pointer;
    // using reverse_iterator = std::reverse_iterator<iterator>;
    // using reverse_const_iterator = std::reverse_iterator<const_iterator>;

    // // Helper Types
    // using ByteVector = BoundedVector<std::byte, UpperBound, Alloc>;
    // using UCharVector = BoundedVector<unsigned char, UpperBound, Alloc>;


    // // Member functions
    // using ByteVector::ByteVector;
    // using ByteVector::operator=;
    // using ByteVector::assign;

    // // Element Access
    // using ByteVector::at;
    // using ByteVector::operator[];
    // using ByteVector::front;
    // using ByteVector::back;
    // using ByteVector::data;

    // // Iterators
    // using ByteVector::begin;
    // using ByteVector::cbegin;
    // using ByteVector::end;
    // using ByteVector::cend;
    // using ByteVector::rbegin;
    // using ByteVector::crbegin;
    // using ByteVector::rend;
    // using ByteVector::crend;

    // // Capacity
    // using ByteVector::empty;
    // using ByteVector::size;
    // using ByteVector::max_size;
    // using ByteVector::reserve;
    // using ByteVector::capacity;
    // using ByteVector::shrink_to_fit;

    // // Modifiers
    // using ByteVector::clear;
    // using ByteVector::insert;
    // using ByteVector::emplace;
    // using ByteVector::erase;
    // using ByteVector::push_back;
    // using ByteVector::emplace_back;
    // using ByteVector::pop_back;
    // using ByteVector::resize;
    // using ByteVector::swap;
};

// // ==
// template<std::size_t N>
// [[deprecated("Do comparisons with std::array<std::byte, N>")]]
// constexpr bool operator==(
//   const std::array<unsigned char, N> & lhs,
//   const DeprecatedHelperArray<N> & rhs)
// {
//   return to_byte_array(lhs) == rhs;
// }

// template<std::size_t N>
// [[deprecated("Do comparisons with std::array<std::byte, N>")]]
// constexpr bool operator==(
//   const DeprecatedHelperArray<N> & lhs,
//   const std::array<unsigned char, N> & rhs)
// {
//   return lhs == to_byte_array(rhs);
// }

// // !=
// template<std::size_t N>
// [[deprecated("Do comparisons with std::array<std::byte, N>")]]
// constexpr bool operator!=(
//   const std::array<unsigned char, N> & lhs,
//   const DeprecatedHelperArray<N> & rhs)
// {
//   return to_byte_array(lhs) != rhs;
// }

// template<std::size_t N>
// [[deprecated("Do comparisons with std::array<std::byte, N>")]]
// constexpr bool operator!=(
//   const DeprecatedHelperArray<N> & lhs,
//   const std::array<unsigned char, N> & rhs)
// {
//   return lhs != to_byte_array(rhs);
// }

// // <
// template<std::size_t N>
// [[deprecated("Do comparisons with std::array<std::byte, N>")]]
// constexpr bool operator<(
//   const std::array<unsigned char, N> & lhs,
//   const DeprecatedHelperArray<N> & rhs)
// {
//   return to_byte_array(lhs) < rhs;
// }

// template<std::size_t N>
// [[deprecated("Do comparisons with std::array<std::byte, N>")]]
// constexpr bool operator<(
//   const DeprecatedHelperArray<N> & lhs,
//   const std::array<unsigned char, N> & rhs)
// {
//   return lhs < to_byte_array(rhs);
// }

// // <=
// template<std::size_t N>
// [[deprecated("Do comparisons with std::array<std::byte, N>")]]
// constexpr bool operator<=(
//   const std::array<unsigned char, N> & lhs,
//   const DeprecatedHelperArray<N> & rhs)
// {
//   return to_byte_array(lhs) <= rhs;
// }

// template<std::size_t N>
// [[deprecated("Do comparisons with std::array<std::byte, N>")]]
// constexpr bool operator<=(
//   const DeprecatedHelperArray<N> & lhs,
//   const std::array<unsigned char, N> & rhs)
// {
//   return lhs <= to_byte_array(rhs);
// }

// // >
// template<std::size_t N>
// [[deprecated("Do comparisons with std::array<std::byte, N>")]]
// constexpr bool operator>(
//   const std::array<unsigned char, N> & lhs,
//   const DeprecatedHelperArray<N> & rhs)
// {
//   return to_byte_array(lhs) > rhs;
// }

// template<std::size_t N>
// [[deprecated("Do comparisons with std::array<std::byte, N>")]]
// constexpr bool operator>(
//   const DeprecatedHelperArray<N> & lhs,
//   const std::array<unsigned char, N> & rhs)
// {
//   return lhs > to_byte_array(rhs);
// }

// // >=
// template<std::size_t N>
// [[deprecated("Do comparisons with std::array<std::byte, N>")]]
// constexpr bool operator>=(
//   const std::array<unsigned char, N> & lhs,
//   const DeprecatedHelperArray<N> & rhs)
// {
//   return to_byte_array(lhs) >= rhs;
// }

// template<std::size_t N>
// [[deprecated("Do comparisons with std::array<std::byte, N>")]]
// constexpr bool operator>=(
//   const DeprecatedHelperArray<N> & lhs,
//   const std::array<unsigned char, N> & rhs)
// {
//   return lhs >= to_byte_array(rhs);
// }

}  // namespace rosidl_runtime_cpp

// Non-member Function

#endif  // ROSIDL_RUNTIME_CPP__DEPRECATED_HELPER_BOUNDED_VECTOR_HPP_
