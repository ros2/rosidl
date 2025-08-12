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

template <std::size_t N>
struct DeprecatedHelperArray : std::array<std::byte, N>
{
  using ByteArray = std::array<std::byte, N>;
  using UCharArray = std::array<unsigned char, N>;

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

  [[deprecated("Switch to comparing to std::array<std::byte, N> instead")]]
  constexpr operator UCharArray() const noexcept
  {
      UCharArray dst{};
      for (size_t i = 0; i < N; i++) {
          dst[i] = std::to_integer<unsigned char>((*this)[i]);
      }
      return dst;
  }

  // Member Functions

  // Element Access

  // Iterators

  // Operations

  [[deprecated("Switch to calling fill with std::byte")]]
  constexpr void fill( const unsigned char & value)
  {
    this->fill(std::byte{value});  
  }

  [[deprecated("Switch to calling fill with std::byte")]]
  constexpr void swap( UCharArray & other)
  {
    this->swap(to_byte_array_ref(other));  
  }
  
  private:

    constexpr UCharArray & to_uchar_array_ref(ByteArray & in)
    {
      return reinterpret_cast<UCharArray &>(in);
    }
    constexpr ByteArray & to_byte_array_ref(UCharArray & in)
    {
      return reinterpret_cast<ByteArray &>(in);
    }

    // Non-member Functions
};

}  // namespace rosidl_runtime_cpp

#endif  // ROSIDL_RUNTIME_CPP__DEPRECATED_HELPER_ARRAY_HPP_
