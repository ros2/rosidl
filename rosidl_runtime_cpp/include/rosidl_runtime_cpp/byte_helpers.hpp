// Copyright 2017 Open Source Robotics Foundation, Inc.
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

#include <cstddef>
#include <utility>
#include <vector>
#include <array>
#include <type_traits>
#include <algorithm>
#include <memory>

#ifndef ROSIDL_RUNTIME_CPP__BYTE_HELPERS_HPP_
#define ROSIDL_RUNTIME_CPP__BYTE_HELPERS_HPP_

namespace rosidl_runtime_cpp
{

struct ByteConverter
{
  std::byte value;

  constexpr ByteConverter() noexcept
  : value(std::byte{0}) {}
  constexpr ByteConverter(std::byte b) noexcept  // NOLINT(runtime/explicit)
  : value(b) {}

  template<typename T,
    typename = std::enable_if_t<std::is_integral_v<T>>>
  [[deprecated("Direct numeric assignment is deprecated; use std::byte")]]
  constexpr ByteConverter(T numeric_value) noexcept  // NOLINT(runtime/explicit)
  : value(static_cast<std::byte>(numeric_value)) {}

  ByteConverter & operator=(std::byte b) noexcept
  {
    value = b;
    return *this;
  }

  template<typename T, typename = std::enable_if_t<std::is_integral_v<T>>>
  [[deprecated("Assignment from numeric is deprecated; use 'std::byte' instead.")]]
  ByteConverter & operator=(T c)
  {
    value = static_cast<std::byte>(c);
    return *this;
  }

  [[deprecated("Reading as 'unsigned char' is deprecated; use 'std::byte' instead.")]]
  constexpr operator unsigned char() const noexcept {return static_cast<unsigned char>(value);}

  constexpr operator std::byte() const noexcept {return value;}

  constexpr bool operator==(const ByteConverter & other) const noexcept
  {
    return value == other.value;
  }
  constexpr bool operator!=(const ByteConverter & other) const noexcept
  {
    return value != other.value;
  }
};

// Equality converted

template<typename T, typename = std::enable_if_t<std::is_integral_v<T>>>
constexpr bool operator==(const ByteConverter & lhs, T rhs) noexcept
{
  return static_cast<unsigned char>(lhs.value) == static_cast<unsigned char>(rhs);
}

template<typename T, typename = std::enable_if_t<std::is_integral_v<T>>>
constexpr bool operator==(T lhs, const ByteConverter & rhs) noexcept {return rhs == lhs;}

template<typename T, typename = std::enable_if_t<std::is_integral_v<T>>>
constexpr bool operator!=(const ByteConverter & lhs, T rhs) noexcept {return !(lhs == rhs);}

template<typename T, typename = std::enable_if_t<std::is_integral_v<T>>>
constexpr bool operator!=(T lhs, const ByteConverter & rhs) noexcept {return !(lhs == rhs);}


// Converter for Vectors

template<typename Alloc = std::allocator<ByteConverter>>
struct ByteVector : public std::vector<ByteConverter, Alloc>
{
  using Base = std::vector<ByteConverter, Alloc>;
  using Base::Base;

  operator std::vector<std::byte> &() {
    return *reinterpret_cast<std::vector<std::byte> *>(this);
  }

  operator const std::vector<std::byte> &() const {
    return *reinterpret_cast<const std::vector<std::byte> *>(this);
  }

  [[deprecated("Implicit conversion to std::vector<unsigned char> is deprecated"
               "use as std::vector<std::byte>")]]
  operator std::vector<unsigned char> &() {
    return *reinterpret_cast<std::vector<unsigned char> *>(this);
  }

  [[deprecated("Implicit conversion to std::vector<unsigned char> is deprecated"
               "use as std::vector<std::byte>")]]
  operator const std::vector<unsigned char> &() const {
    return *reinterpret_cast<const std::vector<unsigned char> *>(this);
  }
};

// Converter for Array

template<size_t N>
struct ByteArray : public std::array<ByteConverter, N>
{
  using Base = std::array<ByteConverter, N>;

  operator std::array<std::byte, N> &() {
    return reinterpret_cast<std::array<std::byte, N> &>(*this);
  }

  operator const std::array<std::byte, N> &() const {
    return reinterpret_cast<const std::array<std::byte, N> &>(*this);
  }

  [[deprecated("Using as std::array<unsigned char> is deprecated use as std::array<std::byte>")]]
  operator std::array<unsigned char, N> &() {
    return reinterpret_cast<std::array<unsigned char, N> &>(*this);
  }

  [[deprecated("Using as std::array<unsigned char> is deprecated use as std::array<std::byte>")]]
  operator const std::array<unsigned char, N> &() const {
    return reinterpret_cast<const std::array<unsigned char, N> &>(*this);
  }
};

// Container Conversion equality checks

template<typename ContainerA, typename ContainerB,
  typename = std::enable_if_t<
    (std::is_same_v<typename ContainerA::value_type, ByteConverter>||
    std::is_same_v<typename ContainerB::value_type, ByteConverter>) &&
    !std::is_integral_v<ContainerA>&& !std::is_integral_v<ContainerB>
  >>
bool operator==(const ContainerA & lhs, const ContainerB & rhs)
{
  if (lhs.size() != rhs.size()) {return false;}
  return std::equal(lhs.begin(), lhs.end(), rhs.begin());
}

template<typename ContainerA, typename ContainerB,
  typename = std::enable_if_t<
    (std::is_same_v<typename ContainerA::value_type, ByteConverter>||
    std::is_same_v<typename ContainerB::value_type, ByteConverter>)
  >>
bool operator!=(const ContainerA & lhs, const ContainerB & rhs) {return !(lhs == rhs);}

}  // namespace rosidl_runtime_cpp

#endif  // ROSIDL_RUNTIME_CPP__BYTE_HELPERS_HPP_
