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

#ifndef ROSIDL_RUNTIME_CPP__DEPRECATED_HELPER_BYTE_HPP_
#define ROSIDL_RUNTIME_CPP__DEPRECATED_HELPER_BYTE_HPP_

#include <cstdint>
#include <iostream>
#include <cstddef>

namespace rosidl_runtime_cpp
{
struct DeprecatedHelperByte
{
  // Constructor
  constexpr DeprecatedHelperByte() noexcept = default;

  [[deprecated("Initialize with std::byte instead")]]
  constexpr DeprecatedHelperByte(unsigned char value) noexcept  // NOLINT(runtime/explicit)
  : b(std::byte{value}) {}

  constexpr DeprecatedHelperByte(std::byte value) noexcept  // NOLINT(runtime/explicit)
  : b(value) {}

  // Getter
  [[deprecated("Get std::byte instead")]]
  constexpr operator unsigned char() const noexcept
  {
    return static_cast<unsigned char>(b);
  }

  constexpr operator std::byte() const noexcept
  {
    return b;
  }

  // Setter
  [[deprecated("Set std::byte instead")]]
  constexpr DeprecatedHelperByte & operator=(unsigned char value) noexcept
  {
    b = std::byte{value};
    return *this;
  }

  constexpr DeprecatedHelperByte & operator=(std::byte value) noexcept
  {
    b = value;
    return *this;
  }

  // Comparisons
  // Equals
  friend constexpr bool operator==(
    const DeprecatedHelperByte & lhs,
    const DeprecatedHelperByte & rhs) noexcept
  {
    return lhs.b == rhs.b;
  }

  friend constexpr bool operator==(
    const DeprecatedHelperByte & lhs,
    const std::byte & rhs) noexcept
  {
    return lhs.b == rhs;
  }

  friend constexpr bool operator==(
    const std::byte & lhs,
    const DeprecatedHelperByte & rhs) noexcept
  {
    return lhs == rhs.b;
  }

  [[deprecated("Check Comparison with std::byte instead")]]
  friend constexpr bool operator==(
    const DeprecatedHelperByte & lhs,
    int rhs) noexcept
  {
    return lhs.b == std::byte{static_cast<unsigned char>(rhs)};
  }

  [[deprecated("Check Comparison with std::byte instead")]]
  friend constexpr bool operator==(
    int lhs,
    const DeprecatedHelperByte & rhs) noexcept
  {
    return std::byte{static_cast<unsigned char>(lhs)} == rhs.b;
  }

  // Not Equals
  friend constexpr bool operator!=(
    const DeprecatedHelperByte & lhs,
    const DeprecatedHelperByte & rhs) noexcept
  {
    return lhs.b != rhs.b;
  }
  friend constexpr bool operator!=(
    const DeprecatedHelperByte & lhs,
    const std::byte & rhs) noexcept
  {
    return lhs.b != rhs;
  }

  friend constexpr bool operator!=(
    const std::byte & lhs,
    const DeprecatedHelperByte & rhs) noexcept
  {
    return lhs != rhs.b;
  }

  [[deprecated("Check Comparison with std::byte instead")]]
  friend constexpr bool operator!=(
    const DeprecatedHelperByte & lhs,
    int rhs) noexcept
  {
    return lhs.b != std::byte{static_cast<unsigned char>(rhs)};
  }

  [[deprecated("Check Comparison with std::byte instead")]]
  friend constexpr bool operator!=(
    int lhs,
    const DeprecatedHelperByte & rhs) noexcept
  {
    return std::byte{static_cast<unsigned char>(lhs)} != rhs.b;
  }

  // Byte supported operations

  // to_integer
  template<class IntegerType>
  constexpr IntegerType to_integer() noexcept
  {
    return std::to_integer<IntegerType>(b);
  }

  // Bit shifts
  template<class IntegerType>
  constexpr std::byte operator<<=(IntegerType shift) noexcept
  {
    return b = b << shift;
  }

  template<class IntegerType>
  constexpr std::byte operator>>=(IntegerType shift) noexcept
  {
    return b = b >> shift;
  }

private:
  std::byte b{};
};
}  // namespace rosidl_runtime_cpp

#endif  // ROSIDL_RUNTIME_CPP__DEPRECATED_HELPER_BYTE_HPP_
