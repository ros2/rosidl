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
#include <cstddef>
#include <utility>

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

  // TODO(invinciblermc): make constexpr in C++23 if the deprecation process is still going
  // Can't be constexpr yet since reinterpret_cast is not constexpr
  [[deprecated("Use static_cast<std::byte &> instead")]]
  operator unsigned char &() noexcept
  {
    return *reinterpret_cast<unsigned char *>(&b);
  }

  // Used for comparisons with const int
  [[deprecated("Use static_cast<std::byte> instead")]]
  constexpr operator unsigned char() const noexcept
  {
    return std::to_integer<unsigned char>(b);
  }

  // To avoid collisions with unsigned char
  // Explicit needing a static_cast<> until unsigned char is removed.
  explicit constexpr operator std::byte & () noexcept
  {
    return b;
  }

  explicit constexpr operator std::byte() const noexcept
  {
    return b;
  }

private:
  std::byte b{};
};
}  // namespace rosidl_runtime_cpp

#endif  // ROSIDL_RUNTIME_CPP__DEPRECATED_HELPER_BYTE_HPP_
