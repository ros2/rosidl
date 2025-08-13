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

#ifndef ROSIDL_RUNTIME_CPP__BYTE_REF_HPP_
#define ROSIDL_RUNTIME_CPP__BYTE_REF_HPP_

#include <cstddef>

namespace rosidl_runtime_cpp
{

namespace detail
{

struct ByteRef
{
  constexpr ByteRef(std::byte & b)  // NOLINT(runtime/explicit)
  : ref(b) {}

  // Assignment from std::byte
  constexpr ByteRef & operator=(std::byte b)
  {
    ref = b;
    return *this;
  }

  // Assignment from unsigned char
  [[deprecated("Assign to with std::byte")]]
  constexpr ByteRef & operator=(unsigned char c)
  {
    ref = std::byte{c};
    return *this;
  }

  // Conversion to std::byte
  constexpr operator std::byte() const
  {
    return ref;
  }

  // Conversion to unsigned char
  [[deprecated("Return a std::byte")]]
  constexpr operator unsigned char() const
  {
    return std::to_integer<unsigned char>(ref);
  }

private:
  std::byte & ref;
};

// struct ConstByteRef
// {
// const std::byte &ref;

// constexpr ConstByteRef(const std::byte &b) noexcept : ref(b) {}

// constexpr operator std::byte() const noexcept { return ref; }

// constexpr operator unsigned char() const noexcept {
//     return std::to_integer<unsigned char>(ref);
// }
// };

}  // namespace detail
}  // namespace rosidl_runtime_cpp

#endif  // ROSIDL_RUNTIME_CPP__BYTE_REF_HPP_
