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

#ifndef ROSIDL_RUNTIME_CPP__BYTE_CONVERSION_HELPERS_HPP_
#define ROSIDL_RUNTIME_CPP__BYTE_CONVERSION_HELPERS_HPP_

#include <cstddef>

namespace rosidl_runtime_cpp
{

namespace detail
{

struct ByteRef
{
  constexpr ByteRef(std::byte & b)  // NOLINT(runtime/explicit)
  : ref(b) {}

  // TODO(invinciblermc)
  // Make constexpr bit_cast in C++20
  [[deprecated("Assign a std::byte &")]]
  ByteRef(unsigned char & b)  // NOLINT(runtime/explicit)
  : ref(reinterpret_cast<std::byte &>(b)) {}

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

struct ConstByteRef
{
  // Construct from const std::byte reference
  constexpr ConstByteRef(const std::byte & b) noexcept  // NOLINT(runtime/explicit)
  : ref(b) {}

  // TODO(invinciblermc)
  // Make constexpr bit_cast in C++20
  [[deprecated("Assign a const std::byte &")]]
  ConstByteRef(const unsigned char & b) noexcept  // NOLINT(runtime/explicit)
  : ref(reinterpret_cast<const std::byte &>(b)) {}

  // Conversion to unsigned char
  [[deprecated("Use static_cast<std::byte> instead")]]
  constexpr operator unsigned char() const noexcept
  {
    return std::to_integer<unsigned char>(ref);
  }

  // Conversion to std::byte
  constexpr operator std::byte() const noexcept
  {
    return ref;
  }

private:
  const std::byte & ref;
};

struct BytePtr
{
  constexpr BytePtr(std::byte * p)  // NOLINT(runtime/explicit)
  : ptr(p) {}

  // TODO(invinciblermc)
  // Make constexpr bit_cast in C++20
  [[deprecated("Assign a std::byte *")]]
  BytePtr(unsigned char * p)  // NOLINT(runtime/explicit)
  : ptr(reinterpret_cast<std::byte *>(p)) {}

  constexpr ByteRef operator*() const
  {
    return ByteRef{*ptr};
  }

  constexpr ByteRef operator[](std::size_t i) const
  {
    return ByteRef{ptr[i]};
  }

  // Pointer arithmetic
  constexpr BytePtr & operator++() {++ptr; return *this;}
  constexpr BytePtr operator++(int) {BytePtr tmp = *this; ++(*this); return tmp;}
  constexpr BytePtr & operator--() {--ptr; return *this;}
  constexpr BytePtr operator--(int) {BytePtr tmp = *this; --(*this); return tmp;}

  // Conversion to raw pointers
  explicit constexpr operator std::byte *() const {return ptr;}

  // TODO(invinciblermc)
  // Make constexpr bit_cast in C++20
  [[deprecated("static_cast<std::byte *>(arr)")]]
  operator unsigned char *() const {return reinterpret_cast<unsigned char *>(ptr);}

private:
  std::byte * ptr;
};

struct ConstBytePtr
{
  constexpr ConstBytePtr(const std::byte * p)  // NOLINT(runtime/explicit)
  : ptr(p) {}

  // TODO(invinciblermc)
  // Make constexpr bit_cast in C++20
  [[deprecated("Assign a const std::byte *")]]
  ConstBytePtr(const unsigned char * p)  // NOLINT(runtime/explicit)
  : ptr(reinterpret_cast<const std::byte *>(p)) {}

  constexpr ConstByteRef operator*() const {return ConstByteRef{*ptr};}
  constexpr ConstByteRef operator[](std::size_t i) const {return ConstByteRef{ptr[i]};}

  constexpr ConstBytePtr & operator++() {++ptr; return *this;}
  constexpr ConstBytePtr operator++(int) {ConstBytePtr tmp = *this; ++(*this); return tmp;}
  constexpr ConstBytePtr & operator--() {--ptr; return *this;}
  constexpr ConstBytePtr operator--(int) {ConstBytePtr tmp = *this; --(*this); return tmp;}

  explicit constexpr operator const std::byte *() const {return ptr;}

  // TODO(invinciblermc)
  // Make constexpr bit_cast in C++20
  [[deprecated("static_cast<const std::byte *>(arr)")]]
  operator const unsigned char *() const {return reinterpret_cast<const unsigned char *>(ptr);}

private:
  const std::byte * ptr;
};

}  // namespace detail
}  // namespace rosidl_runtime_cpp

#endif  // ROSIDL_RUNTIME_CPP__BYTE_CONVERSION_HELPERS_HPP_
