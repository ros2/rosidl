// Copyright 2023 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

#ifndef ROSIDL_RUNTIME_CPP__CDR_COMPATIBLE_FIXED_CAPACITY_STRING_HPP_
#define ROSIDL_RUNTIME_CPP__CDR_COMPATIBLE_FIXED_CAPACITY_STRING_HPP_

#include <cstring>
#include <memory>
#include <ostream>
#include <stdexcept>
#include <string>

namespace rosidl_runtime_cpp
{

/// A fixed-size, POD type, CDR binary compatible string.
/**
 * Meets the same requirements as std::string.
 *
 * \param Capacity Maximum number of characters including the NUL terminator
 */
template<uint32_t Capacity>
class CDRCompatibleFixedCapacityString
{
public:

  static_assert(Capacity > 0, "Should have space for at least the NUL terminator");
  static_assert(0 == (Capacity % sizeof(uint32_t)), "Needs alignment as uint32_t");

  // Default constructor
  CDRCompatibleFixedCapacityString()
  {
    clear();
  }

  // Implicit conversion from C string
  CDRCompatibleFixedCapacityString(const char * const str)
      : CDRCompatibleFixedCapacityString()
  {
    if (nullptr == str) {
      throw std::invalid_argument(
          "Constructing CDRCompatibleFixedCapacityString with null pointer");
    }

    append_value(str);
  }

  // Implicit conversion from C++ string
  CDRCompatibleFixedCapacityString(const std::string& str)
      : CDRCompatibleFixedCapacityString()
  {
    append_value(str.c_str(), str.length());
  }

  // Implicit conversion to C++ string
  operator std::string() const { return std::string(m_string); }

  inline constexpr bool empty() const noexcept { return '\0' == m_string[0u]; }

  constexpr char* data() noexcept { return m_string; }

  constexpr const char* data() const noexcept { return m_string; }

  constexpr const char* c_str() const noexcept { return m_string; }

  inline constexpr size_t capacity() const { return m_capacity - 1u; }

  inline constexpr size_t max_size() const { return m_capacity - 1u; }

  size_t size() const noexcept { return length(); }

  size_t length() const noexcept
  {
    return ::strnlen(m_string, this->capacity());
  }

  char& operator[](size_t idx)
  {
    if (idx >= m_capacity) {
      throw std::out_of_range("idx >= m_capacity");
    }
    return m_string[idx];
  }

  char operator[](size_t idx) const
  {
    if (idx >= m_capacity) {
      throw std::out_of_range("idx >= m_capacity");
    }
    return m_string[idx];
  }

  void clear() noexcept { ::memset(m_string, 0, m_capacity); }

 private:

  void append_value(const char* const str) { append_value(str, ::strlen(str)); }

  void append_value(const char * const str, size_t len)
  {
    size_t my_len = this->length();
    size_t new_len = my_len + len;

    if (new_len > capacity()) {
      // TODO(MiguelCompany): Make this configurable
      throw std::overflow_error("Appending string will overflow capacity");
    }

    ::memmove(&m_string[my_len], str, len);
    m_string[new_len] = '\0';
  }

  // NOTE: In order for this to be binary compatible with its CDR representation, the following
  // two fields shall be the only attributes in this class.

  uint32_t m_capacity = Capacity;
  char m_string[Capacity];
};

template <uint32_t Capacity>
typename ::std::basic_ostream<char>& operator<<(
    std::basic_ostream<char>& out_stream,
    const CDRCompatibleFixedCapacityString<Capacity>& str)
{
  return out_stream.write(str.c_str(),
                          static_cast<std::streamsize>(str.size()));
}

}  // namespace rosidl_runtime_cpp

#endif  // ROSIDL_RUNTIME_CPP__CDR_COMPATIBLE_FIXED_CAPACITY_STRING_HPP_
