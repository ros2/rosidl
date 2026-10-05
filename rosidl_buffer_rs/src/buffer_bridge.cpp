// Copyright 2026 Open Source Robotics Foundation, Inc.
// SPDX-License-Identifier: Apache-2.0

#include "rosidl_buffer_rs/src/buffer_bridge.hpp"

#include <algorithm>
#include <new>
#include <stdexcept>
#include <vector>

namespace rosidl_buffer_rs
{
namespace
{
template<typename Function>
auto checked(int32_t & code, Function && function) -> decltype(function())
{
  try {
    return function();
  } catch (const std::invalid_argument &) {
    code = 1;
    throw;
  } catch (const std::bad_alloc &) {
    code = 3;
    throw;
  } catch (const std::exception &) {
    code = 4;
    throw;
  } catch (...) {
    code = 4;
    throw std::runtime_error("unknown native buffer error");
  }
}
}  // namespace

std::unique_ptr<CxxBuffer> create_cpu(rust::Slice<const uint8_t> data)
{
  auto buffer = std::make_unique<CxxBuffer>(data.size());
  if (!data.empty()) {
    std::copy(data.begin(), data.end(), buffer->data());
  }
  return buffer;
}

std::unique_ptr<CxxBuffer> clone_buffer(const CxxBuffer & buffer, int32_t & error_code)
{
  return checked(error_code, [&]() {return std::make_unique<CxxBuffer>(buffer);});
}

bool are_equal(const CxxBuffer & lhs, const CxxBuffer & rhs, int32_t & error_code)
{
  return checked(error_code, [&]() {return lhs == rhs;});
}

bool equals_data(const CxxBuffer & buffer, rust::Slice<const uint8_t> data, int32_t & error_code)
{
  return checked(error_code, [&]() {
             if (buffer.size() != data.size()) {
               return false;
             }
             const auto values = buffer.to_vector();
             return std::equal(values.begin(), values.end(), data.begin());
    });
}

rust::String backend_name(const CxxBuffer & buffer, int32_t & error_code)
{
  return checked(error_code, [&]() {return rust::String(buffer.get_backend_type());});
}

void copy_to_host(
  const CxxBuffer & buffer, rust::Slice<uint8_t> output, int32_t & error_code)
{
  checked(error_code, [&]() {
      if (output.size() < buffer.size()) {
        throw std::invalid_argument("output is smaller than the buffer");
      }
      const auto data = buffer.to_vector();
      if (!data.empty()) {
        std::copy(data.begin(), data.end(), output.begin());
      }
    });
}
}  // namespace rosidl_buffer_rs
