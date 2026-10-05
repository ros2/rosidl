// Copyright 2026 Open Source Robotics Foundation, Inc.
// SPDX-License-Identifier: Apache-2.0

#ifndef ROSIDL_BUFFER_RS__BUFFER_BRIDGE_HPP_
#define ROSIDL_BUFFER_RS__BUFFER_BRIDGE_HPP_

#include <cstdint>
#include <memory>

#include "rosidl_buffer/buffer.hpp"
#include "rust/cxx.h"

namespace rosidl_buffer_rs
{
using CxxBuffer = rosidl::Buffer<uint8_t>;

std::unique_ptr<CxxBuffer> create_cpu(rust::Slice<const uint8_t> data);
std::unique_ptr<CxxBuffer> clone_buffer(const CxxBuffer & buffer, int32_t & error_code);
bool are_equal(const CxxBuffer & lhs, const CxxBuffer & rhs, int32_t & error_code);
bool equals_data(const CxxBuffer & buffer, rust::Slice<const uint8_t> data, int32_t & error_code);
rust::String backend_name(const CxxBuffer & buffer, int32_t & error_code);
void copy_to_host(
  const CxxBuffer & buffer, rust::Slice<uint8_t> output, int32_t & error_code);
}  // namespace rosidl_buffer_rs

#endif  // ROSIDL_BUFFER_RS__BUFFER_BRIDGE_HPP_
