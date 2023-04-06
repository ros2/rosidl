// Copyright 2023 Foxglove Technologies Inc.
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

#include <sstream>
#include <iomanip>
#include "rosidl_runtime_cpp/type_hash.hpp"

namespace rosidl_runtime_cpp
{

std::string type_hash_to_string(const rosidl_type_hash_t & type_hash)
{
  if (type_hash.version != 1) {
    return "";
  }

  std::stringstream ss;
  ss << "RIHS01_";
  ss << std::hex;
  for (int i = 0; i < ROSIDL_TYPE_HASH_SIZE; ++i) {
    ss << std::setw(2) << std::setfill('0') << static_cast<int>(type_hash.value[i]);
  }
  return ss.str();
}

}  // namespace rosidl_runtime_cpp
