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

#ifndef ROSIDL_RUNTIME_CPP__TYPE_HASH_HPP_
#define ROSIDL_RUNTIME_CPP__TYPE_HASH_HPP_

#include <string>
#include "rosidl_runtime_c/type_hash.h"

namespace rosidl_runtime_cpp
{

// returns the REP-2011 specified RIHS string for a type hash struct.
// returns the empty string for invalid or unset type hash values (identified with a version of 0).
std::string type_hash_to_string(const rosidl_type_hash_t & type_hash);

}  // namespace rosidl_runtime_cpp

#endif  // ROSIDL_RUNTIME_CPP__TYPE_HASH_HPP_
