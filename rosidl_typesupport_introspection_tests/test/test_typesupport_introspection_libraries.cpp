// Copyright 2022 Open Source Robotics Foundation, Inc.
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

#include <gtest/gtest.h>

#include "rosidl_typesupport_introspection_tests/gtest/typesupport_library_test.hpp"

#include "introspection_libraries_under_test.hpp"

namespace rosidl_typesupport_introspection_tests
{
namespace testing
{
namespace
{

using IntrospectionTypeSupportLibraries = ::testing::Types<
  IntrospectionCTypeSupportTestLibrary,
  IntrospectionCppTypeSupportTestLibrary>;

INSTANTIATE_TYPED_TEST_SUITE_P(
  Introspection, TypeSupportLibraryTest,
  IntrospectionTypeSupportLibraries);

}  // namespace
}  // namespace testing
}  // namespace rosidl_typesupport_introspection_tests
