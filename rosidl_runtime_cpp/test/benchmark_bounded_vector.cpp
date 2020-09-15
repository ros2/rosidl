// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#include <forward_list>
#include <iterator>
#include <utility>
#include <sstream>
#include <string>

#include "rosidl_runtime_cpp/bounded_vector.hpp"

#include "performance_test_fixture/performance_test_fixture.hpp"

using performance_test_fixture::PerformanceTest;

BENCHMARK_DEFINE_F(PerformanceTest, bounded_vector)(benchmark::State & st)
{
  const size_t len = st.range(0);

  // Defined the biggest benchmark test size 4096
  rosidl_runtime_cpp::BoundedVector<int, 4096> v;

  for (auto _ : st) {
    for (unsigned int i = 0; i < len; i++) {
      v.push_back(i);
    }
    v.clear();
  }

  st.SetComplexityN(len);
}
BENCHMARK_REGISTER_F(PerformanceTest, bounded_vector)
->RangeMultiplier(2)->Range(1 << 3, 1 << 12);

BENCHMARK_DEFINE_F(PerformanceTest, bounded_vector_rvalue)(benchmark::State & st)
{
  const size_t len = st.range(0);

  // Defined the biggest benchmark test size 4096
  rosidl_runtime_cpp::BoundedVector<int, 4096> v;

  for (auto _ : st) {
    for (unsigned int i = 0; i < len; i++) {
      v.emplace_back(i);
    }
    v.clear();
  }

  st.SetComplexityN(len);
}
BENCHMARK_REGISTER_F(PerformanceTest, bounded_vector_rvalue)
->RangeMultiplier(2)->Range(1 << 3, 1 << 12);

BENCHMARK_DEFINE_F(PerformanceTest, bounded_vector_insert)(benchmark::State & st)
{
  const size_t len = st.range(0);

  // Defined the biggest benchmark test size 4096
  rosidl_runtime_cpp::BoundedVector<int, 4096> v;
  rosidl_runtime_cpp::BoundedVector<int, 4096> v2;

  for (unsigned int i = 0; i < len; i++) {
    v2.push_back(i);
  }

  for (auto _ : st) {
    v.insert(v.begin(), v2.begin(), v2.begin() + len);
    v.clear();
  }

  st.SetComplexityN(len);
}
BENCHMARK_REGISTER_F(PerformanceTest, bounded_vector_insert)
->RangeMultiplier(2)->Range(1 << 3, 1 << 12);

BENCHMARK_DEFINE_F(PerformanceTest, bounded_vector_input_iterators)(benchmark::State & st)
{
  const size_t len = st.range(0);

  // Defined the biggest benchmark test size 4096
  rosidl_runtime_cpp::BoundedVector<int, 4096> v;

  std::string vector_string;
  for (unsigned int i = 0; i < len; i++) {
    vector_string += std::to_string(i) + " ";
  }
  std::istringstream ss;
  ss.str(vector_string);
  std::istream_iterator<int> ii(ss), end;

  for (auto _ : st) {
    v.assign(ii, end);
    v.clear();
  }

  st.SetComplexityN(len);
}
BENCHMARK_REGISTER_F(PerformanceTest, bounded_vector_input_iterators)
->RangeMultiplier(2)->Range(1 << 3, 1 << 12);

BENCHMARK_DEFINE_F(PerformanceTest, bounded_vector_forward_iterators)(benchmark::State & st)
{
  const size_t len = st.range(0);

  // Defined the biggest benchmark test size 4096
  rosidl_runtime_cpp::BoundedVector<int, 4096> v;

  std::forward_list<int> l;
  for (unsigned int i = 0; i < len; i++) {
    l.push_front(i);
  }

  for (auto _ : st) {
    v.assign(l.begin(), l.end());
    v.clear();
  }

  st.SetComplexityN(len);
}
BENCHMARK_REGISTER_F(PerformanceTest, bounded_vector_forward_iterators)
->RangeMultiplier(2)->Range(1 << 3, 1 << 12);
