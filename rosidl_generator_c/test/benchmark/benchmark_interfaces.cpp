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

#include "rosidl_runtime_c/primitives_sequence_functions.h"
#include "rosidl_runtime_c/string_functions.h"
#include "rosidl_runtime_c/u16string_functions.h"

#include "rosidl_generator_c/msg/arrays.h"
#include "rosidl_generator_c/msg/defaults.h"
#include "rosidl_generator_c/msg/basic_types.h"
#include "rosidl_generator_c/msg/bounded_sequences.h"
#include "rosidl_generator_c/msg/multi_nested.h"
#include "rosidl_generator_c/msg/nested.h"
#include "rosidl_generator_c/msg/strings.h"
#include "rosidl_generator_c/msg/unbounded_sequences.h"

#include "performance_test_fixture/performance_test_fixture.hpp"

using performance_test_fixture::PerformanceTest;

BENCHMARK_DEFINE_F(PerformanceTest, msg_array_complexity)(benchmark::State & st)
{
  rosidl_generator_c__msg__Arrays * arr = NULL;

  for (auto _ : st) {
    arr = rosidl_generator_c__msg__Arrays__create();
    if (arr == nullptr) {
      st.SkipWithError("Array create function failed");
      return;
    }

    for (unsigned int i = 0; i < 3; i++) {
      arr->bool_values[i] = true;
      arr->char_values[i] = 'a';
      arr->float32_values[i] = -3.000001f;
      arr->float64_values[i] = -120310.00843902140001;
      arr->int8_values[i] = -50;
      arr->uint8_values[i] = 0;
      arr->int16_values[i] = -2222;
      arr->uint16_values[i] = 0U;
      arr->int32_values[i] = 30;
      arr->uint32_values[i] = 0UL;
      arr->int64_values[i] = -9223372036854775807LL;
      arr->uint64_values[i] = 0ULL;
      rosidl_runtime_c__String__assign(&arr->string_values[i], "value");
    }

    rosidl_generator_c__msg__Arrays__destroy(arr);
  }
}
BENCHMARK_REGISTER_F(PerformanceTest, msg_array_complexity);

BENCHMARK_DEFINE_F(PerformanceTest, msg_default_complexity)(benchmark::State & st)
{
  rosidl_generator_c__msg__Defaults * def = NULL;

  for (auto _ : st) {
    def = rosidl_generator_c__msg__Defaults__create();
    if (def == nullptr) {
      st.SkipWithError("Default create function failed");
      return;
    }
    rosidl_generator_c__msg__Defaults__destroy(def);
  }
}
BENCHMARK_REGISTER_F(PerformanceTest, msg_default_complexity);

BENCHMARK_DEFINE_F(PerformanceTest, msg_basic_types_complexity)(benchmark::State & st)
{
  rosidl_generator_c__msg__BasicTypes * basic = NULL;

  for (auto _ : st) {
    basic = rosidl_generator_c__msg__BasicTypes__create();
    if (basic == nullptr) {
      st.SkipWithError("Default create function failed");
      return;
    }
    basic->bool_value = false;
    basic->byte_value = 25;
    basic->char_value = 0;
    basic->float32_value = 0;
    basic->float64_value = 0;
    basic->int8_value = 0;
    basic->uint8_value = 0;
    basic->int16_value = 0;
    basic->uint16_value = 0;
    basic->int32_value = 0;
    basic->uint32_value = 0;
    basic->int64_value = 0;
    basic->uint64_value = 0;
    rosidl_generator_c__msg__BasicTypes__destroy(basic);
  }
}
BENCHMARK_REGISTER_F(PerformanceTest, msg_basic_types_complexity);

BENCHMARK_DEFINE_F(PerformanceTest, msg_bounded_sequences_complexity)(benchmark::State & st)
{
  size_t len = st.range(0);
  rosidl_generator_c__msg__BoundedSequences * seq = NULL;

  for (auto _ : st) {
    seq = rosidl_generator_c__msg__BoundedSequences__create();

    if (seq == nullptr) {
      st.SkipWithError("BoundedSequences create function failed");
      return;
    }

    bool res = false;
    unsigned int i = 0;
    res = rosidl_runtime_c__boolean__Sequence__init(&seq->bool_values, len);
    if (!res) {
      st.SkipWithError("boolean__Sequence__init failed");
      return;
    }
    res = rosidl_runtime_c__byte__Sequence__init(&seq->byte_values, len);
    if (!res) {
      st.SkipWithError("byte__Sequence__init failed");
      return;
    }
    res = rosidl_runtime_c__uint8__Sequence__init(&seq->char_values, len);
    if (!res) {
      st.SkipWithError("uint8__Sequence__init failed");
      return;
    }
    res = rosidl_runtime_c__float32__Sequence__init(&seq->float32_values, len);
    if (!res) {
      st.SkipWithError("float32__Sequence__init failed");
      return;
    }
    res = rosidl_runtime_c__float64__Sequence__init(&seq->float64_values, len);
    if (!res) {
      st.SkipWithError("float64__Sequence__init failed");
      return;
    }
    res = rosidl_runtime_c__int8__Sequence__init(&seq->int8_values, len);
    if (!res) {
      st.SkipWithError("int8__Sequence__init failed");
      return;
    }
    res = rosidl_runtime_c__uint8__Sequence__init(&seq->uint8_values, len);
    if (!res) {
      st.SkipWithError("uint8__Sequence__init failed");
      return;
    }
    res = rosidl_runtime_c__int16__Sequence__init(&seq->int16_values, len);
    if (!res) {
      st.SkipWithError("int16__Sequence__init failed");
      return;
    }
    res = rosidl_runtime_c__uint16__Sequence__init(&seq->uint16_values, len);
    if (!res) {
      st.SkipWithError("uint16__Sequence__init failed");
      return;
    }
    res = rosidl_runtime_c__int32__Sequence__init(&seq->int32_values, len);
    if (!res) {
      st.SkipWithError("int32__Sequence__init failed");
      return;
    }
    res = rosidl_runtime_c__uint32__Sequence__init(&seq->uint32_values, len);
    if (!res) {
      st.SkipWithError("uint32__Sequence__init failed");
      return;
    }
    res = rosidl_runtime_c__int64__Sequence__init(&seq->int64_values, len);
    if (!res) {
      st.SkipWithError("int64__Sequence__init failed");
      return;
    }
    res = rosidl_runtime_c__uint64__Sequence__init(&seq->uint64_values, len);
    if (!res) {
      st.SkipWithError("uint64__Sequence__init failed");
      return;
    }
    res = rosidl_runtime_c__String__Sequence__init(&seq->string_values, len);
    if (!res) {
      st.SkipWithError("String__Sequence__init failed");
      return;
    }

    for (i = 0; i < len; i++) {
      seq->bool_values.data[i] = true;
      seq->char_values.data[i] = 'a';
      seq->float32_values.data[i] = -3.000001f;
      seq->float64_values.data[i] = -120310.00843902140001;
      seq->int8_values.data[i] = -50;
      seq->uint8_values.data[i] = 0;
      seq->int16_values.data[i] = -2222;
      seq->uint16_values.data[i] = 0U;
      seq->int32_values.data[i] = 30;
      seq->uint32_values.data[i] = 0UL;
      seq->int64_values.data[i] = -9223372036854775807LL;
      seq->uint64_values.data[i] = 0ULL;
      rosidl_runtime_c__String__assign(&seq->string_values.data[i], "value");
    }
    rosidl_generator_c__msg__BoundedSequences__destroy(seq);
  }
}
BENCHMARK_REGISTER_F(PerformanceTest, msg_bounded_sequences_complexity)
->RangeMultiplier(2)->Range(1 << 3, 1 << 12);

BENCHMARK_DEFINE_F(PerformanceTest, msg_multi_nested_complexity)(benchmark::State & st)
{
  rosidl_generator_c__msg__MultiNested * msg = NULL;

  for (auto _ : st) {
      msg = rosidl_generator_c__msg__MultiNested__create();
      if (msg == nullptr) {
      st.SkipWithError("Multi nested create function failed");
      return;
    }
    rosidl_generator_c__msg__MultiNested__destroy(msg);
  }
}
BENCHMARK_REGISTER_F(PerformanceTest, msg_multi_nested_complexity);

BENCHMARK_DEFINE_F(PerformanceTest, msg_nested_complexity)(benchmark::State & st)
{
  rosidl_generator_c__msg__Nested * nested = NULL;

  for (auto _ : st) {
    nested = rosidl_generator_c__msg__Nested__create();
    if (nested == nullptr) {
      st.SkipWithError("Nested create function failed");
      return;
    }
    rosidl_generator_c__msg__Nested__destroy(nested);
  }
}
BENCHMARK_REGISTER_F(PerformanceTest, msg_nested_complexity);

BENCHMARK_DEFINE_F(PerformanceTest, msg_unbounded_sequences_complexity)(benchmark::State & st)
{
  size_t len = st.range(0);
  rosidl_generator_c__msg__UnboundedSequences * seq = NULL;

  for (auto _ : st) {
    seq = rosidl_generator_c__msg__UnboundedSequences__create();

    if (seq == nullptr) {
      st.SkipWithError("BoundedSequences create function failed");
      return;
    }

    bool res = false;
    unsigned int i = 0;
    res = rosidl_runtime_c__boolean__Sequence__init(&seq->bool_values, len);
    if (!res) {
      st.SkipWithError("boolean__Sequence__init failed");
      return;
    }
    res = rosidl_runtime_c__byte__Sequence__init(&seq->byte_values, len);
    if (!res) {
      st.SkipWithError("byte__Sequence__init failed");
      return;
    }
    res = rosidl_runtime_c__uint8__Sequence__init(&seq->char_values, len);
    if (!res) {
      st.SkipWithError("uint8__Sequence__init failed");
      return;
    }
    res = rosidl_runtime_c__float32__Sequence__init(&seq->float32_values, len);
    if (!res) {
      st.SkipWithError("float32__Sequence__init failed");
      return;
    }
    res = rosidl_runtime_c__float64__Sequence__init(&seq->float64_values, len);
    if (!res) {
      st.SkipWithError("float64__Sequence__init failed");
      return;
    }
    res = rosidl_runtime_c__int8__Sequence__init(&seq->int8_values, len);
    if (!res) {
      st.SkipWithError("int8__Sequence__init failed");
      return;
    }
    res = rosidl_runtime_c__uint8__Sequence__init(&seq->uint8_values, len);
    if (!res) {
      st.SkipWithError("uint8__Sequence__init failed");
      return;
    }
    res = rosidl_runtime_c__int16__Sequence__init(&seq->int16_values, len);
    if (!res) {
      st.SkipWithError("int16__Sequence__init failed");
      return;
    }
    res = rosidl_runtime_c__uint16__Sequence__init(&seq->uint16_values, len);
    if (!res) {
      st.SkipWithError("uint16__Sequence__init failed");
      return;
    }
    res = rosidl_runtime_c__int32__Sequence__init(&seq->int32_values, len);
    if (!res) {
      st.SkipWithError("int32__Sequence__init failed");
      return;
    }
    res = rosidl_runtime_c__uint32__Sequence__init(&seq->uint32_values, len);
    if (!res) {
      st.SkipWithError("uint32__Sequence__init failed");
      return;
    }
    res = rosidl_runtime_c__int64__Sequence__init(&seq->int64_values, len);
    if (!res) {
      st.SkipWithError("int64__Sequence__init failed");
      return;
    }
    res = rosidl_runtime_c__uint64__Sequence__init(&seq->uint64_values, len);
    if (!res) {
      st.SkipWithError("uint64__Sequence__init failed");
      return;
    }
    res = rosidl_runtime_c__String__Sequence__init(&seq->string_values, len);
    if (!res) {
      st.SkipWithError("String__Sequence__init failed");
      return;
    }

    for (i = 0; i < len; i++) {
      seq->bool_values.data[i] = true;
      seq->char_values.data[i] = 'a';
      seq->float32_values.data[i] = -3.000001f;
      seq->float64_values.data[i] = -120310.00843902140001;
      seq->int8_values.data[i] = -50;
      seq->uint8_values.data[i] = 0;
      seq->int16_values.data[i] = -2222;
      seq->uint16_values.data[i] = 0U;
      seq->int32_values.data[i] = 30;
      seq->uint32_values.data[i] = 0UL;
      seq->int64_values.data[i] = -9223372036854775807LL;
      seq->uint64_values.data[i] = 0ULL;
      rosidl_runtime_c__String__assign(&seq->string_values.data[i], "value");
    }
    rosidl_generator_c__msg__UnboundedSequences__destroy(seq);
  }
}
BENCHMARK_REGISTER_F(PerformanceTest, msg_unbounded_sequences_complexity)
->RangeMultiplier(2)->Range(1 << 3, 1 << 12);
