// Copyright 2026 Open Source Robotics Foundation, Inc.
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

#include <pybind11/pybind11.h>

#include <cstdint>
#include <memory>
#include <string>
#include <utility>

#include "rosidl_buffer/buffer.hpp"
#include "rosidl_buffer/cpu_buffer_impl.hpp"

namespace py = pybind11;
using SharedBuffer = std::shared_ptr<rosidl::Buffer<uint8_t>>;

class TrackedBufferImpl : public rosidl::CpuBufferImpl<uint8_t>
{
public:
  explicit TrackedBufferImpl(std::shared_ptr<size_t> destructions)
  : destructions_(std::move(destructions))
  {
    get_storage() = {0, 1, 127, 255};
  }

  ~TrackedBufferImpl() override {++*destructions_;}

  std::string get_backend_type() const override {return "ownership_test";}

private:
  std::shared_ptr<size_t> destructions_;
};

py::capsule make_capsule(SharedBuffer buffer, const char * name)
{
  auto holder = std::make_unique<SharedBuffer>(std::move(buffer));
  py::capsule capsule(holder.get(), name, [](PyObject * object) {
      delete static_cast<SharedBuffer *>(
        PyCapsule_GetPointer(object, PyCapsule_GetName(object)));
    });
  holder.release();
  return capsule;
}

class Owner
{
public:
  Owner()
  : destructions_(std::make_shared<size_t>(0)),
    buffer_(std::make_shared<rosidl::Buffer<uint8_t>>(
        std::make_unique<TrackedBufferImpl>(destructions_))) {}

  py::capsule capsule() const
  {
    return make_capsule(buffer_, "rosidl_buffer.shared_buffer");
  }

  size_t destructions() const {return *destructions_;}
  long use_count() const {return buffer_.use_count();}  // NOLINT(runtime/int)
  uintptr_t pointer() const {return reinterpret_cast<uintptr_t>(buffer_.get());}
  void reset() {buffer_.reset();}

  // Exercise the old ABI deliberately; only the legacy compatibility test uses it.
  uintptr_t legacy_pointer()
  {
    return reinterpret_cast<uintptr_t>(
      new rosidl::Buffer<uint8_t>(std::make_unique<TrackedBufferImpl>(destructions_)));
  }

private:
  std::shared_ptr<size_t> destructions_;
  SharedBuffer buffer_;
};

// Never installed; no CUDA or Torch dependency.
PYBIND11_MODULE(_rosidl_buffer_test_utils, module)
{
  py::class_<Owner>(module, "Owner")
  .def(py::init<>())
  .def("capsule", &Owner::capsule)
  .def("reset", &Owner::reset)
  .def("legacy_pointer", &Owner::legacy_pointer)
  .def_property_readonly("destructions", &Owner::destructions)
  .def_property_readonly("use_count", &Owner::use_count)
  .def_property_readonly("pointer", &Owner::pointer);

  module.def("empty_capsule", []() {
      return make_capsule({}, "rosidl_buffer.shared_buffer");
    });
  module.def("wrong_name_capsule", []() {
      return make_capsule({}, "unrelated.shared_buffer");
    });
  module.def("versioned_capsule", []() {
      return make_capsule({}, "rosidl_buffer.shared_buffer.v1");
    });
  module.def("unnamed_capsule", []() {
      return make_capsule({}, nullptr);
    });
}
