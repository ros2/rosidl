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
#include <pybind11/stl.h>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_buffer/buffer.hpp"

namespace py = pybind11;

/// Minimal Python wrapper around rosidl::Buffer<uint8_t>.
///
/// This class exists solely to:
///   1. Provide a Python type for isinstance checks in generated message code
///   2. Hold shared_ptr ownership so the C++ buffer stays alive
///   3. Expose the few properties/methods the pipeline needs
///
/// Users never construct this directly.  Backend factory functions
/// (e.g. demo_buffer.DemoBuffer) create the underlying C++ Buffer and
/// return it via _take_buffer_from_ptr.
class PyBuffer
{
public:
  explicit PyBuffer(std::shared_ptr<rosidl::Buffer<uint8_t>> buf)
  : buffer_(std::move(buf))
  {
  }

  size_t size() const {return buffer_->size();}

  std::string get_backend_type() const {return buffer_->get_backend_type();}

  py::bytes to_bytes() const
  {
    if (buffer_->get_backend_type() == "cpu") {
      return py::bytes(
        reinterpret_cast<const char *>(buffer_->data()),
        buffer_->size());
    }
    std::vector<uint8_t> vec = buffer_->to_vector();
    return py::bytes(
      reinterpret_cast<const char *>(vec.data()),
      vec.size());
  }

  rosidl::Buffer<uint8_t> * get_raw_buffer() {return buffer_.get();}

  std::string repr() const
  {
    return "rosidl_buffer.Buffer(size=" + std::to_string(buffer_->size()) +
           ", backend='" + buffer_->get_backend_type() + "')";
  }

private:
  std::shared_ptr<rosidl::Buffer<uint8_t>> buffer_;
};

PYBIND11_MODULE(_rosidl_buffer_py, m)
{
  m.doc() = "Python bindings for rosidl::Buffer<uint8_t>";

  py::class_<PyBuffer>(m, "Buffer")
  .def("__len__", &PyBuffer::size)
  .def("to_bytes", &PyBuffer::to_bytes,
    "Copy buffer contents to Python bytes (handles non-CPU backends via to_vector)")
  .def_property_readonly("backend_type", &PyBuffer::get_backend_type,
    "Backend type identifier (e.g. 'cpu', 'demo', 'cuda')")
  .def("__repr__", &PyBuffer::repr)
  ;

  m.def("is_buffer", [](py::object obj) -> bool {
      return py::isinstance<PyBuffer>(obj);
    }, "Check if the given object is an rosidl_buffer.Buffer");

  m.def("_get_buffer_ptr", [](PyBuffer & buf) -> uintptr_t {
      return reinterpret_cast<uintptr_t>(buf.get_raw_buffer());
    }, "Get the raw C++ Buffer pointer as an integer (internal use only)");

  m.def("_take_buffer_from_ptr", [](uintptr_t ptr) -> PyBuffer {
      auto * buf = reinterpret_cast<rosidl::Buffer<uint8_t> *>(ptr);
      auto shared_buf = std::shared_ptr<rosidl::Buffer<uint8_t>>(buf);
      return PyBuffer(shared_buf);
    }, "Take ownership of a heap-allocated Buffer* and return a Python Buffer (internal use only)");
}
