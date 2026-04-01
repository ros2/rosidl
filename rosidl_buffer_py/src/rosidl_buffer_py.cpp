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

/// Thin Python wrapper around rosidl::Buffer<uint8_t>.
///
/// This wrapper exists to:
///   1. Hold shared_ptr ownership so the C++ buffer stays alive
///   2. Expose only the narrow API that Python users and the pipeline need
///
/// Users never construct this directly.  Backend factory functions
/// (e.g. DemoBuffer.from_cpu()) create the underlying C++ Buffer
/// and return it via _take_buffer_from_ptr.
class PyBuffer
{
public:
  explicit PyBuffer(std::shared_ptr<rosidl::Buffer<uint8_t>> buf)
  : buffer_(std::move(buf))
  {
  }

  size_t size() const {return buffer_->size();}

  std::string get_backend_type() const {return buffer_->get_backend_type();}

  /// Copy buffer contents to Python bytes (all backends).
  /// Used by rosidl_runtime_py (message_to_ordereddict, message_to_yaml)
  /// to serialize buffer data to dicts, YAML, and CSV.
  py::bytes to_bytes() const
  {
    std::vector<uint8_t> vec = buffer_->to_vector();
    return py::bytes(
      reinterpret_cast<const char *>(vec.data()),
      vec.size());
  }

  /// Convert to array.array('B') — the rclpy CPU storage type.
  py::object to_array() const
  {
    py::module_ array_mod = py::module_::import("array");
    return array_mod.attr("array")("B", to_bytes());
  }

  rosidl::Buffer<uint8_t> * get_raw_buffer() {return buffer_.get();}

  std::string repr() const
  {
    return "Buffer(size=" + std::to_string(buffer_->size()) +
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
  .def_property_readonly("backend_type", &PyBuffer::get_backend_type,
    "Backend type identifier (e.g. 'cpu')")
  .def("to_bytes", &PyBuffer::to_bytes,
    "Copy buffer contents to Python bytes.")
  .def("to_array", &PyBuffer::to_array,
    "Convert to array.array('B') — the rclpy CPU storage type.")
  .def("__repr__", &PyBuffer::repr)
  ;

  // Internal helpers for generated _msg_support.c (plain C code).
  // These use uintptr_t because the C caller cannot unwrap pybind11 types.

  // Returns a raw pointer to the underlying Buffer. Ownership is not
  // transferred; the PyBuffer retains ownership and the pointer is only
  // valid for the lifetime of this PyBuffer (borrow semantics).
  m.def("_get_buffer_ptr", [](PyBuffer & buf) -> uintptr_t {
      return reinterpret_cast<uintptr_t>(buf.get_raw_buffer());
    });

  // Takes ownership of a heap-allocated Buffer<uint8_t>. The pointer
  // must have been allocated with `new` so that the default `delete`
  // deleter is valid. Ownership is transferred to the returned PyBuffer.
  m.def("_take_buffer_from_ptr", [](uintptr_t ptr) -> PyBuffer {
      auto * buf = reinterpret_cast<rosidl::Buffer<uint8_t> *>(ptr);
      auto shared_buf = std::shared_ptr<rosidl::Buffer<uint8_t>>(buf);
      return PyBuffer(shared_buf);
    });
}
