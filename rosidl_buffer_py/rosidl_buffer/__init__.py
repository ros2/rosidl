# Copyright 2026 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

r"""
rosidl_buffer - Python bindings for ROS 2 native buffer feature.

Provides a Buffer type that wraps rosidl::Buffer<uint8_t> and
supports vendor-specific memory backends (CPU, GPU, custom).

CPU-based buffer data is always delivered to rclpy subscribers as
plain ``array.array('B')``, so existing code works unchanged.  The
Buffer class is only instantiated for non-CPU (vendor-backed) data
where users interact with it through vendor-specific APIs.

Users never construct Buffer directly.  Instead, backend providers
supply factory functions (e.g. ``DemoBuffer.from_cpu()``) that
return Buffer objects for publishing vendor-backed data.

Example usage::

    from demo_buffer import DemoBuffer

    buf = DemoBuffer.from_cpu(b'\x00\x01\x02\x03')
    assert buf.backend_type == 'demo'
    assert len(buf) == 4
    assert buf.to_bytes() == b'\x00\x01\x02\x03'

    cpu_array = buf.to_array()  # array.array('B', ...)

    msg = Image()
    msg.data = buf  # or, for CPU route: msg.data = cpu_array
"""

from rosidl_buffer._rosidl_buffer_py import _get_buffer_ptr  # noqa: F401
from rosidl_buffer._rosidl_buffer_py import _take_buffer_from_ptr  # noqa: F401
from rosidl_buffer._rosidl_buffer_py import Buffer


__all__ = [
    'Buffer',
]
