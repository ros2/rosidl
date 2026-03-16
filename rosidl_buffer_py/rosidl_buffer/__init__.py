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
supply factory functions (e.g. demo_buffer.DemoBuffer) that return
Buffer objects for publishing vendor-backed data.

Example usage:
    from demo_buffer import DemoBuffer

    buf = DemoBuffer(b'\x00\x01\x02\x03')
    assert buf.backend_type == 'demo'
    assert len(buf) == 4
    assert buf.to_bytes() == b'\x00\x01\x02\x03'

    msg = Image()
    msg.data = buf
"""

from rosidl_buffer import _rosidl_buffer_py


class Buffer:
    """
    Vendor-backed buffer for non-CPU memory backends.

    This wraps a C++ ``rosidl::Buffer<uint8_t>`` with a vendor-specific
    backend (e.g. GPU, FPGA, demo).  CPU-based data never reaches Python
    as a Buffer — it arrives as ``array.array('B')`` — so this class
    only needs to expose backend-specific metadata and a ``to_bytes()``
    escape hatch for diagnostics/logging.
    """

    __slots__ = ('_native',)

    def __init__(self, native_buffer):
        object.__setattr__(self, '_native', native_buffer)

    @property
    def backend_type(self):
        """Backend type identifier (e.g. 'cpu', 'demo', 'cuda')."""
        return object.__getattribute__(self, '_native').backend_type

    def to_bytes(self):
        """Return buffer contents as bytes (handles any backend)."""
        return object.__getattribute__(self, '_native').to_bytes()

    @property
    def is_cpu(self):
        """Check whether the buffer is backed by CPU memory."""
        return self.backend_type == 'cpu'

    def __len__(self):
        return len(object.__getattribute__(self, '_native'))

    def __repr__(self):
        native = object.__getattribute__(self, '_native')
        return (
            f'Buffer(size={len(native)}, '
            f"backend='{native.backend_type}')"
        )

    __hash__ = None


# ------------------------------------------------------------------
# Pipeline helper functions used by generated message code
# ------------------------------------------------------------------

def _take_buffer_from_ptr(ptr):
    """
    Take ownership of a heap-allocated Buffer* and return a Python Buffer.

    Called by generated C->Python conversion code (_msg_support.c).
    """
    native = _rosidl_buffer_py._take_buffer_from_ptr(ptr)
    return Buffer(native)


def _get_buffer_ptr(buf):
    """
    Get the raw C++ Buffer pointer as an integer.

    Called by generated Python->C conversion code (_msg_support.c).
    Accepts both the new Python Buffer wrapper and the native pybind11 Buffer.
    """
    if isinstance(buf, Buffer):
        return _rosidl_buffer_py._get_buffer_ptr(
            object.__getattribute__(buf, '_native'))
    return _rosidl_buffer_py._get_buffer_ptr(buf)


def is_buffer(obj):
    """Check if the given object is an rosidl_buffer.Buffer (not array.array)."""
    return isinstance(obj, Buffer) or _rosidl_buffer_py.is_buffer(obj)


__all__ = [
    'Buffer',
    'is_buffer',
]
