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

import array


class Buffer:
    """Python wrapper around rosidl::Buffer<uint8_t>."""

    @property
    def backend_type(self) -> str:
        """Backend type identifier (e.g. 'cpu', 'cuda')."""
        ...

    def to_bytes(self) -> bytes:
        """Copy buffer contents to Python bytes."""
        ...

    def to_array(self) -> array.array[int]:
        """Convert to array.array('B') -- the rclpy CPU storage type."""
        ...

    def __len__(self) -> int: ...
    def __repr__(self) -> str: ...


def _get_buffer_ptr(buf: Buffer) -> int: ...
def _take_buffer_from_ptr(ptr: int) -> Buffer: ...
