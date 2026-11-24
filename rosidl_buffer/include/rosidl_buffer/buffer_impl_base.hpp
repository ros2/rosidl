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

#ifndef ROSIDL_BUFFER__BUFFER_IMPL_BASE_HPP_
#define ROSIDL_BUFFER__BUFFER_IMPL_BASE_HPP_

// Revision of this header's BufferImplBase vtable layout. Bump whenever a
// virtual is added, removed or reordered. A build that mixes this header with a
// differently-revisioned copy of it is an ODR violation with a wrong vtable --
// silent at runtime -- so consumers that pin a particular copy compare this
// against a value delivered independently of the include path.
#define ROSIDL_BUFFER_SOURCE_REV 4

#include <cstddef>
#include <memory>
#include <string>

#include "rosidl_buffer/buffer_memory.hpp"

namespace rosidl
{

/// Abstract base class for all buffer implementations (CPU, CUDA, ROCm, etc.).
///
/// This base keeps only what the Buffer<T> pimpl and the serialization layer
/// need.  Backend-specific APIs (element access, resize, iterators,
/// descriptor serialization, etc.) are the responsibility of each concrete
/// implementation or the BufferBackend plugin; the CPU path goes through
/// CpuBufferImpl directly.
template<typename T>
class BufferImplBase
{
public:
  virtual ~BufferImplBase() = default;

  /// Get the backend type identifier (e.g., "cpu", "cuda", "demo").
  /// Each concrete implementation returns its own fixed identifier.
  virtual std::string get_backend_type() const = 0;

  /// Get the number of elements in the buffer.
  /// Required by the serialization layer for all backends.
  virtual size_t size() const = 0;

  /// Create a CPU copy of this buffer.
  /// If already on CPU, may return a copy or the same instance.
  /// @return New BufferImplBase instance on CPU
  virtual std::unique_ptr<BufferImplBase<T>> to_cpu() const = 0;

  /// Create a deep copy of this buffer.
  /// @return New BufferImplBase instance with copied data
  virtual std::unique_ptr<BufferImplBase<T>> clone() const = 0;

  /// Where this buffer's elements live, so a consumer can address them in
  /// place instead of copying them to the host.
  ///
  /// The default answer is "I am not saying" (`Kind::unknown`), which every
  /// caller must handle by falling back to `to_cpu()`. That default is what
  /// makes this safe to add: an implementation written before this existed
  /// keeps compiling and keeps working.
  ///
  /// **The contents are ready when this returns.** An implementation that
  /// writes asynchronously synchronizes before returning, so a caller can use
  /// the handle immediately without knowing anything about the backend's
  /// streams or fences. That costs a synchronization the caller may not have
  /// needed; making it avoidable needs a way to hand back a wait token, which
  /// is deliberately left to a later revision rather than guessed at now.
  ///
  /// The returned handle is valid for at least as long as this object. An
  /// implementation that is a *view* of memory owned elsewhere can promise more
  /// by returning a non-null `BufferMemory::keepalive`, which holds that memory
  /// valid independently of this buffer.
  virtual BufferMemory memory() const {return BufferMemory{};}

  /// `memory()`, but without the synchronization.
  ///
  /// `memory()` guarantees the contents are ready by *waiting*, which is simple
  /// and always correct and costs the caller a stall it may not need. This is
  /// the version for a caller that has its own queue and would rather order
  /// against it: instead of waiting, the implementation makes `queue` wait for
  /// whatever writes are outstanding, and returns immediately.
  ///
  /// `queue` is interpreted according to the returned `BufferMemory::kind` --
  /// a `cudaStream_t` for `Kind::cuda_device`. Passing null means "no queue of
  /// mine", which is just `memory()`.
  ///
  /// The direction matters and is deliberately the same as DLPack's: the
  /// consumer states where it will work, and the producer arranges the
  /// ordering. Handing back a fence for the consumer to interpret would mean
  /// standardizing a fence type across every accelerator, which is a far larger
  /// design and is what NvSciSync exists to be.
  ///
  /// A caller must hold the returned `BufferMemory` for as long as it is still
  /// reading -- an implementation may use `keepalive` to track outstanding
  /// reads, not merely to keep the allocation alive.
  ///
  /// The default ignores `queue` and defers to `memory()`. That is always
  /// correct: waiting more than you needed to is a cost, never a bug.
  virtual BufferMemory memory_on(void * queue) const
  {
    (void)queue;
    return memory();
  }
};

}  // namespace rosidl

#endif  // ROSIDL_BUFFER__BUFFER_IMPL_BASE_HPP_
