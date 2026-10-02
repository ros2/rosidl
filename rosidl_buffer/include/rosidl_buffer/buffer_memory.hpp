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

#ifndef ROSIDL_BUFFER__BUFFER_MEMORY_HPP_
#define ROSIDL_BUFFER__BUFFER_MEMORY_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>

namespace rosidl
{

/// Where a buffer's elements physically live, and how to address them.
///
/// `BufferImplBase` describes a buffer in terms of what you can *do* to it --
/// `size()`, `to_cpu()`, `clone()` -- and deliberately says nothing about where
/// the bytes are. That is the right default: it keeps the interface portable,
/// and a consumer that only wants values should use the accessors.
///
/// It is not enough for a consumer that wants to keep work on an accelerator.
/// Handed a CUDA-backed buffer, such a consumer has only `to_cpu()`, which is
/// the one thing it is trying to avoid. Its alternatives today are to know the
/// concrete implementation type and `dynamic_cast` to it -- which couples it to
/// one backend, and fails silently across a shared-library boundary -- or to
/// copy. This type is the third option: ask the buffer where its memory is, in
/// terms every backend can express and any consumer can interpret.
///
/// Deliberately close in shape to DLPack's `DLTensor`/`DLDevice`, which solves
/// the same problem for the ML frameworks: an address, tagged with what kind of
/// thing it is and which device it belongs to. Reusing that shape is worth more
/// than any originality here.
///
/// \sa BufferImplBase::memory()
struct BufferMemory
{
  /// What `handle` is. A raw address alone is not interpretable: a CUDA device
  /// pointer, a host pointer, a DMA-BUF file descriptor and an NvSciBufObj are
  /// all pointer-shaped and none can be substituted for another.
  enum class Kind : uint32_t
  {
    /// No native handle on offer. The default for any implementation that has
    /// not overridden `memory()`, and the answer every consumer must handle:
    /// fall back to `to_cpu()`, which always works.
    unknown = 0,

    /// `handle` is a host pointer, directly dereferenceable in this process.
    host = 1,

    /// `handle` is a device pointer valid in the CUDA address space of
    /// `device_id`. Process-local.
    cuda_device = 2,

    /// `handle` is a DMA-BUF file descriptor, cast from `int`. Crosses process
    /// boundaries; the receiver imports it.
    dmabuf = 3,

    /// `handle` is an `NvSciBufObj`. Crosses process and partition boundaries.
    nvscibuf = 4,
  };

  Kind kind = Kind::unknown;

  /// Interpreted according to `kind`. Null whenever `kind` is `unknown`.
  void * handle = nullptr;

  /// Which device `handle` belongs to, in whatever numbering `kind` implies --
  /// a CUDA device ordinal for `cuda_device`. -1 when not applicable, which is
  /// the only correct answer for `host` and `unknown`.
  ///
  /// Not optional for the accelerator kinds: a CUDA pointer is meaningless
  /// without knowing whose address space it is in, and assuming "the current
  /// device" is a bug that only shows up on a second GPU.
  int64_t device_id = -1;

  /// Bytes reachable from `handle`. May exceed the buffer's logical size when
  /// the implementation is a view of a larger allocation.
  size_t size_bytes = 0;

  /// Holds `handle` valid for as long as this is held.
  ///
  /// Null means "valid only while the buffer you got this from lives", which is
  /// the honest answer for an implementation that owns its allocation outright
  /// -- there is nothing to hand out, and the buffer already is the lifetime.
  ///
  /// Non-null matters when the implementation is a *view*: memory owned by
  /// something else, on loan for a while. A zero-copy transport is the case
  /// this exists for -- its buffers point into a slot that is recycled once the
  /// message is dropped, so a consumer that wants to outlive the callback, or
  /// merely to finish a kernel it launched, has no safe way to say so today.
  /// Holding this says it.
  ///
  /// It is carried here rather than fetched by a second call on purpose: an
  /// address and the reason it stays valid are one fact, and code that can ask
  /// for them separately will eventually ask for only the first. This is
  /// DLPack's `manager_ctx`/`deleter` pair in a C++ idiom.
  std::shared_ptr<void> keepalive;

  /// True when `handle` is usable. Sugar for the `kind != unknown` check every
  /// caller has to write.
  bool valid() const {return kind != Kind::unknown && handle != nullptr;}
};

}  // namespace rosidl

#endif  // ROSIDL_BUFFER__BUFFER_MEMORY_HPP_
