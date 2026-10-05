// Copyright 2026 Open Source Robotics Foundation, Inc.
// SPDX-License-Identifier: Apache-2.0

//! Backend-neutral buffers for Rust ROS messages.
//!
//! [`Buffer`] owns CPU or native backend storage. Borrow CPU sequence storage
//! with [`Buffer::as_slice`], or copy contents to the host with [`Buffer::to_vec`].
//! Native backends retain allocation ownership through CXX; device-specific
//! access is provided by packages such as `cuda_buffer_rs`.
//!
//! ```
//! use rosidl_buffer_rs::Buffer;
//!
//! let buffer = Buffer::from(vec![1u8, 2, 3]);
//! assert_eq!(buffer.as_slice(), Some(&[1, 2, 3][..]));
//! assert_eq!(buffer.backend_name().unwrap(), "cpu");
//! ```
//!
//! The crate also provides bounded buffers and ABI-compatible primitive sequences.
//! `rosidl_runtime_rs` re-exports these types, so existing imports and generated
//! messages use the same types. Enable `serde` for serialization to host values.
//!
//! # Building and testing
//!
//! Requires Rust 1.88+, a C++20 compiler, and a sourced ROS installation containing
//! `rosidl_buffer` and `rosidl_runtime_c`. Cargo downloads `cxx` and `cxx-build`.
//! For colcon, install `colcon-cargo`, `colcon-ros-cargo`, and `cargo-ament-build`.
//!
//! ```sh
//! colcon build --packages-up-to rosidl_buffer_rs
//! colcon test --packages-select rosidl_buffer_rs
//! ```

#[cxx::bridge(namespace = "rosidl_buffer_rs")]
pub mod ffi {
    // SAFETY: references borrow live native objects, slices carry their bounds,
    // and fallible C++ operations translate exceptions into Result.
    unsafe extern "C++" {
        include!("rosidl_buffer_rs/src/buffer_bridge.hpp");

        type CxxBuffer;

        fn size(self: &CxxBuffer) -> usize;
        fn create_cpu(data: &[u8]) -> Result<UniquePtr<CxxBuffer>>;
        fn clone_buffer(buffer: &CxxBuffer, error_code: &mut i32) -> Result<UniquePtr<CxxBuffer>>;
        fn are_equal(lhs: &CxxBuffer, rhs: &CxxBuffer, error_code: &mut i32) -> Result<bool>;
        fn equals_data(buffer: &CxxBuffer, data: &[u8], error_code: &mut i32) -> Result<bool>;
        fn backend_name(buffer: &CxxBuffer, error_code: &mut i32) -> Result<String>;
        fn copy_to_host(buffer: &CxxBuffer, output: &mut [u8], error_code: &mut i32) -> Result<()>;
    }
}

pub use ffi::CxxBuffer;

mod buffer;
pub use buffer::{BoundedBuffer, BoundedVec, Buffer, BufferError};

mod sequence;
pub use sequence::{
    BoundedPrimitiveSequence, PrimitiveSequence, PrimitiveSequenceIterator,
    SequenceExceedsBoundsError,
};

mod traits;
pub use traits::PrimitiveSequenceAlloc;

#[doc(hidden)]
pub mod native;
