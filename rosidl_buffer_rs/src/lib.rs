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
//! Buffers wrap the runtime's `Sequence<T>` directly. Primitive sequence names
//! remain aliases; there is no second native storage representation.
//! Enable `serde` for serialization to host values.
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

pub use rosidl_runtime_rs::{
    native::{ffi, CxxBuffer},
    BufferError,
};

mod buffer;
pub use buffer::{BoundedBuffer, BoundedVec, Buffer};

pub use rosidl_runtime_rs::{
    BoundedSequence as BoundedPrimitiveSequence, Sequence as PrimitiveSequence,
    SequenceExceedsBoundsError,
};

mod traits;
pub use traits::PrimitiveSequenceAlloc;

#[doc(hidden)]
pub mod native;
