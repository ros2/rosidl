// Copyright 2026 Open Source Robotics Foundation, Inc.
// SPDX-License-Identifier: Apache-2.0

/// Primitive element types supported by buffer storage.
pub trait PrimitiveSequenceAlloc: rosidl_runtime_rs::SequenceAlloc + Copy + PartialEq {}
macro_rules! primitive_elements {
    ($($ty:ty),*) => { $(impl PrimitiveSequenceAlloc for $ty {})* };
}
primitive_elements!(bool, u8, i8, u16, i16, u32, i32, u64, i64, f32, f64);
