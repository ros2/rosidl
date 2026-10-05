// Copyright 2026 Open Source Robotics Foundation, Inc.
// SPDX-License-Identifier: Apache-2.0

/// Primitive sequence allocation and backend-aware copying and equality.
pub trait PrimitiveSequenceAlloc: Copy + Sized {
    /// Wraps the corresponding primitive sequence init function.
    fn primitive_sequence_init(seq: &mut crate::PrimitiveSequence<Self>, size: usize) -> bool;
    /// Wraps the corresponding primitive sequence fini function.
    fn primitive_sequence_fini(seq: &mut crate::PrimitiveSequence<Self>);
    /// Copies CPU storage through the C runtime and opaque byte buffers through CXX.
    fn primitive_sequence_copy(
        in_seq: &crate::PrimitiveSequence<Self>,
        out_seq: &mut crate::PrimitiveSequence<Self>,
    ) -> bool;
    /// Copies sequence contents to CPU memory.
    fn primitive_sequence_to_vec(
        seq: &crate::PrimitiveSequence<Self>,
    ) -> Result<Vec<Self>, crate::BufferError> {
        if seq.is_rosidl_buffer() {
            return Err(crate::BufferError::UnsupportedElement);
        }
        Ok(seq.as_slice().to_vec())
    }
    /// Compares CPU or Buffer-backed sequence contents.
    fn primitive_sequence_are_equal(
        lhs: &crate::PrimitiveSequence<Self>,
        rhs: &crate::PrimitiveSequence<Self>,
    ) -> bool;
}
