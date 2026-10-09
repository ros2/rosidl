// Copyright 2026 Open Source Robotics Foundation, Inc.
// SPDX-License-Identifier: Apache-2.0

//! Native buffer ownership and sequence integration.

pub use crate::{ffi, CxxBuffer};

/// Transfer a C++ buffer into the native message sequence without copying.
pub fn into_buffer(
    buffer: rosidl_runtime_rs::native::UniquePtr<CxxBuffer>,
) -> Result<crate::Buffer<u8>, crate::BufferError> {
    let size = buffer
        .as_ref()
        .ok_or(crate::BufferError::Native {
            operation: "adopt buffer",
            code: 1,
        })?
        .size();
    // SAFETY: into_raw releases the sole owner of this non-null native buffer.
    // Sequence finalization uses the canonical native buffer destructor.
    let sequence = unsafe {
        crate::PrimitiveSequence::from_owned_rosidl_buffer(buffer.into_raw().cast(), size)
    }
    .expect("validated native buffer");
    Ok(sequence.into())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn native_buffer_transfers_to_sequence_and_remains_cloneable() {
        let owner = ffi::create_cpu(&[1, 2, 3]).unwrap();
        let pointer = owner.as_ref().unwrap() as *const CxxBuffer;
        let buffer = into_buffer(owner).unwrap();
        assert_eq!(
            buffer
                .as_sequence()
                .rosidl_buffer_ptr()
                .unwrap()
                .cast_const(),
            pointer.cast()
        );
        assert_eq!(buffer.backend_name().unwrap(), "cpu");
        let clone = buffer.try_clone().unwrap();
        drop(buffer);
        assert_eq!(clone.to_vec().unwrap(), [1, 2, 3]);
        assert_eq!(clone, crate::Buffer::from(vec![1u8, 2, 3]));
    }

    #[test]
    fn native_sequence_copy_replaces_cpu_and_native_storage() {
        use crate::PrimitiveSequence;
        use rosidl_runtime_rs::SequenceAlloc;

        let source = into_buffer(ffi::create_cpu(&[2, 4, 6]).unwrap())
            .unwrap()
            .into_sequence();
        let mut target = PrimitiveSequence::from(&[9u8][..]);
        assert!(u8::sequence_copy(&source, &mut target));
        assert!(target.is_rosidl_buffer());
        assert_ne!(target.rosidl_buffer_ptr(), source.rosidl_buffer_ptr());
        assert_eq!(target, source);
        assert!(u8::sequence_copy(&source, &mut target));
        drop(source);
        assert_eq!(target.try_to_vec().unwrap(), [2, 4, 6]);

        let cpu = PrimitiveSequence::from(&[7u8, 8][..]);
        assert!(u8::sequence_copy(&cpu, &mut target));
        assert!(!target.is_rosidl_buffer());
        assert_eq!(target, cpu);
    }

    #[test]
    fn native_sequence_equality_compares_contents_in_both_directions() {
        for values in [vec![], vec![1, 2, 3]] {
            let native = into_buffer(ffi::create_cpu(&values).unwrap()).unwrap();
            let another = into_buffer(ffi::create_cpu(&values).unwrap()).unwrap();
            let cpu = crate::Buffer::from(values);
            assert_eq!(native, another);
            assert_eq!(native, cpu);
            assert_eq!(cpu, native);
            assert_ne!(native, crate::Buffer::from(vec![9u8, 8, 7]));
            assert_ne!(crate::Buffer::from(vec![9u8, 8, 7]), native);
            let different = into_buffer(ffi::create_cpu(&[9, 8, 7]).unwrap()).unwrap();
            assert_ne!(native, different);
        }
    }
}
