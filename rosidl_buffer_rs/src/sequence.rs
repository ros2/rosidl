// Copyright 2026 Open Source Robotics Foundation, Inc.
// SPDX-License-Identifier: Apache-2.0

//! Primitive storage shared by buffers and native ROS message fields.

use std::{
    cmp::Ordering,
    fmt::{self, Debug, Display},
    hash::{Hash, Hasher},
    iter::{Extend, FromIterator, FusedIterator},
    ops::{Deref, DerefMut},
};

use crate::PrimitiveSequenceAlloc;

#[cfg(feature = "serde")]
mod serde;

/// A sequence or buffer exceeds its declared IDL bound.
#[derive(Debug)]
pub struct SequenceExceedsBoundsError {
    /// The actual length the sequence would have after the operation.
    pub len: usize,
    /// The upper bound on the sequence length.
    pub upper_bound: usize,
}

/// An ABI-compatible `rosidl_runtime_c` primitive sequence.
///
/// CPU sequences support slice access. Opaque Buffer sequences support cloning,
/// equality, length queries, and metadata formatting. Slice access, iteration,
/// ordering, and hashing panic for opaque storage.
#[repr(C)]
pub struct PrimitiveSequence<T: PrimitiveSequenceAlloc> {
    data: *mut T,
    size: usize,
    capacity: usize,
    is_rosidl_buffer: bool,
    owns_rosidl_buffer: bool,
}

/// A bounded primitive sequence.
#[derive(Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[repr(transparent)]
pub struct BoundedPrimitiveSequence<T: PrimitiveSequenceAlloc, const N: usize> {
    inner: PrimitiveSequence<T>,
}

/// A by-value iterator over a normal, C-allocated primitive sequence.
pub struct PrimitiveSequenceIterator<T: PrimitiveSequenceAlloc> {
    seq: PrimitiveSequence<T>,
    idx: usize,
}

impl<T: PrimitiveSequenceAlloc> PrimitiveSequence<T> {
    /// Creates a sequence of `len` zero-initialized elements.
    pub fn new(len: usize) -> Self {
        Self::try_new(len).expect("PrimitiveSequence initialization failed")
    }

    /// Allocates zero-initialized elements, reporting native allocation failure.
    pub fn try_new(len: usize) -> Result<Self, crate::BufferError> {
        let mut seq = Self::default();
        if !T::primitive_sequence_init(&mut seq, len) {
            return Err(crate::BufferError::Allocation(
                "primitive sequence allocation failed".into(),
            ));
        }
        Ok(seq)
    }

    /// Takes ownership of CPU storage allocated by the matching C sequence allocator.
    ///
    /// # Safety
    ///
    /// The parts must describe a live C-allocated sequence of this element type,
    /// with `size <= capacity` and `size` initialized elements. A null pointer is
    /// valid only for empty storage. No other owner may release the allocation.
    pub unsafe fn from_cpu_raw_parts(data: *mut T, size: usize, capacity: usize) -> Self {
        Self {
            data,
            size,
            capacity,
            is_rosidl_buffer: false,
            owns_rosidl_buffer: false,
        }
    }

    /// Releases CPU storage to its caller without deallocating it.
    ///
    /// The caller must eventually release it with the matching C sequence
    /// allocator. Opaque backend storage is returned unchanged as `Err(self)`.
    pub fn into_cpu_raw_parts(self) -> Result<(*mut T, usize, usize), Self> {
        if self.is_rosidl_buffer {
            return Err(self);
        }
        let sequence = std::mem::ManuallyDrop::new(self);
        Ok((sequence.data, sequence.size, sequence.capacity))
    }

    /// Returns whether this sequence contains an opaque `rosidl::Buffer<T>`.
    pub fn is_rosidl_buffer(&self) -> bool {
        self.is_rosidl_buffer
    }

    /// Returns the number of elements represented by the sequence.
    pub fn len(&self) -> usize {
        self.size
    }

    /// Returns whether the sequence contains no elements.
    pub fn is_empty(&self) -> bool {
        self.size == 0
    }

    /// Extracts a slice from a normal primitive sequence.
    ///
    /// Panics when the C sequence contains an opaque `rosidl::Buffer<T>`.
    pub fn as_slice(&self) -> &[T] {
        self.assert_contiguous();
        if self.data.is_null() {
            &[]
        } else {
            unsafe { std::slice::from_raw_parts(self.data, self.size) }
        }
    }

    /// Extracts a mutable slice from a normal primitive sequence.
    ///
    /// Panics when the C sequence contains an opaque `rosidl::Buffer<T>`.
    pub fn as_mut_slice(&mut self) -> &mut [T] {
        self.assert_contiguous();
        if self.data.is_null() {
            &mut []
        } else {
            unsafe { std::slice::from_raw_parts_mut(self.data, self.size) }
        }
    }

    fn assert_contiguous(&self) {
        assert!(
            !self.is_rosidl_buffer,
            "an opaque rosidl buffer cannot be viewed as a primitive slice"
        );
    }
}

impl<T: PrimitiveSequenceAlloc> PrimitiveSequence<T> {
    /// Copies contents to a host vector, including opaque backend storage.
    pub fn try_to_vec(&self) -> Result<Vec<T>, crate::BufferError> {
        T::primitive_sequence_to_vec(self)
    }

    /// Materializes opaque storage; contiguous storage retains its allocation.
    pub fn try_into_cpu(self) -> Result<Self, crate::BufferError> {
        if self.is_rosidl_buffer {
            let values = self.try_to_vec()?;
            let mut sequence = Self::try_new(values.len())?;
            sequence.as_mut_slice().copy_from_slice(&values);
            Ok(sequence)
        } else {
            Ok(self)
        }
    }

    pub(crate) fn opaque_ptr(&self) -> Option<*mut std::ffi::c_void> {
        self.is_rosidl_buffer.then_some(self.data.cast())
    }
}

impl<T: PrimitiveSequenceAlloc> Clone for PrimitiveSequence<T> {
    fn clone(&self) -> Self {
        let mut seq = Self::default();
        if !T::primitive_sequence_copy(self, &mut seq) {
            panic!("Cloning PrimitiveSequence failed");
        }
        seq
    }
}

impl<T: Debug + PrimitiveSequenceAlloc> Debug for PrimitiveSequence<T> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.is_rosidl_buffer {
            f.debug_struct("PrimitiveSequence")
                .field("len", &self.size)
                .field("buffer", &self.data)
                .field("owned", &self.owns_rosidl_buffer)
                .finish()
        } else {
            self.as_slice().fmt(f)
        }
    }
}

impl<T: PrimitiveSequenceAlloc> Default for PrimitiveSequence<T> {
    fn default() -> Self {
        Self {
            data: std::ptr::null_mut(),
            size: 0,
            capacity: 0,
            is_rosidl_buffer: false,
            owns_rosidl_buffer: false,
        }
    }
}

impl<T: PrimitiveSequenceAlloc> Deref for PrimitiveSequence<T> {
    type Target = [T];

    fn deref(&self) -> &Self::Target {
        self.as_slice()
    }
}

impl<T: PrimitiveSequenceAlloc> DerefMut for PrimitiveSequence<T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.as_mut_slice()
    }
}

impl<T: PrimitiveSequenceAlloc> Drop for PrimitiveSequence<T> {
    fn drop(&mut self) {
        T::primitive_sequence_fini(self);
    }
}

impl<T: PrimitiveSequenceAlloc> Extend<T> for PrimitiveSequence<T> {
    fn extend<I: IntoIterator<Item = T>>(&mut self, iter: I) {
        self.assert_contiguous();
        let old_len = self.size;
        let appended: Vec<T> = iter.into_iter().collect();
        if appended.is_empty() {
            return;
        }
        let len = old_len
            .checked_add(appended.len())
            .expect("sequence length overflow");
        let mut replacement = Self::new(len);
        let (prefix, suffix) = replacement.as_mut_slice().split_at_mut(old_len);
        prefix.copy_from_slice(self.as_slice());
        suffix.copy_from_slice(&appended);
        *self = replacement;
    }
}

impl<T: PrimitiveSequenceAlloc + Copy> From<&[T]> for PrimitiveSequence<T> {
    fn from(slice: &[T]) -> Self {
        let mut seq = Self::new(slice.len());
        seq.as_mut_slice().copy_from_slice(slice);
        seq
    }
}

impl<T: PrimitiveSequenceAlloc> From<Vec<T>> for PrimitiveSequence<T> {
    fn from(values: Vec<T>) -> Self {
        Self::from(values.as_slice())
    }
}

impl<T: PrimitiveSequenceAlloc + Copy> From<PrimitiveSequence<T>> for Vec<T> {
    fn from(seq: PrimitiveSequence<T>) -> Self {
        seq.as_slice().to_vec()
    }
}

impl<T: PrimitiveSequenceAlloc> FromIterator<T> for PrimitiveSequence<T> {
    fn from_iter<I: IntoIterator<Item = T>>(iter: I) -> Self {
        let values: Vec<T> = iter.into_iter().collect();
        Self::from(values.as_slice())
    }
}

impl<T: PrimitiveSequenceAlloc> IntoIterator for PrimitiveSequence<T> {
    type Item = T;
    type IntoIter = PrimitiveSequenceIterator<T>;

    fn into_iter(self) -> Self::IntoIter {
        self.assert_contiguous();
        PrimitiveSequenceIterator { seq: self, idx: 0 }
    }
}

impl<T: PrimitiveSequenceAlloc + PartialEq> PartialEq for PrimitiveSequence<T> {
    fn eq(&self, other: &Self) -> bool {
        T::primitive_sequence_are_equal(self, other)
    }
}

impl<T: PrimitiveSequenceAlloc + Eq> Eq for PrimitiveSequence<T> {}

impl<T: PrimitiveSequenceAlloc + PartialOrd> PartialOrd for PrimitiveSequence<T> {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        self.as_slice().partial_cmp(other.as_slice())
    }
}

impl<T: PrimitiveSequenceAlloc + Ord> Ord for PrimitiveSequence<T> {
    fn cmp(&self, other: &Self) -> Ordering {
        self.as_slice().cmp(other.as_slice())
    }
}

impl<T: PrimitiveSequenceAlloc + Hash> Hash for PrimitiveSequence<T> {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.as_slice().hash(state);
    }
}

unsafe impl<T: PrimitiveSequenceAlloc + Send> Send for PrimitiveSequence<T> {}
unsafe impl<T: PrimitiveSequenceAlloc + Sync> Sync for PrimitiveSequence<T> {}

impl PrimitiveSequence<u8> {
    /// Takes ownership of an opaque `rosidl::Buffer<u8>`.
    ///
    /// # Safety
    ///
    /// `buffer` must be a non-null pointer returned by the ROS IDL Buffer API,
    /// `size` must be its byte length, and ownership must not be retained
    /// elsewhere.
    pub unsafe fn from_owned_rosidl_buffer(
        buffer: *mut std::ffi::c_void,
        size: usize,
    ) -> Option<Self> {
        if buffer.is_null() {
            return None;
        }
        Some(Self {
            data: buffer.cast(),
            size,
            capacity: size,
            is_rosidl_buffer: true,
            owns_rosidl_buffer: true,
        })
    }

    /// Returns the opaque Buffer pointer when this sequence is Buffer-backed.
    pub fn rosidl_buffer_ptr(&self) -> Option<*mut std::ffi::c_void> {
        self.is_rosidl_buffer.then_some(self.data.cast())
    }

    /// Releases and returns the owned opaque Buffer pointer.
    ///
    /// Returns `Err(self)` for normal sequences or non-owning Buffer views.
    pub fn into_owned_rosidl_buffer(mut self) -> Result<*mut std::ffi::c_void, Self> {
        if !self.is_rosidl_buffer || !self.owns_rosidl_buffer {
            return Err(self);
        }
        let buffer = self.data.cast();
        self.data = std::ptr::null_mut();
        self.size = 0;
        self.capacity = 0;
        self.is_rosidl_buffer = false;
        self.owns_rosidl_buffer = false;
        Ok(buffer)
    }
}

impl<T: PrimitiveSequenceAlloc, const N: usize> BoundedPrimitiveSequence<T, N> {
    /// Creates a bounded sequence of `len` zero-initialized elements.
    pub fn new(len: usize) -> Self {
        Self::try_new(len).unwrap()
    }

    /// Attempts to create a bounded sequence.
    pub fn try_new(len: usize) -> Result<Self, SequenceExceedsBoundsError> {
        if len > N {
            return Err(SequenceExceedsBoundsError {
                len,
                upper_bound: N,
            });
        }
        Ok(Self {
            inner: PrimitiveSequence::new(len),
        })
    }

    /// Returns whether this sequence contains an opaque `rosidl::Buffer<T>`.
    pub fn is_rosidl_buffer(&self) -> bool {
        self.inner.is_rosidl_buffer()
    }

    /// Returns the number of elements, including for opaque storage.
    pub fn len(&self) -> usize {
        self.inner.len()
    }

    /// Returns whether the sequence has no elements.
    pub fn is_empty(&self) -> bool {
        self.inner.is_empty()
    }

    /// Extracts a slice from a normal bounded primitive sequence.
    pub fn as_slice(&self) -> &[T] {
        self.inner.as_slice()
    }

    /// Extracts a mutable slice from a normal bounded primitive sequence.
    pub fn as_mut_slice(&mut self) -> &mut [T] {
        self.inner.as_mut_slice()
    }
}

impl<T: PrimitiveSequenceAlloc, const N: usize> Default for BoundedPrimitiveSequence<T, N> {
    fn default() -> Self {
        Self {
            inner: PrimitiveSequence::default(),
        }
    }
}

impl<T: PrimitiveSequenceAlloc + Debug, const N: usize> Debug for BoundedPrimitiveSequence<T, N> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        self.inner.fmt(f)
    }
}

impl<T: PrimitiveSequenceAlloc, const N: usize> Deref for BoundedPrimitiveSequence<T, N> {
    type Target = [T];

    fn deref(&self) -> &Self::Target {
        self.inner.deref()
    }
}

impl<T: PrimitiveSequenceAlloc, const N: usize> DerefMut for BoundedPrimitiveSequence<T, N> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.inner.deref_mut()
    }
}

impl<T: PrimitiveSequenceAlloc, const N: usize> TryFrom<Vec<T>> for BoundedPrimitiveSequence<T, N> {
    type Error = SequenceExceedsBoundsError;

    fn try_from(values: Vec<T>) -> Result<Self, Self::Error> {
        if values.len() > N {
            return Err(SequenceExceedsBoundsError {
                len: values.len(),
                upper_bound: N,
            });
        }
        Ok(Self {
            inner: values.into(),
        })
    }
}

impl<T: PrimitiveSequenceAlloc + Copy, const N: usize> TryFrom<&[T]>
    for BoundedPrimitiveSequence<T, N>
{
    type Error = SequenceExceedsBoundsError;

    fn try_from(values: &[T]) -> Result<Self, Self::Error> {
        Self::try_from(values.to_vec())
    }
}

impl<T: PrimitiveSequenceAlloc, const N: usize> FromIterator<T> for BoundedPrimitiveSequence<T, N> {
    fn from_iter<I: IntoIterator<Item = T>>(iter: I) -> Self {
        let values: Vec<T> = iter.into_iter().take(N).collect();
        Self {
            inner: values.into(),
        }
    }
}

impl<T: PrimitiveSequenceAlloc, const N: usize> IntoIterator for BoundedPrimitiveSequence<T, N> {
    type Item = T;
    type IntoIter = PrimitiveSequenceIterator<T>;

    fn into_iter(self) -> Self::IntoIter {
        self.inner.into_iter()
    }
}

impl<T: PrimitiveSequenceAlloc> Iterator for PrimitiveSequenceIterator<T> {
    type Item = T;

    fn next(&mut self) -> Option<Self::Item> {
        if self.idx >= self.seq.size {
            return None;
        }
        let elem = unsafe { self.seq.data.add(self.idx).read() };
        self.idx += 1;
        Some(elem)
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        let len = self.seq.size - self.idx;
        (len, Some(len))
    }
}

impl<T: PrimitiveSequenceAlloc> ExactSizeIterator for PrimitiveSequenceIterator<T> {}
impl<T: PrimitiveSequenceAlloc> FusedIterator for PrimitiveSequenceIterator<T> {}

impl Display for SequenceExceedsBoundsError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> Result<(), fmt::Error> {
        write!(
            f,
            "BoundedSequence with upper bound {} initialized with len {}",
            self.upper_bound, self.len
        )
    }
}

impl std::error::Error for SequenceExceedsBoundsError {}

macro_rules! impl_primitive_sequence_alloc {
    ($rust_type:ty, $init_func:ident, $fini_func:ident, $copy_func:ident, $equal_func:ident $(; native($copy_native:path, $equal_native:path))? $(, $extra:item)*) => {
        #[link(name = "rosidl_runtime_c")]
        unsafe extern "C" {
            fn $init_func(seq: *mut PrimitiveSequence<$rust_type>, size: usize) -> bool;
            fn $fini_func(seq: *mut PrimitiveSequence<$rust_type>);
            fn $copy_func(
                in_seq: *const PrimitiveSequence<$rust_type>,
                out_seq: *mut PrimitiveSequence<$rust_type>,
            ) -> bool;
            fn $equal_func(
                lhs: *const PrimitiveSequence<$rust_type>,
                rhs: *const PrimitiveSequence<$rust_type>,
            ) -> bool;
        }

        impl PrimitiveSequenceAlloc for $rust_type {
            $($extra)*
            fn primitive_sequence_init(seq: &mut PrimitiveSequence<Self>, size: usize) -> bool {
                // SAFETY: There are no special preconditions to the sequence_init function.
                unsafe {
                    // This allocates space and sets seq.size and seq.capacity to size
                    let ret = $init_func(seq as *mut _, size);
                    if ret && !seq.data.is_null() {
                        // Zero memory, since it will be uninitialized if there is no default value
                        std::ptr::write_bytes(seq.data, 0u8, size);
                    }
                    ret
                }
            }
            fn primitive_sequence_fini(seq: &mut PrimitiveSequence<Self>) {
                // SAFETY: There are no special preconditions to the sequence_fini function.
                unsafe { $fini_func(seq as *mut _) }
            }
            fn primitive_sequence_copy(
                in_seq: &PrimitiveSequence<Self>,
                out_seq: &mut PrimitiveSequence<Self>,
            ) -> bool {
                $(if let Some(result) = $copy_native(in_seq, out_seq) {
                    return result;
                })?
                // SAFETY: both sequences use the matching native element layout.
                unsafe { $copy_func(in_seq as *const _, out_seq as *mut _) }
            }
            fn primitive_sequence_are_equal(
                lhs: &PrimitiveSequence<Self>,
                rhs: &PrimitiveSequence<Self>,
            ) -> bool {
                $(if let Some(result) = $equal_native(lhs, rhs) {
                    return result;
                })?
                unsafe { $equal_func(lhs, rhs) }
            }
        }
    };
}

// Primitives are not messages themselves, but there can be sequences of them.
//
// See https://github.com/ros2/rosidl/blob/master/rosidl_runtime_c/include/rosidl_runtime_c/primitives_sequence.h
// Long double isn't available in Rust, so it is skipped.
impl_primitive_sequence_alloc!(
    f32,
    rosidl_runtime_c__float__Sequence__init,
    rosidl_runtime_c__float__Sequence__fini,
    rosidl_runtime_c__float__Sequence__copy,
    rosidl_runtime_c__float__Sequence__are_equal
);
impl_primitive_sequence_alloc!(
    f64,
    rosidl_runtime_c__double__Sequence__init,
    rosidl_runtime_c__double__Sequence__fini,
    rosidl_runtime_c__double__Sequence__copy,
    rosidl_runtime_c__double__Sequence__are_equal
);
impl_primitive_sequence_alloc!(
    bool,
    rosidl_runtime_c__boolean__Sequence__init,
    rosidl_runtime_c__boolean__Sequence__fini,
    rosidl_runtime_c__boolean__Sequence__copy,
    rosidl_runtime_c__boolean__Sequence__are_equal
);
impl_primitive_sequence_alloc!(
    u8,
    rosidl_runtime_c__uint8__Sequence__init,
    rosidl_runtime_c__uint8__Sequence__fini,
    rosidl_runtime_c__uint8__Sequence__copy,
    rosidl_runtime_c__uint8__Sequence__are_equal;
    native(crate::native::copy_sequence, crate::native::sequences_equal),
    fn primitive_sequence_to_vec(
        seq: &PrimitiveSequence<Self>,
    ) -> Result<Vec<Self>, crate::BufferError> {
        match seq.opaque_ptr() {
            Some(pointer) => crate::buffer::copy_to_host(pointer, seq.len()),
            None => Ok(seq.as_slice().to_vec()),
        }
    }
);
impl_primitive_sequence_alloc!(
    i8,
    rosidl_runtime_c__int8__Sequence__init,
    rosidl_runtime_c__int8__Sequence__fini,
    rosidl_runtime_c__int8__Sequence__copy,
    rosidl_runtime_c__int8__Sequence__are_equal
);
impl_primitive_sequence_alloc!(
    u16,
    rosidl_runtime_c__uint16__Sequence__init,
    rosidl_runtime_c__uint16__Sequence__fini,
    rosidl_runtime_c__uint16__Sequence__copy,
    rosidl_runtime_c__uint16__Sequence__are_equal
);
impl_primitive_sequence_alloc!(
    i16,
    rosidl_runtime_c__int16__Sequence__init,
    rosidl_runtime_c__int16__Sequence__fini,
    rosidl_runtime_c__int16__Sequence__copy,
    rosidl_runtime_c__int16__Sequence__are_equal
);
impl_primitive_sequence_alloc!(
    u32,
    rosidl_runtime_c__uint32__Sequence__init,
    rosidl_runtime_c__uint32__Sequence__fini,
    rosidl_runtime_c__uint32__Sequence__copy,
    rosidl_runtime_c__uint32__Sequence__are_equal
);
impl_primitive_sequence_alloc!(
    i32,
    rosidl_runtime_c__int32__Sequence__init,
    rosidl_runtime_c__int32__Sequence__fini,
    rosidl_runtime_c__int32__Sequence__copy,
    rosidl_runtime_c__int32__Sequence__are_equal
);
impl_primitive_sequence_alloc!(
    u64,
    rosidl_runtime_c__uint64__Sequence__init,
    rosidl_runtime_c__uint64__Sequence__fini,
    rosidl_runtime_c__uint64__Sequence__copy,
    rosidl_runtime_c__uint64__Sequence__are_equal
);
impl_primitive_sequence_alloc!(
    i64,
    rosidl_runtime_c__int64__Sequence__init,
    rosidl_runtime_c__int64__Sequence__fini,
    rosidl_runtime_c__int64__Sequence__copy,
    rosidl_runtime_c__int64__Sequence__are_equal
);

impl<T: PrimitiveSequenceAlloc, const N: usize> BoundedPrimitiveSequence<T, N> {
    /// Materializes opaque storage without changing the bound.
    pub fn try_into_cpu(self) -> Result<Self, crate::BufferError> {
        Ok(Self {
            inner: self.inner.try_into_cpu()?,
        })
    }

    /// Returns the underlying sequence without copying its storage.
    pub fn into_unbounded(self) -> PrimitiveSequence<T> {
        self.inner
    }

    /// Checks the bound without copying sequence storage.
    pub fn try_from_unbounded(
        inner: PrimitiveSequence<T>,
    ) -> Result<Self, SequenceExceedsBoundsError> {
        if inner.len() > N {
            return Err(SequenceExceedsBoundsError {
                len: inner.len(),
                upper_bound: N,
            });
        }
        Ok(Self { inner })
    }
}

#[cfg(test)]
mod tests {
    use quickcheck::{quickcheck, Arbitrary, Gen};

    use super::*;

    impl<T: Arbitrary + PrimitiveSequenceAlloc> Arbitrary for PrimitiveSequence<T> {
        fn arbitrary(g: &mut Gen) -> Self {
            Vec::arbitrary(g).into()
        }
    }

    impl<T: Arbitrary + PrimitiveSequenceAlloc> Arbitrary for BoundedPrimitiveSequence<T, 256> {
        fn arbitrary(g: &mut Gen) -> Self {
            let len = u8::arbitrary(g);
            (0..len).map(|_| T::arbitrary(g)).collect()
        }
    }

    quickcheck! {
        fn test_extend_native(xs: Vec<i32>, ys: Vec<i32>) -> bool {
            let mut xs_seq = PrimitiveSequence::new(xs.len());
            xs_seq.copy_from_slice(&xs);
            xs_seq.extend(ys.clone());
            if xs_seq.len() != xs.len() + ys.len() {
                return false;
            }
            if xs_seq[..xs.len()] != xs[..] {
                return false;
            }
            if xs_seq[xs.len()..] != ys[..] {
                return false;
            }
            true
        }
    }

    quickcheck! {
        fn test_iteration_native(xs: Vec<i32>) -> bool {
            let mut seq_1 = PrimitiveSequence::new(xs.len());
            seq_1.copy_from_slice(&xs);
            let seq_2 = seq_1.clone().into_iter().collect();
            seq_1 == seq_2
        }
    }

    quickcheck! {
        fn test_into_vec_primitive_quickcheck_native(xs: Vec<i32>) -> bool {
            let seq: PrimitiveSequence<i32> = PrimitiveSequence::from(&xs[..]);
            let ys: Vec<i32> = seq.into();
            xs == ys
        }
    }

    #[test]
    fn primitive_iterator_tracks_remaining_elements() {
        let mut iter = PrimitiveSequence::from(&[3i32, 5][..]).into_iter();
        assert_eq!(iter.len(), 2);
        assert_eq!(iter.next(), Some(3));
        assert_eq!(iter.size_hint(), (1, Some(1)));
        assert_eq!(iter.next(), Some(5));
        assert_eq!(iter.len(), 0);
        assert_eq!(iter.next(), None);
        assert_eq!(iter.next(), None);
    }

    #[test]
    fn bounded_primitive_sequences_enforce_limits() {
        type Bounded = BoundedPrimitiveSequence<u32, 2>;
        assert!(Bounded::try_new(3).is_err());
        assert!(Bounded::try_from(vec![1, 2, 3]).is_err());
        assert!(Bounded::try_from(&[1, 2, 3][..]).is_err());
        let mut sequence = Bounded::try_from(&[1, 2][..]).unwrap();
        sequence.as_mut_slice()[1] = 9;
        assert_eq!(sequence.len(), 2);
        assert!(!sequence.is_empty());
        assert_eq!(sequence.clone().into_iter().collect::<Vec<_>>(), vec![1, 9]);
        assert_eq!((0..4).collect::<Bounded>().as_slice(), &[0, 1]);
    }

    #[test]
    fn primitive_types_initialize_copy_and_compare() {
        macro_rules! check {
            ($($ty:ty),+ $(,)?) => {$(
                let sequence = PrimitiveSequence::<$ty>::new(3);
                assert_eq!(sequence.as_slice(), &[<$ty>::default(); 3]);
                assert_eq!(sequence, sequence.clone());
                assert_ne!(sequence, PrimitiveSequence::<$ty>::new(2));
            )+};
        }
        check!(bool, u8, i8, u16, i16, u32, i32, u64, i64, f32, f64);
        let nan = PrimitiveSequence::from(&[f32::NAN][..]);
        assert_ne!(nan, nan.clone());
    }
}
