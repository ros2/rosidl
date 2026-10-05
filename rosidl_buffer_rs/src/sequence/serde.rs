// Copyright 2026 Open Source Robotics Foundation, Inc.
// SPDX-License-Identifier: Apache-2.0

use serde::{de::Error, Deserialize, Deserializer, Serialize, Serializer};

use super::{BoundedPrimitiveSequence, PrimitiveSequence};
use crate::PrimitiveSequenceAlloc;

impl<'de, T: Deserialize<'de> + PrimitiveSequenceAlloc> Deserialize<'de> for PrimitiveSequence<T> {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        let values: Vec<_> = Deserialize::deserialize(deserializer)?;
        Ok(Self::from(values))
    }
}

impl<T: Serialize + PrimitiveSequenceAlloc> Serialize for PrimitiveSequence<T> {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        if self.is_rosidl_buffer() {
            return Err(serde::ser::Error::custom(
                "opaque buffers cannot be serialized as CPU sequences",
            ));
        }
        self.as_slice().serialize(serializer)
    }
}

impl<'de, T: Deserialize<'de> + PrimitiveSequenceAlloc, const N: usize> Deserialize<'de>
    for BoundedPrimitiveSequence<T, N>
{
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        let values: Vec<_> = Deserialize::deserialize(deserializer)?;
        Self::try_from(values).map_err(D::Error::custom)
    }
}

impl<T: Serialize + PrimitiveSequenceAlloc, const N: usize> Serialize
    for BoundedPrimitiveSequence<T, N>
{
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        self.inner.serialize(serializer)
    }
}

#[cfg(test)]
mod tests {
    use quickcheck::quickcheck;

    use crate::{BoundedPrimitiveSequence, PrimitiveSequence};

    quickcheck! {
        fn test_json_roundtrip_primitive_sequence(xs: PrimitiveSequence<i32>) -> bool {
            let value = serde_json::to_value(xs.clone()).unwrap();
            let recovered = serde_json::from_value(value).unwrap();
            xs == recovered
        }
    }

    quickcheck! {
        fn test_json_roundtrip_bounded_primitive_sequence(xs: BoundedPrimitiveSequence<i32, 256>) -> bool {
            let value = serde_json::to_value(xs.clone()).unwrap();
            let recovered = serde_json::from_value(value).unwrap();
            xs == recovered
        }
    }

    #[test]
    fn bounded_deserialization_rejects_overflow() {
        assert!(serde_json::from_str::<BoundedPrimitiveSequence<u32, 2>>("[1, 2, 3]").is_err());
        assert_eq!(
            serde_json::from_str::<BoundedPrimitiveSequence<u32, 2>>("[1, 2]")
                .unwrap()
                .as_slice(),
            &[1, 2]
        );
    }

    #[test]
    fn opaque_serialization_returns_an_error() {
        let sequence = PrimitiveSequence::<u8> {
            data: std::ptr::null_mut(),
            size: 0,
            capacity: 0,
            is_rosidl_buffer: true,
            owns_rosidl_buffer: false,
        };
        assert!(serde_json::to_string(&sequence)
            .unwrap_err()
            .to_string()
            .contains("opaque buffers"));
        let bounded = BoundedPrimitiveSequence::<u8, 4> { inner: sequence };
        assert!(serde_json::to_string(&bounded).is_err());
    }
}
