//! Bit-based hashing of floats for the vector, matrix and quaternion types. Every
//! `std::hash::Hash` impl in this crate folds its components through these.
//!
//! 0.0 and -0.0 compare equal, so they must hash alike: the zero bit patterns are folded
//! together; NaN hashes by its bits as-is. The types implement
//! `PartialEq` only, not `Eq` (a NaN component is never equal to itself), so they can be hashed
//! but are not meant as `HashMap` keys unless the caller rules NaN out.

use std::hash::Hasher;

/// Feeds `value` to `state`, with -0.0 hashed as 0.0.
pub fn hash_f64<H: Hasher>(value: f64, state: &mut H) {
    let bits = if value == 0.0 { 0 } else { value.to_bits() };
    state.write_u64(bits);
}

/// Feeds `value` to `state`, with -0.0 hashed as 0.0.
pub fn hash_f32<H: Hasher>(value: f32, state: &mut H) {
    let bits = if value == 0.0 { 0 } else { value.to_bits() };
    state.write_u32(bits);
}
