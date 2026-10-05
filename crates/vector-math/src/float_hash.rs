//! Bit-based hashing of floats, the Rust side of the C# `GetHashCode` overrides on the vector,
//! matrix and quaternion types (agg-sharp builds them from an anonymous type over the
//! components). Every `std::hash::Hash` impl in this crate folds its components through these.
//!
//! C# `double.GetHashCode()` makes 0.0 and -0.0 hash alike (they compare equal), so the zero
//! bit patterns are folded together here too; NaN hashes by its bits as-is. The types implement
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
