//! FNV-1a (64-bit) over a value's bytes. Ports agg-sharp `VectorMath/LongHash.cs` and
//! `Vector4.ComputeHash`; every vector's `get_long_hash_code` folds its components through
//! [`of`]. Bytes are little-endian, as `BitConverter.GetBytes` produces on every platform
//! MatterCAD ships on, so hashes match the C# values.

const FNV64_PRIME: u64 = 0x100000001b3;

/// Folds the 8 little-endian bytes of `data` into `hash` (C# `LongHash.Of`).
pub fn of(data: f64, hash: u64) -> u64 {
    compute_hash(&data.to_le_bytes(), hash)
}

/// FNV-1a over `bytes`, starting from `hash` (C# `Vector4.ComputeHash`). C# `ulong` arithmetic
/// is unchecked, so the multiply wraps.
pub fn compute_hash(bytes: &[u8], hash: u64) -> u64 {
    let mut hash = hash;
    for &value in bytes {
        hash ^= u64::from(value);
        hash = hash.wrapping_mul(FNV64_PRIME);
    }
    hash
}
