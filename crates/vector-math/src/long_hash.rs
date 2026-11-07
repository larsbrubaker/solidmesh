//! FNV-1a (64-bit) over a value's bytes; every vector's `get_long_hash_code` folds its
//! components through [`of`]. Bytes are always taken little-endian, so a value hashes the same
//! on every platform.

const FNV64_PRIME: u64 = 0x100000001b3;

/// Folds the 8 little-endian bytes of `data` into `hash`.
pub fn of(data: f64, hash: u64) -> u64 {
    compute_hash(&data.to_le_bytes(), hash)
}

/// FNV-1a over `bytes`, starting from `hash`. FNV is defined modulo 2^64, so the multiply
/// wraps.
pub fn compute_hash(bytes: &[u8], hash: u64) -> u64 {
    let mut hash = hash;
    for &value in bytes {
        hash ^= u64::from(value);
        hash = hash.wrapping_mul(FNV64_PRIME);
    }
    hash
}
