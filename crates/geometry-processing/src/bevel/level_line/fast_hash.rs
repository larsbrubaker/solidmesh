// A multiply-and-rotate hasher for the small integer keys the level line files edges under (the
// FxHash scheme), in place of std's SipHash. C#'s `Dictionary<(int, int), ...>` hashes a pair of
// ints with a couple of arithmetic operations; SipHash costs several times that and its DoS
// resistance means nothing for keys the kernel makes itself. Used only with `IndexMap`, whose
// iteration order is insertion order whatever the hash, so no output can depend on the hasher.

use std::hash::{BuildHasherDefault, Hasher};

/// The FxHash state: one word, folded with a rotate, an xor and a multiply per write.
#[derive(Debug, Clone, Copy, Default)]
pub struct FastHasher {
    hash: u64,
}

/// The FxHash multiplier (an odd constant with well-mixed bits).
const SEED: u64 = 0x51_7c_c1_b7_27_22_0a_95;

impl FastHasher {
    fn fold(&mut self, word: u64) {
        self.hash = (self.hash.rotate_left(5) ^ word).wrapping_mul(SEED);
    }
}

impl Hasher for FastHasher {
    fn write(&mut self, bytes: &[u8]) {
        for chunk in bytes.chunks(8) {
            let mut word = [0u8; 8];
            word[..chunk.len()].copy_from_slice(chunk);
            self.fold(u64::from_le_bytes(word));
        }
    }

    fn write_u8(&mut self, value: u8) {
        self.fold(u64::from(value));
    }

    fn write_u32(&mut self, value: u32) {
        self.fold(u64::from(value));
    }

    fn write_i32(&mut self, value: i32) {
        self.fold(u64::from(value as u32));
    }

    fn write_u64(&mut self, value: u64) {
        self.fold(value);
    }

    fn write_usize(&mut self, value: usize) {
        self.fold(value as u64);
    }

    fn finish(&self) -> u64 {
        self.hash
    }
}

/// The `BuildHasher` for [`FastHasher`], for `IndexMap<K, V, FastBuildHasher>`.
pub type FastBuildHasher = BuildHasherDefault<FastHasher>;
