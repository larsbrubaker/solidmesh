//! A Morton (Z-order) encoder with 21 bits per channel for x, y, z data (agg-sharp
//! `VectorMath/Bvh/MortonCodes.cs`).

use crate::Vector3;

/// The number of cells per axis, 2^21 (C# `MortonCodes.Size`).
pub const SIZE: i64 = 1 << 21;

// method to seperate the bits from a position
fn split_by_3(position: u32) -> i64 {
    let mut x: i64 = i64::from(position & 0x1fffff); // we only look at the first 21 bits
    x = (x | x << 32) & 0x1f00000000ffff; // shift left 32 bits, OR with self, and 00011111000000000000000000000000000000001111111111111111
    x = (x | x << 16) & 0x1f0000ff0000ff; // shift left 32 bits, OR with self, and 00011111000000000000000011111111000000000000000011111111
    x = (x | x << 8) & 0x100f00f00f00f00f; // shift left 32 bits, OR with self, and 0001000000001111000000001111000000001111000000001111000000000000
    x = (x | x << 4) & 0x10c30c30c30c30c3; // shift left 32 bits, OR with self, and 0001000011000011000011000011000011000011000011000011000100000000
    x = (x | x << 2) & 0x1249249249249249;
    x
}

/// The code of a position already scaled to [0, 2^21) per axis. Components convert as C#
/// `(uint)double` does on .NET 9+ (truncate, saturate, NaN to 0); the C# DEBUG-only range
/// check is not part of the Release behavior.
pub fn encode3_position(position: Vector3) -> i64 {
    encode3(position.x as u32, position.y as u32, position.z as u32)
}

/// The interleaved code of three 21-bit cells.
pub fn encode3(x: u32, y: u32, z: u32) -> i64 {
    split_by_3(x) | split_by_3(y) << 1 | split_by_3(z) << 2
}
