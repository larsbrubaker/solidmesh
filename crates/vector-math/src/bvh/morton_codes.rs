//! A Morton (Z-order) encoder with 21 bits per channel for x, y, z data.

use crate::Vector3;

/// The number of cells per axis, 2^21.
pub const SIZE: i64 = 1 << 21;

/// The extent the box centers of joint bounds `size` are scaled by to fill the code grid: the
/// bounds or a unit box, whichever is the longer vector, with a little room so the far edge
/// stays inside the grid.
///
/// An axis with no extent keeps a scale of 1. Dividing the grid by a zero extent is infinite, and
/// the scale matrix built from it holds NaN in every cell, so every center of a flat mesh would
/// get the code 0 and no tree could tell them apart. Every other axis keeps exactly the scale it
/// had, so trees over input with extent on each axis do not change.
pub fn center_scale(size: Vector3) -> Vector3 {
    let scale = Vector3::max(size, Vector3::ONE) * 1.001;
    let usable = |extent: f64| {
        if (SIZE as f64 / extent).is_finite() {
            extent
        } else {
            1.0
        }
    };
    Vector3::new(usable(scale.x), usable(scale.y), usable(scale.z))
}

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

/// The code of a position already scaled to [0, 2^21) per axis. Components convert with `as
/// u32` (truncate, saturate, NaN to 0), so an out-of-range position still gets a code instead
/// of failing.
pub fn encode3_position(position: Vector3) -> i64 {
    encode3(position.x as u32, position.y as u32, position.z as u32)
}

/// The interleaved code of three 21-bit cells.
pub fn encode3(x: u32, y: u32, z: u32) -> i64 {
    split_by_3(x) | split_by_3(y) << 1 | split_by_3(z) << 2
}
