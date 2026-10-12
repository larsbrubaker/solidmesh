//! Port of the fixture builders of `LevelLine/CreaseDiesOutTests.cs` (`SwollenRidgeBlock` and
//! `Chain`): a block whose top ridge swells from nearly flat at the caps to a real crease in the
//! middle, so the picked crease dies out into the flat top at both ends.

#![allow(dead_code)]

use vector_math::{Vector2, Vector3};

use crate::level_line_feature_edges::feature_edges_over;

/// The bend threshold the kernel is handed.
pub const FOLD_DEGREES: f64 = 10.0;

/// The cross-section's point count.
pub const RING: i32 = 5;

/// How many slices the block is cut into along x.
pub const SLICES: i32 = 24;

/// The ridge's height in the middle of the block.
pub const PEAK_RIDGE: f64 = 0.70;

/// The ridge's height at each cap.
pub const FLAT_RIDGE: f64 = 0.01;

/// C# `Chain`: the pick - the maximal run of FEATURE edges along the ridge, read off the geometry
/// at the same threshold the kernel is handed. C#'s defaults are stride `RING` and column 4.
pub fn chain(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    stride: i32,
    column: i32,
) -> Vec<i32> {
    let feature = feature_edges_over(positions, indices, FOLD_DEGREES);
    let ridge: Vec<i32> = (0..=SLICES).map(|s| (s * stride) + column).collect();
    let mut run: Vec<i32> = Vec::new();
    let mut best: Vec<i32> = Vec::new();
    let mut i = 0;
    while i + 1 < ridge.len() {
        if feature(ridge[i], ridge[i + 1]) {
            if run.is_empty() {
                run.push(ridge[i]);
            }

            run.push(ridge[i + 1]);
        } else {
            if run.len() > best.len() {
                best = run;
            }

            run = Vec::new();
        }

        i += 1;
    }

    if run.len() > best.len() {
        run
    } else {
        best
    }
}

/// C# `SwollenRidgeBlock`: a block whose top carries a ridge that SWELLS - a raised cosine from
/// `FLAT_RIDGE` at each cap to `PEAK_RIDGE` in the middle - so the crease is a feature only over
/// the middle of the length and dies out into the flat top at both ends.
pub fn swollen_ridge_block() -> (Vec<Vector3>, Vec<(i32, i32, i32)>) {
    let mut positions = Vec::new();
    for slice in 0..=SLICES {
        let x = (-12 + slice) as f64;
        let half = SLICES as f64 / 2.0;
        let swell = 0.5 * (1.0 + (std::f64::consts::PI * (slice as f64 - half) / half).cos());
        let ridge = FLAT_RIDGE + ((PEAK_RIDGE - FLAT_RIDGE) * swell);

        // The cross-section in (y, z), counter-clockwise seen down -x, from the top-front corner
        // round the front and the bottom and back over the ridge.
        let section = [
            Vector2::new(-5.0, 5.0),
            Vector2::new(-5.0, -5.0),
            Vector2::new(5.0, -5.0),
            Vector2::new(5.0, 5.0),
            Vector2::new(0.0, 5.0 + ridge),
        ];

        for point in section {
            positions.push(Vector3::new(x, point.x, point.y));
        }
    }

    let mut indices = Vec::new();
    for slice in 0..SLICES {
        let a = slice * RING;
        let b = (slice + 1) * RING;
        for i in 0..RING {
            let next = (i + 1) % RING;
            indices.push((a + i, b + i, b + next));
            indices.push((a + i, b + next, a + next));
        }
    }

    // The two end caps, fanned from the section's first point.
    let last = SLICES * RING;
    for i in 1..RING - 1 {
        indices.push((0, i, i + 1));
        indices.push((last, last + i + 1, last + i));
    }

    // The section above is wound so the strip comes out inside-out; the block is turned the right
    // way once, here, so the fixture is a solid of POSITIVE volume like every other fixture.
    (
        positions,
        indices.into_iter().map(|t| (t.0, t.2, t.1)).collect(),
    )
}
