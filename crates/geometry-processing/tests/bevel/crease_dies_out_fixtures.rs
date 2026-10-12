//! Port of the fixture builders of `LevelLine/CreaseDiesOutTests.cs` (`SwollenRidgeBlock`,
//! `SwollenRidgeBlockWithASeamCopy`, `Chain`, `Neighbours`, `Build` and `SeamEpsilon`): a block
//! whose top ridge swells from nearly flat at the caps to a real crease in the middle, so the
//! picked crease dies out into the flat top at both ends. Read by `crease_dies_out_tests.rs` and
//! `band_side_end_tag_tests.rs`.

#![allow(dead_code)]

use geometry_processing::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use geometry_processing::bevel::level_line::{BandGapResult, BandProfile};
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

/// The seam fixture's cross-section point count - the ridge carries one extra point.
pub const SEAM_RING: i32 = 6;

/// How wide the doubled crease's ribbon is: eight microns, under the seam epsilon here.
pub const SEAM_RIBBON: f64 = 0.008;

/// C# `SeamEpsilon`: the app's seam epsilon by the app's own rule - 5e-4 of the bounding-box
/// diagonal.
pub fn seam_epsilon() -> f64 {
    let (positions, _) = swollen_ridge_block();
    let mut low = positions[0];
    let mut high = positions[0];
    for &point in &positions {
        low = Vector3::component_min(low, point);
        high = Vector3::component_max(high, point);
    }

    5e-4 * (high - low).length()
}

/// C# `Build`: the whole picked run at a size, fillet profile, at the app's seam epsilon.
pub fn build(positions: &[Vector3], indices: &[(i32, i32, i32)], size: f64) -> BandGapResult {
    build_chain(
        positions,
        indices,
        &chain(positions, indices, RING, 4),
        size,
    )
}

/// The same build for any chain - C#'s `BandGap.Build` call the mixed-end and seam tests make.
pub fn build_chain(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    chain: &[i32],
    size: f64,
) -> BandGapResult {
    let options = BandGapBuildOptions {
        seam_epsilon: seam_epsilon(),
        ..BandGapBuildOptions::default()
    };
    band_gap_entry_points::build(
        positions,
        indices,
        chain,
        false,
        Some(feature_edges_over(positions, indices, FOLD_DEGREES)),
        size,
        BandProfile::Fillet,
        &options,
    )
    .expect("the pick is well formed")
}

/// C# `Neighbours`: every vertex joined to this one by a triangle edge (a `HashSet` in C#;
/// first-seen order here).
pub fn neighbours(indices: &[(i32, i32, i32)], vertex: i32) -> Vec<i32> {
    let mut found = Vec::new();
    for &(ta, tb, tc) in indices {
        for (a, b) in [(ta, tb), (tb, tc), (tc, ta)] {
            let other = if a == vertex {
                Some(b)
            } else if b == vertex {
                Some(a)
            } else {
                None
            };
            if let Some(other) = other {
                if !found.contains(&other) {
                    found.push(other);
                }
            }
        }
    }

    found
}

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

/// C# `SwollenRidgeBlockWithASeamCopy`: the same block with its crease DOUBLED - the ridge carries
/// a second point a ribbon's width along the top, so the picked crease has a copy of itself beside
/// it sharing both end vertices, as on the real part. The ribbon lies ALONG the left face (a flat
/// ribbon across the top would split the crease's fold in two and neither half would be a
/// feature) and is narrower than the app's seam epsilon, so the copy reads as a seam and the flood
/// crosses it.
pub fn swollen_ridge_block_with_a_seam_copy() -> (Vec<Vector3>, Vec<(i32, i32, i32)>) {
    let mut positions = Vec::new();
    for slice in 0..=SLICES {
        let x = (-12 + slice) as f64;
        let half = SLICES as f64 / 2.0;
        let swell = 0.5 * (1.0 + (std::f64::consts::PI * (slice as f64 - half) / half).cos());
        let ridge = FLAT_RIDGE + ((PEAK_RIDGE - FLAT_RIDGE) * swell);
        let section = [
            Vector2::new(-5.0, 5.0),
            Vector2::new(-5.0, -5.0),
            Vector2::new(5.0, -5.0),
            Vector2::new(5.0, 5.0),
            Vector2::new(0.0, 5.0 + ridge),
            Vector2::new(-SEAM_RIBBON, 5.0 + ridge - (SEAM_RIBBON * ridge / 5.0)),
        ];

        for point in section {
            positions.push(Vector3::new(x, point.x, point.y));
        }
    }

    let mut indices = Vec::new();
    for slice in 0..SLICES {
        let a = slice * SEAM_RING;
        let b = (slice + 1) * SEAM_RING;
        for i in 0..SEAM_RING {
            let next = (i + 1) % SEAM_RING;
            indices.push((a + i, b + i, b + next));
            indices.push((a + i, b + next, a + next));
        }
    }

    let last = SLICES * SEAM_RING;
    for i in 1..SEAM_RING - 1 {
        indices.push((0, i, i + 1));
        indices.push((last, last + i + 1, last + i));
    }

    (
        positions,
        indices.into_iter().map(|t| (t.0, t.2, t.1)).collect(),
    )
}
