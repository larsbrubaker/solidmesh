//! The refinement that splits the band's triangles until the level line runs through them as
//! chords. Ports `Bevel/LevelLine/BandRefinement.cs` (a C# static class): so far its depth bound,
//! a triangle's span and the meter's nearness readings; the refinement lands with later packages.

use vector_math::{cs_math, Vector3};

/// How many times one triangle may be halved before the refinement gives up rather than loop.
/// This bounds the LIPSCHITZ halving only: the asking refinement - every field that answers as a
/// chain distance field, which is every real build - never reads it, and its only bound is the
/// deterministic work budget. Do not reason from 24 generations as if it limited a real build.
pub const DEFAULT_MAX_DEPTH: i32 = 24;

/// The longest side of a triangle - what a bounded search over it has to halve away.
pub fn span(a: Vector3, b: Vector3, c: Vector3) -> f64 {
    cs_math::max_f64(
        (a - b).length(),
        cs_math::max_f64((b - c).length(), (c - a).length()),
    )
}

/// How many SIZES from the level line the NEAREST of two corners stands - the least absolute field
/// value over the corners, against the size, which is twenty chord tolerances. A meter only.
pub fn nearest_to_the_line(chord_tolerance: f64, a: f64, b: f64) -> f64 {
    cs_math::min_f64(a.abs(), b.abs()) / (20.0 * chord_tolerance)
}

/// [`nearest_to_the_line`] over three corners.
pub fn nearest_to_the_line3(chord_tolerance: f64, a: f64, b: f64, c: f64) -> f64 {
    cs_math::min_f64(a.abs(), cs_math::min_f64(b.abs(), c.abs())) / (20.0 * chord_tolerance)
}

/// How many sizes from the line the FARTHEST of two corners stands.
pub fn farthest_from_the_line(chord_tolerance: f64, a: f64, b: f64) -> f64 {
    cs_math::max_f64(a.abs(), b.abs()) / (20.0 * chord_tolerance)
}

/// [`farthest_from_the_line`] over three corners.
pub fn farthest_from_the_line3(chord_tolerance: f64, a: f64, b: f64, c: f64) -> f64 {
    cs_math::max_f64(a.abs(), cs_math::max_f64(b.abs(), c.abs())) / (20.0 * chord_tolerance)
}
