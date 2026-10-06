//! Port of the test fixture `LevelLine/OpenChainFixtures.cs`: the stage 2 open-chain fixture, a
//! faceted cylinder (40 sides, radius 10) whose top rim is a feature crease that carries on past a
//! partial pick, and whose wall facet edges are NOT features (their bend is under the threshold).

#![allow(dead_code)]

use vector_math::{Matrix4X4, Vector3};

use crate::level_line_test_geometry;

/// The bend threshold that makes the rim a feature and leaves the 9-degree wall facets plain.
pub const FOLD_DEGREES: f64 = 10.0;

pub const SIDES: i32 = 40;

pub const RADIUS: f64 = 10.0;

pub const HEIGHT: f64 = 20.0;

/// The length of one top-rim facet edge: 1.5643 at 40 sides and radius 10.
pub fn facet_length() -> f64 {
    2.0 * RADIUS * (std::f64::consts::PI / SIDES as f64).sin()
}

/// C# `Cylinder()`.
pub fn cylinder() -> (Vec<Vector3>, Vec<(i32, i32, i32)>) {
    cylinder_of_height(HEIGHT)
}

/// C# `Cylinder(double height)`: the same cylinder at another height; the vertex numbering is
/// unchanged, so [`top_rim_run`] names the same rim either way.
pub fn cylinder_of_height(height: f64) -> (Vec<Vector3>, Vec<(i32, i32, i32)>) {
    level_line_test_geometry::cylinder_solid(SIDES, RADIUS, height)
}

/// C# `TopRimRun`: an OPEN chain of `edges` consecutive top-rim edges (vertex `SIDES + i` in
/// increasing i, counter-clockwise seen from +z).
pub fn top_rim_run(edges: i32) -> Vec<i32> {
    (SIDES..SIDES + edges + 1).collect()
}

/// C# `Moved`: the cylinder moved by `by`.
pub fn moved(by: Matrix4X4) -> (Vec<Vector3>, Vec<(i32, i32, i32)>) {
    let (positions, indices) = cylinder();
    (
        positions.into_iter().map(|p| p.transform(by)).collect(),
        indices,
    )
}
