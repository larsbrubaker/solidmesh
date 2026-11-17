//! The two-way surface deviation (`mesh_distance::surface_deviation`) on shapes whose answer is
//! known: a box against the same box, and against a box shrunk by a tenth all round.

use solidmesh_mesh::mesh_distance::{surface_deviation, DEFAULT_SAMPLES_PER_SURFACE};
use solidmesh_mesh::solids::platonic_solids::create_cube;
use solidmesh_mesh::Mesh;

/// Vertices are stored as `f32` and distances come from squared distances, so they are exact
/// to about this.
const TOLERANCE: f64 = 1e-6;

#[test]
fn a_mesh_is_zero_from_itself() {
    let cube = create_cube(2.0, 2.0, 2.0);
    let d = surface_deviation(&cube, &cube, 10_000, 0.99);
    // A sample inside a face lands a rounding error off it, and the distance is the square
    // root of a squared distance, so zero comes out as a few 1e-8.
    assert!(d.max < TOLERANCE, "max {}", d.max);
    assert!(d.percentile_distance < TOLERANCE);
    assert_eq!(d.scale, 2.0);
}

#[test]
fn a_box_shrunk_by_a_tenth_strays_a_tenth_and_its_corners_a_tenth_root_three() {
    let outer = create_cube(2.0, 2.0, 2.0);
    let inner = create_cube(1.8, 1.8, 1.8);
    let d = surface_deviation(&outer, &inner, 20_000, 0.5);
    // The outer box's corners are the farthest points from the inner box.
    assert!(
        (d.max - 0.1 * 3f64.sqrt()).abs() < TOLERANCE,
        "max {} should be the corner gap 0.1 * sqrt(3)",
        d.max
    );
    // Most of either surface faces the other across the 0.1 gap.
    assert!(
        (d.percentile_distance - 0.1).abs() < TOLERANCE,
        "median {} should be the face gap 0.1",
        d.percentile_distance
    );
    assert!((d.max_fraction() - 0.05 * 3f64.sqrt()).abs() < TOLERANCE);
    assert!((d.percentile_fraction() - 0.05).abs() < TOLERANCE);
    // Measured either way round, the largest distance is the same.
    let back = surface_deviation(&inner, &outer, 20_000, 0.5);
    assert!((back.max - d.max).abs() < TOLERANCE);
}

#[test]
fn the_measure_repeats_to_the_bit() {
    let outer = create_cube(2.0, 3.0, 4.0);
    let inner = create_cube(1.9, 2.7, 3.95);
    let first = surface_deviation(&outer, &inner, DEFAULT_SAMPLES_PER_SURFACE, 0.99);
    let second = surface_deviation(&outer, &inner, DEFAULT_SAMPLES_PER_SURFACE, 0.99);
    assert_eq!(first.max.to_bits(), second.max.to_bits());
    assert_eq!(
        first.percentile_distance.to_bits(),
        second.percentile_distance.to_bits()
    );
}

#[test]
fn an_empty_mesh_is_infinitely_far_from_a_solid_and_zero_from_another_empty_one() {
    let cube = create_cube(1.0, 1.0, 1.0);
    let empty = Mesh::new();
    assert_eq!(
        surface_deviation(&cube, &empty, 1000, 0.99).max,
        f64::INFINITY
    );
    assert_eq!(surface_deviation(&empty, &empty, 1000, 0.99).max, 0.0);
}
