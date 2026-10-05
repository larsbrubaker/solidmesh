//! Rust-only checks of the bevel core's R1.P03 package (surface plan read-throughs, band ends at a
//! mitre, ring readers). No C# test class is runnable on these methods alone - the classes that
//! exercise them read whole planned chains - so these pin the closed forms the C# remarks state.

use geometry_processing::bevel::surface::{
    surface_plan_band_ends, SurfaceBevelPlan, SurfaceMixedCornerDualGrid,
    SurfaceTwoCreaseJunctionRings,
};
use vector_math::Vector3;

#[test]
fn rust_only_mitre_normal_bisects_the_turn() {
    let n = surface_plan_band_ends::mitre_normal(
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
        Vector3::new(0.0, 0.0, 1.0),
    );
    let half = 0.5f64.sqrt();
    assert!((n - Vector3::new(half, half, 0.0)).length() < 1e-12);
}

#[test]
fn rust_only_mitre_normal_falls_back_square_where_the_chain_doubles_back() {
    let square = Vector3::new(0.0, 0.0, 1.0);
    let n = surface_plan_band_ends::mitre_normal(
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(-1.0, 0.0, 0.0),
        square,
    );
    assert_eq!(n, square);
}

#[test]
fn rust_only_band_ends_point_into_their_own_band() {
    let corner = Vector3::new(1.0, 2.0, 3.0);
    let arriving = Vector3::new(1.0, 0.0, 0.0);
    let here = Vector3::new(0.0, 1.0, 0.0);

    // At the corner an edge starts at, the normal leans the way the edge runs.
    let start =
        surface_plan_band_ends::band_end_where_the_chain_arrives_along(corner, arriving, here);
    assert_eq!(start.point, corner);
    assert!(start.inward.dot(here) > 0.0);

    // At the corner it ends at, the normal turns back along it.
    let end = surface_plan_band_ends::band_end_where_the_chain_leaves_along(corner, here, arriving);
    assert!(end.inward.dot(here) < 0.0);

    // The degenerate fallback is negated with the rest, so it too points into the band.
    let back = surface_plan_band_ends::band_end_where_the_chain_leaves_along(corner, here, -here);
    assert_eq!(back.inward, -here);
}

#[test]
fn rust_only_ring_readers_reverse_into_a_new_array() {
    assert_eq!(
        SurfaceMixedCornerDualGrid::reversed(&[1, 2, 3]),
        vec![3, 2, 1]
    );
    assert_eq!(
        SurfaceTwoCreaseJunctionRings::reversed::<i32>(&[]),
        Vec::<i32>::new()
    );
}

#[test]
fn rust_only_the_wedge_floors_are_one_degree_from_either_end() {
    assert_eq!(SurfaceBevelPlan::MINIMUM_WEDGE_ANGLE_DEGREES, 1.0);
    assert_eq!(SurfaceBevelPlan::MAXIMUM_WEDGE_ANGLE_DEGREES, 179.0);
}

#[test]
fn rust_only_offset_to_segment_reads_the_nearest_clipped_end() {
    use geometry_processing::bevel::surface::surface_room_measure::offset_to_segment;

    let from = Vector3::new(0.0, 0.0, 0.0);
    let along = Vector3::new(1.0, 0.0, 0.0);
    let inward = Vector3::new(0.0, 1.0, 0.0);

    // A rim segment leaning from 2 to 4 across a span of 10, overhanging both ends: clipped to the
    // span it is 2 + 2 * (0 - -5) / 20 = 2.5 at the near end.
    let offset = offset_to_segment(
        from,
        along,
        10.0,
        inward,
        Vector3::new(-5.0, 2.0, 0.0),
        Vector3::new(15.0, 4.0, 0.0),
    );
    assert!((offset - 2.5).abs() < 1e-12);

    // A segment ON the edge's line is the edge itself, not an offset.
    let on_the_line = offset_to_segment(
        from,
        along,
        10.0,
        inward,
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(10.0, 0.0, 0.0),
    );
    assert!(on_the_line.is_infinite());

    // One sharing no more than a corner's width of the span faces nothing.
    let past_the_end = offset_to_segment(
        from,
        along,
        10.0,
        inward,
        Vector3::new(10.0, 1.0, 0.0),
        Vector3::new(20.0, 1.0, 0.0),
    );
    assert!(past_the_end.is_infinite());
}
