//! Rust-only checks of the bevel core's R1.P03 package (surface plan read-throughs, band ends at a
//! mitre, ring readers). The tests that exercise these methods read whole planned chains, so these
//! pin, method by method, the closed forms the code's comments state.

use solidmesh::bevel::surface::{
    surface_plan_band_ends, SurfaceBevelPlan, SurfaceMixedCornerDualGrid,
    SurfaceTwoCreaseJunctionRings,
};
use solidmesh_math::Vector3;

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
    use solidmesh::bevel::surface::surface_room_measure::offset_to_segment;

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

#[test]
fn rust_only_the_first_root_is_the_one_the_turn_reaches() {
    use solidmesh::bevel::surface::SurfaceMixedCornerWall;
    use std::f64::consts::PI;

    // cos t = 1/2 has roots at +-pi/3; from zero the turn reaches pi/3 first.
    let root = SurfaceMixedCornerWall::try_the_first_root(1.0, 0.0, 0.5, 0.0).unwrap();
    assert!((root - PI / 3.0).abs() < 1e-12);

    // Past pi/3 the next one reached is -pi/3 carried a whole turn on.
    let root = SurfaceMixedCornerWall::try_the_first_root(1.0, 0.0, 0.5, 1.5).unwrap();
    assert!((root - 5.0 * PI / 3.0).abs() < 1e-12);

    // A tangency is a double root, not a miss; a plane the curve never reaches is.
    assert!(SurfaceMixedCornerWall::try_the_first_root(1.0, 0.0, 1.0 + 1e-12, 0.0).is_some());
    assert!(SurfaceMixedCornerWall::try_the_first_root(1.0, 0.0, 1.1, 0.0).is_none());
}

#[test]
fn rust_only_a_missing_wall_section_has_no_sagitta() {
    use solidmesh::bevel::surface::surface_mixed_corner_cut_edge::sagitta_of_the_walls_chords;

    assert_eq!(sagitta_of_the_walls_chords(None), 0.0);
}

#[test]
fn rust_only_a_room_cut_keeps_its_defaults_and_is_told_its_plan() {
    use solidmesh::bevel::surface::{SurfaceRoomCut, SurfaceRoomFootprint};

    let cut = SurfaceRoomCut::new(
        7,
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(2.0, 0.0, 0.0),
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
        2.0,
        1.0,
        f64::INFINITY,
    );
    assert_eq!(cut.shape, SurfaceRoomFootprint::Band);
    assert_eq!((cut.plan_index, cut.edge_index, cut.side), (-1, -1, -1));
    assert_eq!(cut.corner_key, SurfaceRoomCut::NO_CORNER);
    assert!(cut.section_side_a.is_none() && cut.section_side_b.is_none());

    let on = SurfaceRoomCut {
        edge_index: 3,
        ..cut
    }
    .on_plan(4);
    assert_eq!((on.plan_index, on.edge_index, on.region_key), (4, 3, 7));
}

#[test]
fn rust_only_a_square_mitre_carries_the_outside_band_one_setback() {
    use solidmesh::bevel::surface::surface_room_mitre::overhang_per_setback;

    let into_the_edge = Vector3::new(-1.0, 0.0, 0.0);
    let past_the_corner = Vector3::new(0.0, 1.0, 0.0);

    // The band on the outside of a square turn runs a whole setback past the corner; the one on
    // the inside is cut back, which reads as no overhang.
    let outside =
        overhang_per_setback(into_the_edge, past_the_corner, Vector3::new(0.0, -1.0, 0.0));
    assert!((outside - 1.0).abs() < 1e-12);
    assert_eq!(
        overhang_per_setback(into_the_edge, past_the_corner, Vector3::new(0.0, 1.0, 0.0)),
        0.0
    );

    // Nothing carries on past a free end.
    assert_eq!(
        overhang_per_setback(into_the_edge, Vector3::ZERO, Vector3::new(0.0, -1.0, 0.0)),
        0.0
    );
}

#[test]
fn rust_only_a_chord_meets_the_touch_circle_within_its_own_span() {
    use solidmesh::bevel::surface::SurfaceRunOutMeetsAWallBand;

    // From inside a circle of radius 2 out to past it, along +x: the crossing is at x = 2.
    let at = SurfaceRunOutMeetsAWallBand::try_across(
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(3.0, 0.0, 0.0),
        2.0,
    )
    .unwrap();
    assert!((at - Vector3::new(2.0, 0.0, 0.0)).length() < 1e-12);

    // A chord wholly inside never reaches it.
    assert!(SurfaceRunOutMeetsAWallBand::try_across(
        Vector3::new(0.5, 0.0, 0.0),
        Vector3::new(1.5, 0.0, 0.0),
        2.0
    )
    .is_none());

    // The turn-back is where the turns peak, and an end rail counts against nothing beyond it.
    let turns = [0.1, 0.4, 0.3];
    assert!(SurfaceRunOutMeetsAWallBand::it_is_the_turn_back_of(
        &turns, 1
    ));
    assert!(!SurfaceRunOutMeetsAWallBand::it_is_the_turn_back_of(
        &turns, 0
    ));
    assert!(!SurfaceRunOutMeetsAWallBand::it_is_the_turn_back_of(
        &turns, 3
    ));
}

#[test]
fn rust_only_three_planes_meet_at_their_common_point() {
    use solidmesh::bevel::surface::SurfaceStripTrimmedByTheFaceBeyond;

    let at = SurfaceStripTrimmedByTheFaceBeyond::try_three_planes(
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(0.0, 2.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
        Vector3::new(0.0, 0.0, 3.0),
        Vector3::new(0.0, 0.0, 1.0),
    )
    .unwrap();
    assert!((at - Vector3::new(1.0, 2.0, 3.0)).length() < 1e-12);

    // Two parallel planes do not meet at a point.
    assert!(SurfaceStripTrimmedByTheFaceBeyond::try_three_planes(
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(0.0, 0.0, 1.0),
    )
    .is_none());
}
