//! Rust-only checks of the seventh bevel slice (`SurfaceRing`, `BevelCornerHandover`,
//! `BevelEdgeToolBuilder`, `BevelMixedBlend`) and of slice 8's `RoomCandidate`. The tests that
//! exercise them in context read whole bevels (the tool-plane accuracy tests through the group
//! tools, the mixed-corner tests through the surface builder); these pin each piece alone. The
//! void-wedge section has its own test file.

use std::collections::HashSet;

use geometry_processing::bevel::bevel_corner_handover::try_hand_over;
use geometry_processing::bevel::bevel_edge_tool_builder as builder;
use geometry_processing::bevel::surface::{surface_ring, SurfaceVertexPool};
use geometry_processing::bevel::{
    BevelAdvisoryCode, BevelEndCap, BevelMixedBlend, BevelMixedBlendSet,
};
use geometry_processing::topology::EdgeConvexity;
use indexmap::IndexMap;
use polygon_mesh::Mesh;
use vector_math::{Vector2, Vector3};

// ---------------------------------------------------------------- SurfaceRing

#[test]
fn rust_only_two_triangles_of_a_square_union_to_its_outline() {
    let rings = vec![vec![0, 1, 2], vec![0, 2, 3]];
    assert_eq!(
        surface_ring::try_boundary_of_the_union(&rings),
        Some(vec![0, 1, 2, 3])
    );
}

#[test]
fn rust_only_the_boundary_starts_at_the_first_rings_first_outline_corner() {
    // Vertex 5 is interior to nothing here, but the first ring lists 2 first.
    let rings = vec![vec![2, 0, 1], vec![2, 1, 5]];
    assert_eq!(
        surface_ring::try_boundary_of_the_union(&rings),
        Some(vec![2, 0, 1, 5])
    );
}

#[test]
fn rust_only_a_fold_a_hole_or_a_pinch_has_no_one_outline() {
    // The same directed edge twice: a fold.
    assert_eq!(
        surface_ring::try_boundary_of_the_union(&[vec![0, 1, 2], vec![0, 1, 3]]),
        None
    );
    // Two separate triangles: two loops.
    assert_eq!(
        surface_ring::try_boundary_of_the_union(&[vec![0, 1, 2], vec![3, 4, 5]]),
        None
    );
    // Two triangles touching at vertex 0 only: the outline forks there.
    assert_eq!(
        surface_ring::try_boundary_of_the_union(&[vec![0, 1, 2], vec![0, 3, 4]]),
        None
    );
    // A triangle and its reverse cancel completely.
    assert_eq!(
        surface_ring::try_boundary_of_the_union(&[vec![0, 1, 2], vec![2, 1, 0]]),
        None
    );
}

#[test]
fn rust_only_without_repeats_drops_a_point_repeated_after_itself_including_around_the_end() {
    assert_eq!(
        surface_ring::without_repeats(&[1, 1, 2, 3, 3, 1]),
        vec![2, 3, 1]
    );
    assert_eq!(surface_ring::without_repeats(&[4, 5, 6]), vec![4, 5, 6]);
}

#[test]
fn rust_only_knit_inserts_a_vertex_standing_on_another_pieces_edge_in_order() {
    let mut pool = SurfaceVertexPool::new(&[]);
    let a = pool.add(Vector3::new(0.0, 0.0, 0.0));
    let b = pool.add(Vector3::new(4.0, 0.0, 0.0));
    let c = pool.add(Vector3::new(2.0, 3.0, 0.0));
    let m1 = pool.add(Vector3::new(1.0, 0.0, 0.0));
    let m3 = pool.add(Vector3::new(3.0, 0.0, 0.0));
    let d = pool.add(Vector3::new(2.0, -3.0, 0.0));

    // The upper piece knows the long edge whole; the lower one was cut at 1 and 3.
    let pieces = vec![vec![a, b, c], vec![a, d, m3, b, m1]];
    let knitted = surface_ring::knit(pieces, &pool);

    assert_eq!(knitted[0], vec![a, m1, m3, b, c]);
    assert_eq!(knitted[1], vec![a, d, m3, b, m1]);
}

#[test]
fn rust_only_knit_leaves_a_single_piece_alone() {
    let pool = SurfaceVertexPool::new(&[]);
    let pieces = vec![vec![0, 1, 2]];
    assert_eq!(surface_ring::knit(pieces.clone(), &pool), pieces);
}

// ---------------------------------------------------------------- BevelCornerHandover

#[test]
fn rust_only_a_patched_corner_hands_over_square_through_the_ball_centre() {
    let mut patched = IndexMap::new();
    patched.insert(7, Vector3::new(1.0, 1.0, 1.0));
    let mut seams = IndexMap::new();
    seams.insert((7, 3), vec![Vector3::new(1.0, 0.0, 0.0)]);

    let outward = Vector3::new(1.0, 0.0, 0.0);
    let cap = try_hand_over(
        7,
        Vector3::new(2.0, 0.0, 0.0),
        outward,
        3,
        false,
        Some(&patched),
        None,
        None,
        None,
        Some(&seams),
    )
    .expect("the patch owns the corner");

    assert_eq!(cap.plane_normal, outward);
    assert_eq!(cap.plane_distance, 1.0);
    assert!(!cap.drives_through);
    assert_eq!(cap.seam_points, Some(vec![Vector3::new(1.0, 0.0, 0.0)]));
}

#[test]
fn rust_only_a_cutter_at_a_drive_through_corner_runs_past_the_ball_centre() {
    let mut patched = IndexMap::new();
    patched.insert(7, Vector3::new(1.0, 1.0, 1.0));
    let drive: HashSet<i32> = [7].into_iter().collect();

    let cap = try_hand_over(
        7,
        Vector3::new(2.0, 0.0, 0.0),
        Vector3::new(1.0, 0.0, 0.0),
        3,
        false,
        Some(&patched),
        None,
        None,
        Some(&drive),
        None,
    )
    .expect("the patch owns the corner");

    assert!(cap.drives_through);
    assert_eq!(cap.plane_distance, 1.0);
    assert!(cap.drive_through_distance > 1.0 && cap.drive_through_distance < 2.0);
}

#[test]
fn rust_only_a_blend_handover_is_turned_to_point_out_of_the_end() {
    let mut handovers = IndexMap::new();
    handovers.insert((4, 9), BevelEndCap::new(Vector3::new(-1.0, 0.0, 0.0), -5.0));

    let cap = try_hand_over(
        4,
        Vector3::new(5.0, 0.0, 0.0),
        Vector3::new(1.0, 0.0, 0.0),
        9,
        false,
        None,
        Some(&handovers),
        None,
        None,
        None,
    )
    .expect("the blend owns the corner");

    assert_eq!(cap, BevelEndCap::new(Vector3::new(1.0, 0.0, 0.0), 5.0));

    // A different edge at the same vertex is not this blend's handover.
    assert!(try_hand_over(
        4,
        Vector3::new(5.0, 0.0, 0.0),
        Vector3::new(1.0, 0.0, 0.0),
        8,
        false,
        None,
        Some(&handovers),
        None,
        None,
        None,
    )
    .is_none());
}

#[test]
fn rust_only_a_bead_caps_on_the_blends_face_and_an_edge_on_plane_says_nothing() {
    let mut capped = IndexMap::new();
    capped.insert((4, 9), Vector3::new(0.0, 0.0, 1.0));
    let position = Vector3::new(1.0, 2.0, 3.0);

    let cap = try_hand_over(
        4,
        position,
        Vector3::new(0.0, 0.0, -1.0),
        9,
        true,
        None,
        None,
        Some(&capped),
        None,
        None,
    )
    .expect("the blend caps the bead");
    assert_eq!(cap, BevelEndCap::new(Vector3::new(0.0, 0.0, -1.0), -3.0));

    // Square to the run's own direction: no sense to choose, so the caller decides.
    assert!(try_hand_over(
        4,
        position,
        Vector3::new(1.0, 0.0, 0.0),
        9,
        true,
        None,
        None,
        Some(&capped),
        None,
        None,
    )
    .is_none());
}

// ---------------------------------------------------------------- BevelEdgeToolBuilder

/// A convex box edge along z with faces facing +x and +y.
fn box_edge_frame() -> (Vector3, Vector3, Vector3, Vector3, Vector3) {
    let (n0, n1) = builder::try_get_profile_normals(
        EdgeConvexity::Convex,
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
    )
    .unwrap();
    let (x, y, z) = builder::try_build_frame(n0, n1, Vector3::new(0.0, 0.0, 1.0)).unwrap();
    (n0, n1, x, y, z)
}

#[test]
fn rust_only_a_right_angle_chamfer_section_is_five_points_reaching_root_two_sizes() {
    let (n0, n1, x, y, z) = box_edge_frame();
    let (points, tangent) =
        builder::try_build_cross_section(n0, n1, x, y, z, 2.0, 1, false, 0.0).unwrap();

    assert_eq!(points.len(), 5);
    assert_eq!(tangent, 2.0);
    let reach = points.iter().map(Vector2::length).fold(0.0, f64::max);
    assert!((reach - builder::cross_section_reach(2.0, 90.0)).abs() < 1e-12);
    assert!((reach - 2.0 * 2.0_f64.sqrt()).abs() < 1e-12);
}

#[test]
fn rust_only_a_right_angle_fillet_section_lays_its_arc_on_the_rolling_ball() {
    let (n0, n1, x, y, z) = box_edge_frame();
    let (points, tangent) =
        builder::try_build_cross_section(n0, n1, x, y, z, 2.0, 8, true, 0.0).unwrap();

    assert_eq!(points.len(), 5 + 7);
    assert!((tangent - 2.0).abs() < 1e-12);
    let centre = (points[0] + points[4]).get_normal() * (2.0 / std::f64::consts::FRAC_PI_4.sin());
    for point in &points[5..] {
        assert!(((*point - centre).length() - 2.0).abs() < 1e-12);
    }
}

#[test]
fn rust_only_the_ball_section_stands_a_quarter_sagitta_off_the_ball() {
    let (n0, n1, x, y, z) = box_edge_frame();
    let (points, _) = builder::try_build_ball_cross_section(n0, n1, x, y, z, 2.0, 4, true).unwrap();

    assert_eq!(points.len(), 16);
    let expected = 2.0 + 0.25 * 2.0 * (1.0 - (std::f64::consts::PI / 16.0).cos());
    let centre = Vector2::new(0.0, -2.0 * 2.0_f64.sqrt());
    for point in &points {
        assert!(((*point - centre).length() - expected).abs() < 1e-12);
    }

    let refused = builder::try_build_ball_cross_section(n0, n1, x, y, z, 2.0, 4, false);
    assert_eq!(
        refused.unwrap_err().code(),
        BevelAdvisoryCode::ChamferHasNoBall
    );
}

#[test]
fn rust_only_the_builder_refuses_what_has_no_corner() {
    assert_eq!(
        builder::try_get_profile_normals(EdgeConvexity::Flat, Vector3::UNIT_X, Vector3::UNIT_Y)
            .unwrap_err()
            .code(),
        BevelAdvisoryCode::FacesCoplanar
    );
    assert_eq!(
        builder::try_build_frame(Vector3::UNIT_X, Vector3::UNIT_X, Vector3::UNIT_X)
            .unwrap_err()
            .code(),
        BevelAdvisoryCode::FacesFoldBack
    );
    assert_eq!(
        builder::try_validate_size(0.0).unwrap_err().code(),
        BevelAdvisoryCode::SizeRemovesNothing
    );
    assert!(builder::try_validate_size(0.5).is_ok());

    // Two faces a hair from flat: the wedge is past the maximum interior angle.
    let tilt = 0.2_f64.to_radians();
    let n0 = Vector3::new(tilt.sin(), tilt.cos(), 0.0);
    let n1 = Vector3::new(-tilt.sin(), tilt.cos(), 0.0);
    let (x, y, z) = builder::try_build_frame(n0, n1, Vector3::UNIT_Z).unwrap();
    let refused = builder::try_build_cross_section(n0, n1, x, y, z, 1.0, 4, true, 0.0);
    assert_eq!(
        refused.unwrap_err().code(),
        BevelAdvisoryCode::FaceAngleTooFlat
    );
}

#[test]
fn rust_only_the_closed_forms_read_the_size_and_the_wedge() {
    assert_eq!(builder::overshoot(2.0, 3.0), 3.0);
    assert_eq!(builder::overshoot(4.0, 3.0), 4.0);
    assert_eq!(builder::tangent_distance(2.0, 60.0, false), 2.0);
    assert!(
        (builder::tangent_distance(2.0, 60.0, true) - 2.0 / 30.0_f64.to_radians().tan()).abs()
            < 1e-12
    );
    assert_eq!(builder::tangent_distance(2.0, 0.0, true), f64::INFINITY);
    assert_eq!(builder::cross_section_reach(1.0, 0.0), f64::INFINITY);
    assert!(
        (builder::cross_section_reach(1.0, 30.0) - 1.0 / 15.0_f64.to_radians().sin()).abs() < 1e-12
    );
}

// ---------------------------------------------------------------- BevelMixedBlend

#[test]
fn rust_only_a_blend_set_hands_back_what_it_was_built_with() {
    let blend = BevelMixedBlend::new(Mesh::new(), vec![0, 2], 1.5, 1.0, 1.75);
    let mut handovers = IndexMap::new();
    handovers.insert((3, 5), BevelEndCap::new(Vector3::UNIT_X, 1.0));
    let mut normals = IndexMap::new();
    normals.insert((3, 6), Vector3::UNIT_Z);

    let set = BevelMixedBlendSet::new(vec![blend], handovers, normals, vec![None, Some(vec![])]);

    assert_eq!(set.blends()[0].groups(), &[0, 2]);
    assert_eq!(set.blends()[0].cap_face_reach(), 1.5);
    assert_eq!(set.blends()[0].void_face_reach(), 1.0);
    assert_eq!(set.blends()[0].runout_reach(), 1.75);
    assert!(set.convex_handovers().contains_key(&(3, 5)));
    assert_eq!(set.concave_cap_normals()[&(3, 6)], Vector3::UNIT_Z);
    assert!(set.warnings()[0].is_none());
}

// ---------------------------------------------------------------- RoomCandidate (slice 8)

#[test]
fn rust_only_a_room_candidate_takes_the_record_defaults() {
    use geometry_processing::bevel::level_line::{RoomCandidate, RoomSource};

    let candidate = RoomCandidate::new(RoomSource::foreign(3), 1.25);
    assert_eq!(candidate.source, RoomSource::foreign(3));
    assert_eq!(candidate.room, 1.25);
    assert!(candidate.room_without_the_other.is_nan());
    assert_eq!(candidate.room_without_the_other_at_most, f64::INFINITY);
    assert!(candidate.group_members.is_none());
}
