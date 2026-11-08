//! Rust-only checks of the seventh bevel slice (`SurfaceRing`, `BevelEdgeToolBuilder`) and of
//! slice 8's `RoomCandidate`. The tests that exercise them in context read whole bevels; these pin
//! each piece alone.

use solidmesh::bevel::bevel_edge_tool_builder as builder;
use solidmesh::bevel::surface::{surface_ring, SurfaceVertexPool};
use solidmesh::bevel::BevelAdvisoryCode;
use solidmesh::topology::EdgeConvexity;
use solidmesh_math::{Vector2, Vector3};

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
    assert!((reach - 2.0 * 2.0_f64.sqrt()).abs() < 1e-12);
}

#[test]
fn rust_only_the_closed_forms_read_the_size_and_the_wedge() {
    assert_eq!(builder::tangent_distance(2.0, 60.0, false), 2.0);
    assert!(
        (builder::tangent_distance(2.0, 60.0, true) - 2.0 / 30.0_f64.to_radians().tan()).abs()
            < 1e-12
    );
    assert_eq!(builder::tangent_distance(2.0, 0.0, true), f64::INFINITY);
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

// ---------------------------------------------------------------- RoomCandidate (slice 8)

#[test]
fn rust_only_a_room_candidate_takes_the_record_defaults() {
    use solidmesh::bevel::level_line::{RoomCandidate, RoomSource};

    let candidate = RoomCandidate::new(RoomSource::foreign(3), 1.25);
    assert_eq!(candidate.source, RoomSource::foreign(3));
    assert_eq!(candidate.room, 1.25);
    assert!(candidate.room_without_the_other.is_nan());
    assert_eq!(candidate.room_without_the_other_at_most, f64::INFINITY);
    assert!(candidate.group_members.is_none());
}
