//! Rust-only checks of R2.P05's small working-mesh readings: the corner cap's mitre depth, the
//! self-room ceiling over chain edges, the knife-end overrun's plane neighbour and the foreign
//! edge room's knife test by edge, open gap loops meeting, and the opposite-sense crease on the
//! working mesh.

use geometry_processing::bevel::level_line::{
    chain_on_the_mesh, corner_cap, end_face_sense, foreign_edge_room, knife_end_overrun,
    open_gap_fill, self_room, BandProfile, GapLoop, LevelLineMesh,
};
use vector_math::Vector3;

fn v(x: f64, y: f64, z: f64) -> Vector3 {
    Vector3::new(x, y, z)
}

/// A unit square as two triangles sharing the diagonal 0-2, tagged `tags`.
fn square(tags: [i32; 2]) -> LevelLineMesh {
    LevelLineMesh::from_triangles(
        &[
            v(0.0, 0.0, 0.0),
            v(1.0, 0.0, 0.0),
            v(1.0, 1.0, 0.0),
            v(0.0, 1.0, 0.0),
        ],
        &[(0, 1, 2), (0, 2, 3)],
        Some(&tags),
    )
    .expect("a square builds")
}

#[test]
fn rust_only_a_mitre_depth_averages_the_strip_planes() {
    let one = LevelLineMesh::from_triangles(
        &[v(0.0, 0.0, 0.0), v(1.0, 0.0, 0.0), v(0.0, 1.0, 0.0)],
        &[(0, 1, 2)],
        None,
    )
    .unwrap();
    let up = v(0.0, 0.0, 1.0);

    assert_eq!(
        corner_cap::mitre_depth(&one, &[0, 1, 2], v(0.3, 0.3, 5.0), up, &[0]),
        Some(-5.0)
    );
    // A cycle edge on no strip triangle, or a plane edge-on to the axis: no depth.
    assert_eq!(
        corner_cap::mitre_depth(&one, &[0, 1, 2], v(0.0, 0.0, 5.0), up, &[]),
        None
    );
    assert_eq!(
        corner_cap::mitre_depth(&one, &[0, 1, 2], v(0.0, 0.0, 5.0), v(1.0, 0.0, 0.0), &[0]),
        None
    );
    // The square's diagonal stands on two triangles.
    assert_eq!(
        corner_cap::mitre_depth(&square([0, 0]), &[0, 1, 2], v(0.0, 0.0, 1.0), up, &[0, 1]),
        None
    );
}

#[test]
fn rust_only_the_self_room_ceiling_reads_each_chain_vertex_once() {
    let mesh = square([0, 0]);
    let ceiling = self_room::self_room_ceiling(&mesh, &[(0, 1), (1, 2), (2, 1)]);
    assert_eq!(
        ceiling,
        self_room::chain_ceiling(&[v(0.0, 0.0, 0.0), v(1.0, 0.0, 0.0), v(1.0, 1.0, 0.0)])
    );
    assert!((ceiling - 2f64.sqrt()).abs() < 1e-12);
}

#[test]
fn rust_only_the_overrun_finds_the_open_neighbour_on_the_end_plane() {
    let mesh = square([0, 0]);
    let end = |vertex: Vector3, normal: Vector3| knife_end_overrun::KnifeEndOverrunEnd {
        vertex,
        normal,
        ..Default::default()
    };

    // Vertex 0's open edges reach 1 and 3; its diagonal to 2 stands on two triangles.
    assert_eq!(
        knife_end_overrun::meets_the_plane(&mesh, 0, -1, &end(v(1.0, 0.0, 0.0), v(1.0, 0.0, 0.0))),
        1
    );
    assert_eq!(
        knife_end_overrun::meets_the_plane(&mesh, 0, -1, &end(v(0.0, 1.0, 0.0), v(0.0, 1.0, 0.0))),
        3
    );
    assert_eq!(
        knife_end_overrun::meets_the_plane(&mesh, 0, 1, &end(v(1.0, 0.0, 0.0), v(1.0, 0.0, 0.0))),
        -1,
        "the inward vertex is never the answer"
    );
    assert_eq!(
        knife_end_overrun::meets_the_plane(
            &mesh,
            0,
            -1,
            &end(v(1.0, 1.0, 0.0), v(1.0, 1.0, 0.0).get_normal())
        ),
        -1,
        "the diagonal's far end is not on an open edge"
    );
}

#[test]
fn rust_only_a_knife_edge_needs_one_filled_side_and_a_fold() {
    // Both filled, or neither: not a fill's crease at all.
    assert!(!foreign_edge_room::is_a_knife_edge(&square([1, 2]), (0, 2)));
    assert!(!foreign_edge_room::is_a_knife_edge(&square([0, 0]), (0, 2)));
    // One filled, but flat: no knife.
    assert!(!foreign_edge_room::is_a_knife_edge(&square([1, 0]), (0, 2)));
    // An open edge stands on one triangle.
    assert!(!foreign_edge_room::is_a_knife_edge(&square([1, 0]), (0, 1)));
}

fn open_loop(side: i32, vertices: Vec<i32>) -> GapLoop {
    let parameters = vec![0.0; vertices.len()];
    GapLoop::new(side, false, vertices, parameters, 0, false, None)
}

#[test]
fn rust_only_open_loops_meet_end_to_end_on_the_ridge_between_their_sides() {
    let mesh = square([1, 2]);

    assert!(open_gap_fill::on_the_ridge_between(&mesh, 0, 1, 2).unwrap());
    assert!(open_gap_fill::on_the_ridge_between(&mesh, 2, 2, 1).unwrap());
    assert!(!open_gap_fill::on_the_ridge_between(&mesh, 0, 1, 3).unwrap());
    assert!(
        !open_gap_fill::on_the_ridge_between(&mesh, 1, 1, 2).unwrap(),
        "only open edges at 1"
    );

    // Left 0 .. 2 and right 2 .. 0: both ends shared, both on the 1|2 diagonal.
    assert!(open_gap_fill::ends_meet(
        &mesh,
        &open_loop(1, vec![0, 5, 2]),
        &open_loop(2, vec![2, 6, 0])
    )
    .unwrap());
    // Ends that are not the same vertex, or a loop too short to have two ends.
    assert!(
        !open_gap_fill::ends_meet(&mesh, &open_loop(1, vec![0, 2]), &open_loop(2, vec![0, 2]))
            .unwrap()
    );
    assert!(
        !open_gap_fill::ends_meet(&mesh, &open_loop(1, vec![0]), &open_loop(2, vec![2, 0]))
            .unwrap()
    );
    // Shared ends, but the sides are not the ones the ridge divides.
    assert!(
        !open_gap_fill::ends_meet(&mesh, &open_loop(1, vec![0, 2]), &open_loop(3, vec![2, 0]))
            .unwrap()
    );
}

#[test]
fn rust_only_an_opposite_sense_crease_needs_two_triangles_on_each_edge_and_one_filled() {
    // The chain edge 0-1 is open: one triangle.
    assert!(!end_face_sense::is_opposite_sense_crease_on(
        &square([1, 0]),
        (0, 1),
        (0, 2)
    ));
    // The crease's triangles both filled.
    assert!(!end_face_sense::is_opposite_sense_crease_on(
        &square([1, 2]),
        (0, 2),
        (0, 2)
    ));
}

#[test]
fn rust_only_unit_setbacks_read_each_edges_dihedral_and_average_at_the_vertices() {
    // Two triangles folded square along 0-1: the floor z = 0 and the wall y = 0.
    let positions = [
        v(0.0, 0.0, 0.0),
        v(1.0, 0.0, 0.0),
        v(0.0, 1.0, 0.0),
        v(0.0, 0.0, 1.0),
    ];
    let indices = [(0, 1, 2), (1, 0, 3)];
    let edges = chain_on_the_mesh::chain_edges(&[0, 1], false);

    let chamfer = chain_on_the_mesh::per_edge_unit_setbacks(
        &positions,
        &indices,
        &edges,
        BandProfile::Chamfer,
    )
    .unwrap();
    assert_eq!(chamfer, vec![1.0]);

    // A fillet of radius 1 in a square wedge sets back tan(45 degrees) = 1.
    let fillet = chain_on_the_mesh::per_edge_unit_setbacks(
        &positions,
        &indices,
        &edges,
        BandProfile::Fillet,
    )
    .unwrap();
    assert!((fillet[0] - 1.0).abs() < 1e-12, "{fillet:?}");

    let at_vertices = chain_on_the_mesh::unit_setbacks(
        &positions,
        &indices,
        &[0, 1],
        false,
        &edges,
        BandProfile::Chamfer,
    )
    .unwrap();
    assert_eq!(at_vertices, vec![1.0, 1.0]);

    // An edge on one triangle only is refused, as C# throws.
    let open = chain_on_the_mesh::chain_edges(&[1, 2], false);
    assert!(chain_on_the_mesh::per_edge_unit_setbacks(
        &positions,
        &indices,
        &open,
        BandProfile::Chamfer
    )
    .is_err());
}
