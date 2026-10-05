//! Rust-only checks of the bevel-core round-1 mesh readings (package R1.P01, run 3): where a
//! point stands in a triangle, the longest edge, the end-face bend, the needle bookkeeping and the
//! open fill's small tests. Their C# coverage comes through the whole-build test classes of later
//! packages.

use geometry_processing::bevel::level_line::{
    corner_fill, end_face_flow, open_gap_fill, refinement_splits, triangle_edges, LevelLineMesh,
    LevelLineTriangle, PointPlace,
};
use vector_math::Vector3;

fn v(x: f64, y: f64, z: f64) -> Vector3 {
    Vector3::new(x, y, z)
}

/// A right triangle 0 (0,0) 1 (4,0) 2 (0,3): its longest side is 1-2.
fn triangle() -> LevelLineMesh {
    LevelLineMesh {
        vertices: vec![v(0.0, 0.0, 0.0), v(4.0, 0.0, 0.0), v(0.0, 3.0, 0.0)],
        triangles: vec![LevelLineTriangle {
            a: 0,
            b: 1,
            c: 2,
            tag: 1,
        }],
        removed: vec![false],
        ..Default::default()
    }
}

#[test]
fn rust_only_a_point_stands_at_a_corner_before_on_an_edge() {
    let mesh = triangle();
    let t = mesh.triangles()[0];
    assert_eq!(
        refinement_splits::where_the_point_stands(&mesh, t, v(4.0, 1e-7, 0.0), None),
        (PointPlace::AtACorner, 1, 1)
    );
    assert_eq!(
        refinement_splits::where_the_point_stands(&mesh, t, v(2.0, 0.0, 0.0), None),
        (PointPlace::OnAnEdge, 0, 1)
    );
    assert_eq!(
        refinement_splits::where_the_point_stands(&mesh, t, v(1.0, 1.0, 0.0), None),
        (PointPlace::Interior, -1, -1)
    );
    // A caller's own, wider corner test.
    assert_eq!(
        refinement_splits::where_the_point_stands(&mesh, t, v(2.0, 0.0, 0.0), Some(2.5)),
        (PointPlace::AtACorner, 0, 0)
    );

    assert_eq!(refinement_splits::longest_edge(&mesh, 0), (1, 2));
    assert_eq!(refinement_splits::longest_edge_length(&mesh, t), 5.0);
    assert!(refinement_splits::cannot_be_divided_further(&mesh, 0, 5.0));
    assert!(!refinement_splits::cannot_be_divided_further(&mesh, 0, 4.9));
    assert_eq!(mesh.apex_of(t), 0);
}

#[test]
fn rust_only_a_needle_left_alone_is_counted_once() {
    let mut mesh = triangle();
    mesh.leave_the_needle_on_its_chain(0);
    mesh.leave_the_needle_on_its_chain(0);
    mesh.leave_the_needle(0);
    assert_eq!(mesh.needles_left_on_a_chain_edge, 1);
    assert_eq!(mesh.left_alone.len(), 1);
}

#[test]
fn rust_only_an_end_face_bends_only_past_half_a_degree() {
    let positions = [
        v(0.0, 0.0, 0.0),
        v(1.0, 0.0, 0.0),
        v(0.5, 1.0, 0.0),
        v(0.5, -1.0, 0.001),
        v(0.5, -1.0, 1.0),
    ];
    let flat = [(0, 1, 2), (1, 0, 3)];
    let standing = triangle_edges::standing_map(&flat);
    assert!(!end_face_flow::bends(&positions, &flat, &standing, 0, 1));
    let folded = [(0, 1, 2), (1, 0, 4)];
    let standing = triangle_edges::standing_map(&folded);
    assert!(end_face_flow::bends(&positions, &folded, &standing, 0, 1));
    // An edge on one triangle is a bend: it is a boundary of the end face.
    assert!(end_face_flow::bends(&positions, &folded, &standing, 1, 2));
}

#[test]
fn rust_only_small_fill_tests_read_as_csharp() {
    assert!(open_gap_fill::is_straight_row(&[3, 7, 7, 7]));
    assert!(open_gap_fill::is_straight_row(&[3, 3, 3, 9]));
    assert!(!open_gap_fill::is_straight_row(&[3, 7]));
    assert!(!open_gap_fill::is_straight_row(&[3, 5, 7]));

    assert!(corner_fill::same(v(1e6, 0.0, 0.0), v(1e6 + 1e-4, 0.0, 0.0)));
    assert!(!corner_fill::same(
        v(1.0, 0.0, 0.0),
        v(1.0 + 1e-6, 0.0, 0.0)
    ));
}
