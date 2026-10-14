//! Rust-only checks of the first bevel-core round-1 leaves (package R1.P01): the chain field's
//! closest points, the closed-form chain approach, the edge plumbing (`TriangleEdges`,
//! `ChainOnTheMesh`, `BandTriangles`, `VirtualRail`) and the working mesh's accessors. The
//! whole-build tests also cover them.

use geometry_processing::bevel::level_line::{
    band_triangles, chain_approach, chain_on_the_mesh, sharp_joint_corners, triangle_edges,
    virtual_rail, ChainField, LevelLineMesh, LevelLineTriangle,
};
use vector_math::Vector3;

fn v(x: f64, y: f64, z: f64) -> Vector3 {
    Vector3::new(x, y, z)
}

fn tri(a: i32, b: i32, c: i32, tag: i32) -> LevelLineTriangle {
    LevelLineTriangle { a, b, c, tag }
}

/// A unit square in z = 0 as two triangles (0 1 2) tag 1 and (0 2 3) tag 2, sharing edge 0-2.
fn square() -> LevelLineMesh {
    let mut mesh = LevelLineMesh {
        vertices: vec![
            v(0.0, 0.0, 0.0),
            v(1.0, 0.0, 0.0),
            v(1.0, 1.0, 0.0),
            v(0.0, 1.0, 0.0),
        ],
        triangles: vec![tri(0, 1, 2, 1), tri(0, 2, 3, 2)],
        removed: vec![false, false],
        ..Default::default()
    };
    for (i, t) in mesh.triangles.clone().iter().enumerate() {
        for (p, q) in triangle_edges::level_line_triangle_corners(*t) {
            mesh.edge_triangles
                .entry(triangle_edges::key(p, q))
                .or_default()
                .push(i as i32);
        }
    }
    mesh
}

#[test]
fn rust_only_closest_on_triangle_reads_each_region() {
    let (a, b, c) = (v(0.0, 0.0, 0.0), v(2.0, 0.0, 0.0), v(0.0, 2.0, 0.0));
    assert_eq!(
        ChainField::closest_on_triangle(v(-1.0, -1.0, 3.0), a, b, c),
        a
    );
    assert_eq!(
        ChainField::closest_on_triangle(v(3.0, -1.0, 0.0), a, b, c),
        b
    );
    assert_eq!(
        ChainField::closest_on_triangle(v(1.0, -1.0, 0.0), a, b, c),
        v(1.0, 0.0, 0.0)
    );
    assert_eq!(
        ChainField::closest_on_triangle(v(0.5, 0.5, 4.0), a, b, c),
        v(0.5, 0.5, 0.0)
    );
    assert_eq!(
        ChainField::closest_on_triangle(v(2.0, 2.0, 0.0), a, b, c),
        v(1.0, 1.0, 0.0)
    );
    assert_eq!(ChainField::closest_on_segment(v(5.0, 1.0, 0.0), a, b), b);
    assert_eq!(ChainField::closest_on_segment(v(1.0, 1.0, 0.0), a, a), a);
}

#[test]
fn rust_only_chain_approach_ties_parallel_segments_at_the_lowest_parameter() {
    // Two parallel segments one apart: every chain parameter attains 1, and the tie answers at
    // the lowest, so the attaining point is the mesh segment's point beside the chain's start.
    let (distance, on_piece) = chain_approach::segment_to_segment(
        v(0.0, 1.0, 0.0),
        v(4.0, 1.0, 0.0),
        v(1.0, 0.0, 0.0),
        v(3.0, 0.0, 0.0),
    );
    assert_eq!(distance, 1.0);
    assert_eq!(on_piece, v(1.0, 1.0, 0.0));

    // A chain segment piercing a triangle answers zero at the crossing.
    let (distance, on_piece) = chain_approach::triangle_to_segment(
        v(0.0, 0.0, 0.0),
        v(4.0, 0.0, 0.0),
        v(0.0, 4.0, 0.0),
        v(1.0, 1.0, -1.0),
        v(1.0, 1.0, 3.0),
    );
    assert!(distance.abs() < 1e-12);
    assert!((on_piece - v(1.0, 1.0, 0.0)).length() < 1e-12);
}

#[test]
fn rust_only_chain_edges_stand_between_their_two_triangles() {
    let indices = [(0, 1, 2), (0, 2, 3)];
    let standing = triangle_edges::standing_map(&indices);
    assert_eq!(standing[&(0, 2)], vec![0, 1]);
    assert_eq!(standing[&(0, 1)], vec![0]);

    assert_eq!(
        chain_on_the_mesh::sides_of(&indices, &standing, 2, 0),
        Ok((0, 1))
    );
    assert_eq!(
        chain_on_the_mesh::sides_of(&indices, &standing, 0, 2),
        Ok((1, 0))
    );
    assert!(chain_on_the_mesh::sides_of(&indices, &standing, 0, 1).is_err());

    assert_eq!(
        chain_on_the_mesh::chain_edges(&[4, 5, 6], true),
        vec![(4, 5), (5, 6), (6, 4)]
    );
    assert_eq!(
        chain_on_the_mesh::chain_edges(&[4, 5, 6], false),
        vec![(4, 5), (5, 6)]
    );
    assert!(triangle_edges::traverses((0, 1, 2), 2, 0));
    assert!(!triangle_edges::traverses((0, 1, 2), 0, 2));
}

#[test]
fn rust_only_working_mesh_reads_its_edges_in_canonical_order() {
    let mut mesh = square();
    assert_eq!(mesh.edges(), vec![(0, 1), (0, 2), (0, 3), (1, 2), (2, 3)]);
    assert_eq!(
        band_triangles::edges_of_side(&mesh, 2),
        vec![(0, 2), (0, 3), (2, 3)]
    );
    assert_eq!(virtual_rail::chain_pair(&mesh), Some((0, 1, 0)));
    assert_eq!(mesh.length(0, 2), 2f64.sqrt());
    assert_eq!(mesh.triangles()[1].corner(2), 3);
    assert_eq!(strip_directed(&mesh), (2, 0));

    assert!(mesh.set_same_place(f64::NAN).is_err());
    assert!(mesh.set_same_place(-1.0).is_err());
    assert_eq!(mesh.set_same_place(0.25), Ok(()));
    assert_eq!(mesh.same_place(), 0.25);

    // A point well inside the first triangle is held; one on its diagonal is not.
    assert!(sharp_joint_corners::holds(
        &mesh,
        0,
        v(0.7, 0.2, 0.0),
        v(0.0, 0.0, 1.0)
    ));
    assert!(!sharp_joint_corners::holds(
        &mesh,
        0,
        v(0.5, 0.5, 0.0),
        v(0.0, 0.0, 1.0)
    ));
    assert!(!sharp_joint_corners::holds(
        &mesh,
        0,
        v(0.7, 0.2, 0.0),
        v(1.0, 0.0, 0.0)
    ));
}

fn strip_directed(mesh: &LevelLineMesh) -> (i32, i32) {
    geometry_processing::bevel::level_line::strip_clip::directed(mesh.triangles()[0], 0, 2)
}
