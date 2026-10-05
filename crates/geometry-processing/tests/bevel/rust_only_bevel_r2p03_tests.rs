//! Rust-only checks of bevel-core package R2.P03's working-mesh edits: building from triangles,
//! the conforming edge split (and its answer at an existing vertex), the interior split, a free
//! vertex and a face, and the needle repair re-tiling a neighbour through an apex. Their C#
//! coverage comes through the whole-build test classes of later packages.

use std::collections::VecDeque;

use geometry_processing::bevel::level_line::{LevelLineMesh, LevelLineMeshError};
use vector_math::Vector3;

fn v(x: f64, y: f64, z: f64) -> Vector3 {
    Vector3::new(x, y, z)
}

/// A unit square as two triangles sharing the diagonal 0-2.
fn square() -> LevelLineMesh {
    LevelLineMesh::from_triangles(
        &[
            v(0.0, 0.0, 0.0),
            v(1.0, 0.0, 0.0),
            v(1.0, 1.0, 0.0),
            v(0.0, 1.0, 0.0),
        ],
        &[(0, 1, 2), (0, 2, 3)],
        Some(&[1, 2]),
    )
    .expect("a square builds")
}

#[test]
fn rust_only_the_working_mesh_splits_conformingly() {
    let mut mesh = square();
    assert_eq!(mesh.input_edge_behind(3), (3, 3));
    assert!(matches!(
        LevelLineMesh::from_triangles(&[v(0.0, 0.0, 0.0)], &[(0, 0, 0)], None),
        Err(LevelLineMeshError::RepeatsAVertex(_))
    ));

    // Splitting the diagonal splits both triangles, each keeping its tag.
    let mid = mesh
        .split_edge(0, 2, v(0.5, 0.5, 0.0), true)
        .expect("an edge");
    assert_eq!(mid, 4);
    assert_eq!(mesh.triangle_count(), 4);
    assert_eq!(mesh.triangles_on(0, 4).len(), 2);
    assert!(!mesh.has_edge(0, 2));
    assert_eq!(mesh.input_edge_behind(4), (0, 2));
    let tags: Vec<i32> = mesh.triangles().iter().map(|t| t.tag).collect();
    assert_eq!(tags, vec![1, 2, 1, 2]);

    // Asked again at an end, it answers with that end and splits nothing.
    assert_eq!(mesh.split_edge(0, 4, v(0.0, 0.0, 0.0), true).ok(), Some(0));
    assert!(matches!(
        mesh.split_edge(1, 3, v(0.5, 0.5, 0.0), true),
        Err(LevelLineMeshError::NoEdge(_))
    ));

    let inside = mesh.split_triangle(0, v(0.6, 0.2, 0.0)).expect("no budget");
    assert_eq!(inside, 5);
    assert_eq!(mesh.triangle_count(), 6);
    assert_eq!(mesh.input_edge_behind(inside), (-1, -1));

    let free = mesh.add_vertex(v(5.0, 5.0, 0.0));
    assert_eq!(free, 6);
    assert!(mesh.add_face(1, 1, 2, 0).is_err());
    assert_eq!(mesh.add_face(1, 2, 6, 0).ok(), Some(6));
}

#[test]
fn rust_only_a_needle_is_repaired_through_its_neighbour() {
    // A needle (0, 1, 2) with its apex 2 inside its long edge 0-1, and a healthy neighbour (1, 0, 3)
    // across that edge: the repair drops the needle and re-tiles the neighbour through the apex.
    let mut mesh = LevelLineMesh::from_triangles(
        &[
            v(0.0, 0.0, 0.0),
            v(10.0, 0.0, 0.0),
            v(4.0, 1e-9, 0.0),
            v(5.0, -5.0, 0.0),
        ],
        &[(0, 1, 2), (1, 0, 3)],
        None,
    )
    .expect("builds");
    mesh.needle_floor = 1e-6;
    let mut pending = VecDeque::from([0]);
    let mut touched = Vec::new();
    mesh.repair_from(&mut pending, &mut touched)
        .expect("no budget");
    assert_eq!(mesh.needles_repaired, 1);
    assert_eq!(touched.len(), 2);
    assert!(mesh.is_removed(0));
    assert!(!mesh.has_edge(0, 1));
    assert!(mesh.has_edge(2, 3));
}

#[test]
fn rust_only_the_fill_lays_skip_degenerate_triangles() {
    use geometry_processing::bevel::level_line::{corner_fill, gap_fill};

    let mut mesh = square();
    let free = mesh.add_vertex(v(2.0, 0.0, 0.0));
    let mut added = Vec::new();

    // The corner fill skips any repeat; the gap fill skips exactly one repeated pair.
    corner_fill::lay(&mut mesh, 1, 1, free, &mut added).expect("skipped");
    corner_fill::lay(&mut mesh, 1, 1, 1, &mut added).expect("skipped");
    gap_fill::lay(&mut mesh, 1, free, free, &mut added).expect("skipped");
    assert!(added.is_empty());

    // A whole triangle is laid with the fill's tag; the gap fill hands a triple repeat to the
    // mesh, which refuses it.
    gap_fill::lay(&mut mesh, 1, free, 2, &mut added).expect("laid");
    assert_eq!(added.len(), 1);
    assert_eq!(mesh.triangles()[added[0] as usize].tag, gap_fill::FILL_TAG);
    corner_fill::lay(&mut mesh, 2, free, 1, &mut added).expect("laid");
    assert_eq!(added.len(), 2);
    assert!(matches!(
        gap_fill::lay(&mut mesh, 1, 1, 1, &mut added),
        Err(LevelLineMeshError::RepeatsAVertex(_))
    ));
}
