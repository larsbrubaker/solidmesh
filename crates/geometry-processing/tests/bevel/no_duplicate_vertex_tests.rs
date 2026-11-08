//! No split - by a rule, the chord rule, or the mesh's own edge and triangle splits - mints a
//! vertex where the mesh already stands one.

use std::cell::RefCell;
use std::collections::{HashMap, VecDeque};
use std::rc::Rc;

use solidmesh::bevel::level_line::refinement_splits::NOTHING_TO_INSERT;
use solidmesh::bevel::level_line::refinement_splits_enqueue::{
    insert_on_the_line, insert_where_it_stands, split_edge_and_enqueue,
};
use solidmesh::bevel::level_line::{LevelLineMesh, VertexFieldReading, VertexSide};
use solidmesh_math::Vector3;

use crate::level_line_test_geometry::PlaneField;

/// A fan of four round a hub, with its spoke 0-1 split at (0.5, 0, 0): (mesh, hub, split, neighbour).
fn split_fan() -> (LevelLineMesh, i32, i32, i32) {
    let positions = vec![
        Vector3::new(0.0, 0.0, 0.0),  // 0 - the hub
        Vector3::new(1.0, 0.0, 0.0),  // 1
        Vector3::new(0.0, 1.0, 0.0),  // 2
        Vector3::new(-1.0, 0.0, 0.0), // 3
        Vector3::new(0.0, -1.0, 0.0), // 4
    ];

    let indices = vec![(0, 1, 2), (0, 2, 3), (0, 3, 4), (0, 4, 1)];

    let mut mesh = LevelLineMesh::from_triangles(&positions, &indices, None).expect("a fan");

    let mid = mesh
        .split_edge(0, 1, Vector3::new(0.5, 0.0, 0.0), true)
        .expect("a split");
    (mesh, 0, mid, 2)
}

fn triangle_over(mesh: &LevelLineMesh, a: i32, b: i32, c: i32) -> i32 {
    for triangle in 0..mesh.triangle_count() {
        let t = mesh.triangles()[triangle as usize];
        let has = (0..3)
            .filter(|&corner| {
                let v = t.corner(corner);
                v == a || v == b || v == c
            })
            .count();
        if has == 3 {
            return triangle;
        }
    }

    panic!("no triangle over those three vertices");
}

fn no_two_vertices_coincide(mesh: &LevelLineMesh) {
    for v in 0..mesh.vertex_count() {
        for w in (v + 1)..mesh.vertex_count() {
            assert!(
                mesh.length(v, w) > 1e-6,
                "vertices {v} and {w} stand at the same place"
            );
        }
    }
}

fn no_degenerate_triangles(mesh: &LevelLineMesh) {
    for triangle in 0..mesh.triangle_count() {
        if mesh.is_removed(triangle) {
            continue;
        }

        let t = mesh.triangles()[triangle as usize];
        let a = mesh.position(t.a);
        let b = mesh.position(t.b);
        let c = mesh.position(t.c);
        assert!(
            (b - a).cross(c - a).length() > 1e-9,
            "triangle {triangle} ({}, {}, {}) has no area",
            t.a,
            t.b,
            t.c
        );
    }
}

#[test]
fn a_rule_asking_at_an_existing_corner_mints_nothing() {
    let (mut mesh, hub, split, neighbour) = split_fan();
    let triangle = triangle_over(&mesh, hub, split, neighbour);
    let before = mesh.vertex_count();

    let mut pending = VecDeque::new();
    let mut depth = HashMap::new();
    let at = mesh.position(split);
    let landed = insert_where_it_stands(&mut mesh, triangle, at, &mut pending, &mut depth, None)
        .expect("inserted");

    assert_eq!(landed, split, "the corner already carries that point");
    assert_eq!(mesh.vertex_count(), before);
    no_two_vertices_coincide(&mesh);
    no_degenerate_triangles(&mesh);
}

#[test]
fn a_rule_asking_within_the_tolerance_of_a_corner_mints_nothing() {
    let (mut mesh, hub, split, neighbour) = split_fan();
    let triangle = triangle_over(&mesh, hub, split, neighbour);
    let before = mesh.vertex_count();

    let nearly = mesh.position(split) + Vector3::new(1e-7, 1e-7, 0.0);

    let mut pending = VecDeque::new();
    let mut depth = HashMap::new();
    let landed =
        insert_where_it_stands(&mut mesh, triangle, nearly, &mut pending, &mut depth, None)
            .expect("inserted");

    assert_eq!(landed, split);
    assert_eq!(mesh.vertex_count(), before);
    no_two_vertices_coincide(&mesh);
    no_degenerate_triangles(&mesh);
}

#[test]
fn the_chord_rule_at_an_existing_corner_puts_the_corner_on_the_line() {
    let (mut mesh, hub, split, neighbour) = split_fan();
    let triangle = triangle_over(&mesh, hub, split, neighbour);
    let before = mesh.vertex_count();

    let mut pending = VecDeque::new();
    let mut depth = HashMap::new();
    let at = mesh.position(split) + Vector3::new(3e-10, 3e-10, 0.0);
    let landed = insert_on_the_line(&mut mesh, triangle, at, 1e-9, &mut pending, &mut depth)
        .expect("inserted");

    assert_eq!(landed, split);
    assert_eq!(mesh.vertex_count(), before);
    no_two_vertices_coincide(&mesh);
    no_degenerate_triangles(&mesh);
}

#[test]
fn the_chord_rule_just_off_a_corner_mints_nothing_and_moves_no_side() {
    let (mesh, hub, split, neighbour) = split_fan();
    let triangle = triangle_over(&mesh, hub, split, neighbour);
    let before = mesh.vertex_count();

    let reading = VertexFieldReading::new(
        Rc::new(RefCell::new(mesh)),
        Rc::new(PlaneField::new(Vector3::new(0.0, 0.0, 1.0), -1.0)),
        1e-9,
    )
    .expect("a snap");
    assert_eq!(reading.side_of(split).ok(), Some(VertexSide::Outside));

    let just_off = reading.mesh.borrow().position(split) + Vector3::new(1e-7, 0.0, 0.0);

    let mut pending = VecDeque::new();
    let mut depth = HashMap::new();
    let landed = insert_on_the_line(
        &mut reading.mesh.borrow_mut(),
        triangle,
        just_off,
        1e-9,
        &mut pending,
        &mut depth,
    )
    .expect("inserted");

    assert_eq!(landed, NOTHING_TO_INSERT);
    assert_eq!(reading.mesh.borrow().vertex_count(), before);
    assert_eq!(
        pending.len(),
        0,
        "nothing was divided, so there are no children"
    );

    assert_eq!(reading.side_of(split).ok(), Some(VertexSide::Outside));
    no_two_vertices_coincide(&reading.mesh.borrow());
    no_degenerate_triangles(&reading.mesh.borrow());
}

#[test]
fn splitting_a_triangle_at_its_own_corner_mints_nothing() {
    let (mut mesh, hub, split, neighbour) = split_fan();
    let triangle = triangle_over(&mesh, hub, split, neighbour);
    mesh.set_same_place(1e-3).expect("a length");
    let before = mesh.vertex_count();
    let triangles_before = mesh.triangle_count();

    let at = mesh.position(hub) + Vector3::new(1e-4, 0.0, 0.0);
    let landed = mesh.split_triangle(triangle, at).expect("no budget");

    assert_eq!(landed, hub);
    assert_eq!(mesh.vertex_count(), before);
    assert_eq!(mesh.triangle_count(), triangles_before);
    no_degenerate_triangles(&mesh);
}

#[test]
fn a_point_inside_an_edge_still_splits_it() {
    let (mut mesh, hub, split, neighbour) = split_fan();
    let triangle = triangle_over(&mesh, hub, split, neighbour);
    let before = mesh.vertex_count();

    let inside = 0.5 * (mesh.position(hub) + mesh.position(split));

    let mut pending = VecDeque::new();
    let mut depth = HashMap::new();
    let landed =
        insert_where_it_stands(&mut mesh, triangle, inside, &mut pending, &mut depth, None)
            .expect("inserted");

    assert_eq!(landed, before, "a new vertex is the next id");
    assert_eq!(mesh.vertex_count(), before + 1);
    assert!(!mesh.triangles_on(hub, landed).is_empty());
    no_two_vertices_coincide(&mesh);
    no_degenerate_triangles(&mesh);
}

#[test]
fn splitting_an_edge_at_its_own_end_mints_nothing() {
    let (mut mesh, hub, split, _) = split_fan();
    let before = mesh.vertex_count();
    let triangles_before = mesh.triangle_count();

    let at = mesh.position(split);
    let landed = mesh.split_edge(hub, split, at, true).expect("a split");

    assert_eq!(landed, split);
    assert_eq!(mesh.vertex_count(), before);
    assert_eq!(mesh.triangle_count(), triangles_before);
    no_degenerate_triangles(&mesh);
}

#[test]
fn a_split_within_the_snap_of_an_end_mints_nothing_and_enqueues_nothing() {
    let (mut mesh, hub, split, _) = split_fan();
    let snap = 1e-3;
    mesh.set_same_place(snap).expect("a length");
    let before = mesh.vertex_count();
    let triangles_before = mesh.triangle_count();

    let nearly_the_end = mesh.position(split)
        + ((mesh.position(hub) - mesh.position(split)).get_normal() * (0.1 * snap));

    let mut pending = VecDeque::new();
    let mut depth = HashMap::new();
    let landed = split_edge_and_enqueue(
        &mut mesh,
        hub,
        split,
        nearly_the_end,
        &mut pending,
        &mut depth,
    )
    .expect("a split");

    assert_eq!(landed, split);
    assert_eq!(mesh.vertex_count(), before);
    assert_eq!(mesh.triangle_count(), triangles_before);
    assert_eq!(
        pending.len(),
        0,
        "nothing was divided, so there are no children"
    );
    no_degenerate_triangles(&mesh);
}

#[test]
fn splitting_an_edge_where_an_opposite_corner_stands_mints_nothing() {
    let positions = vec![
        Vector3::new(-9.0, -3.0, 0.0), // 0 - one end of the long edge
        Vector3::new(-4.0, -3.0, 0.0), // 1 - the apex, standing exactly on the segment from 0 to 2
        Vector3::new(5.0, -3.0, 0.0),  // 2 - the other end of the long edge
        Vector3::new(0.0, 3.0, 0.0),   // 3 - the ordinary neighbour across the long edge
    ];

    let indices = vec![
        (0, 1, 2), // the needle: no height at all
        (2, 0, 3),
    ];

    let mut mesh = LevelLineMesh::from_triangles(&positions, &indices, None).expect("a mesh");
    let before = mesh.vertex_count();
    let triangles_before = mesh.triangle_count();

    let mut pending = VecDeque::new();
    let mut depth = HashMap::new();
    let at = mesh.position(1);
    let landed =
        split_edge_and_enqueue(&mut mesh, 0, 2, at, &mut pending, &mut depth).expect("a split");

    assert_eq!(landed, 1, "the apex already carries that point");
    assert_eq!(mesh.vertex_count(), before);
    assert_eq!(mesh.triangle_count(), triangles_before);
    assert_eq!(
        pending.len(),
        0,
        "nothing was divided, so there are no children"
    );
    no_two_vertices_coincide(&mesh);
}

#[test]
fn a_same_place_that_is_not_a_length_is_refused() {
    let (mut mesh, _, _, _) = split_fan();

    assert!(mesh.set_same_place(-1e-6).is_err());
    assert!(mesh.set_same_place(f64::NAN).is_err());
    assert_eq!(mesh.same_place(), 0.0);
}
