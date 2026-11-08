//! `refinement_splits_enqueue::bisect_along_longest_edge_path` - the bisection the sliver guard
//! takes instead of a rule's thinning split - on hand-built strips.

use std::collections::{HashMap, VecDeque};
use std::rc::Rc;

use solidmesh::bevel::level_line::refinement_splits::LONGEST_EDGE_PATH_CAP;
use solidmesh::bevel::level_line::refinement_splits_enqueue::bisect_along_longest_edge_path;
use solidmesh::bevel::level_line::{LevelLineMesh, LevelLineTriangle};
use solidmesh_math::{ieee_math, Vector3};

#[test]
fn a_thin_strip_loses_its_longest_edge_and_stays_conforming_and_shaped() {
    let mut mesh = path_strip();
    let input_angle = smallest_angle(&mesh);
    let input_boundary = boundary_length(&mesh);
    let mut pending = VecDeque::new();

    let done = bisect_along_longest_edge_path(&mut mesh, 0, &mut pending, &mut HashMap::new())
        .expect("the bisection");

    assert!(done);
    assert!(
        !mesh.has_edge(1, 2),
        "the target's longest edge is what the bisection is for"
    );
    assert!(mesh
        .edges()
        .iter()
        .all(|e| mesh.triangles_on(e.0, e.1).len() <= 2));
    assert!(
        (boundary_length(&mesh) - input_boundary).abs() < 1e-9,
        "a hanging vertex would leave an unmatched edge inside, adding to the boundary"
    );
    assert!(
        smallest_angle(&mesh) >= input_angle / 2.0,
        "Rivara's bound: input {input_angle}"
    );
    assert!(
        !mesh.has_edge(2, 3),
        "the path's terminal edge 2-3 was bisected on the way"
    );
}

#[test]
fn a_chain_piece_at_the_paths_end_stops_it_and_leaves_the_mesh_alone() {
    let mut mesh = path_strip();
    mesh.is_chain_edge = Some(Rc::new(|a, b| (a == 2 && b == 3) || (a == 3 && b == 2)));
    let mut pending = VecDeque::new();

    let done = bisect_along_longest_edge_path(&mut mesh, 0, &mut pending, &mut HashMap::new())
        .expect("the bisection");

    assert!(!done);
    assert_eq!(mesh.vertex_count(), 5);
    assert_eq!(mesh.triangle_count(), 3);
    assert_eq!(pending.len(), 0);
}

#[test]
fn a_boundary_longest_edge_is_bisected_and_its_children_queued_one_deeper() {
    let mut mesh = LevelLineMesh::from_triangles(
        &[
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(16.0, 0.0, 0.0),
            Vector3::new(0.0, 0.5, 0.0),
        ],
        &[(0, 1, 2)],
        None,
    )
    .expect("a mesh");
    let mut pending = VecDeque::new();
    let mut depth = HashMap::from([(0, 3)]);

    let done = bisect_along_longest_edge_path(&mut mesh, 0, &mut pending, &mut depth)
        .expect("the bisection");

    assert!(done);
    assert!(!mesh.has_edge(1, 2));
    let live: Vec<i32> = (0..mesh.triangle_count())
        .filter(|&t| !mesh.is_removed(t))
        .collect();
    assert_eq!(live.len(), 2);
    assert!(live.iter().all(|t| pending.contains(t)));
    assert!(live.iter().all(|t| depth[t] == 4));
}

#[test]
fn the_children_keep_their_parents_winding_and_tag() {
    let mut mesh = LevelLineMesh::from_triangles(
        &[
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(16.0, 0.0, 0.0),
            Vector3::new(0.0, 0.5, 0.0),
            Vector3::new(16.5, 1.0, 0.0),
            Vector3::new(0.5, 1.5, 0.0),
        ],
        &[(0, 1, 2), (2, 1, 3), (2, 3, 4)],
        Some(&[7, 7, 7]),
    )
    .expect("a mesh");

    assert!(bisect_along_longest_edge_path(
        &mut mesh,
        0,
        &mut VecDeque::new(),
        &mut HashMap::new()
    )
    .expect("the bisection"));

    for t in live(&mesh) {
        let normal = (mesh.position(t.b) - mesh.position(t.a))
            .cross(mesh.position(t.c) - mesh.position(t.a));
        assert!(
            normal.z > 0.0,
            "every input triangle winds counter-clockwise in z"
        );
        assert_eq!(t.tag, 7);
    }
}

#[test]
fn a_midpoint_the_mesh_answers_with_an_existing_vertex_stops_it_part_way() {
    // The first bisection (terminal edge 2-3, half-length 8.25) mints; the second (the target's
    // own 1-2, half-length 8.004) stands within SamePlace of an end, so the mesh answers with
    // that end.
    let mut mesh = path_strip();
    mesh.same_place = 8.1;

    let done =
        bisect_along_longest_edge_path(&mut mesh, 0, &mut VecDeque::new(), &mut HashMap::new())
            .expect("the bisection");

    assert!(!done);
    assert_eq!(mesh.vertex_count(), 6, "the first bisection is kept");
    assert!(!mesh.has_edge(2, 3));
    assert!(mesh.has_edge(1, 2));
}

#[test]
fn a_non_manifold_edge_on_the_path_stops_it_and_leaves_the_mesh_alone() {
    let mut mesh = LevelLineMesh::from_triangles(
        &[
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(16.0, 0.0, 0.0),
            Vector3::new(0.0, 0.5, 0.0),
            Vector3::new(16.5, 1.0, 0.0),
            Vector3::new(0.5, 1.5, 0.0),
            Vector3::new(8.0, 0.75, 3.0),
        ],
        &[(0, 1, 2), (2, 1, 3), (2, 3, 4), (3, 2, 5)],
        None,
    )
    .expect("a mesh");

    let done =
        bisect_along_longest_edge_path(&mut mesh, 0, &mut VecDeque::new(), &mut HashMap::new())
            .expect("the bisection");

    assert!(!done);
    assert_eq!(mesh.vertex_count(), 6);
    assert_eq!(mesh.triangle_count(), 4);
}

#[test]
fn a_path_longer_than_the_cap_stops_it_and_leaves_the_mesh_alone() {
    // A fan whose spokes grow by a hundredth each: every triangle's longest edge is the spoke it
    // shares with the next, so the path from the first runs the whole fan - longer than the cap.
    let count = LONGEST_EDGE_PATH_CAP + 8;
    let mut positions = vec![Vector3::ZERO];
    for i in 0..=count {
        let r = 10.0 + (0.01 * i as f64);
        positions.push(Vector3::new(
            r * (0.02 * i as f64).cos(),
            r * (0.02 * i as f64).sin(),
            0.0,
        ));
    }

    let indices: Vec<(i32, i32, i32)> = (0..count).map(|i| (0, i + 1, i + 2)).collect();
    let mut mesh = LevelLineMesh::from_triangles(&positions, &indices, None).expect("a mesh");

    let done =
        bisect_along_longest_edge_path(&mut mesh, 0, &mut VecDeque::new(), &mut HashMap::new())
            .expect("the bisection");

    assert!(!done);
    assert_eq!(mesh.vertex_count(), positions.len() as i32);
}

/// Three triangles whose longest-edge path runs target (0, 1, 2) -> (2, 1, 3) -> (2, 3, 4): the
/// target's longest edge 1-2 is not the middle triangle's (2-3 is), and 2-3 is the longest of both
/// triangles on it, so it is the terminal edge.
fn path_strip() -> LevelLineMesh {
    LevelLineMesh::from_triangles(
        &[
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(16.0, 0.0, 0.0),
            Vector3::new(0.0, 0.5, 0.0),
            Vector3::new(16.5, 1.0, 0.0),
            Vector3::new(0.5, 1.5, 0.0),
        ],
        &[(0, 1, 2), (2, 1, 3), (2, 3, 4)],
        None,
    )
    .expect("a mesh")
}

fn live(mesh: &LevelLineMesh) -> Vec<LevelLineTriangle> {
    (0..mesh.triangle_count())
        .filter(|&t| !mesh.is_removed(t))
        .map(|t| mesh.triangles()[t as usize])
        .collect()
}

fn smallest_angle(mesh: &LevelLineMesh) -> f64 {
    let mut smallest = f64::MAX;
    for t in live(mesh) {
        for corner in 0..3 {
            let at = mesh.position(t.corner(corner));
            let u = (mesh.position(t.corner((corner + 1) % 3)) - at).get_normal();
            let v = (mesh.position(t.corner((corner + 2) % 3)) - at).get_normal();
            smallest = ieee_math::min_f64(smallest, u.dot(v).clamp(-1.0, 1.0).acos());
        }
    }

    smallest
}

fn boundary_length(mesh: &LevelLineMesh) -> f64 {
    mesh.edges()
        .iter()
        .filter(|e| mesh.triangles_on(e.0, e.1).len() == 1)
        .map(|e| mesh.length(e.0, e.1))
        .sum()
}
