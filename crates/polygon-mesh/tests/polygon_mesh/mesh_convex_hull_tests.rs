//! The mesh's cached convex hull, plus Rust-only checks of the transformed bounds
//! (`TransformedAabbCache`) built on the hull.
//!
//! The hull cache is typed and keyed on `changed_count`, read with `has_cached_convex_hull`, and
//! "same reference" is `Arc::ptr_eq`. Hulls are only ever built synchronously, so there is no
//! in-flight hull to wait on: `sync_hull_does_not_wait_on_an_in_flight_hull` checks that a
//! hull request answers promptly.

use std::sync::Arc;
use std::time::Instant;

use solidmesh_math::{Matrix4X4, Vector3, Vector3Float};
use solidmesh_mesh::processors::ConvexHull;
use solidmesh_mesh::solids::platonic_solids;

fn hull_arc(hull: Option<ConvexHull<'_>>) -> Arc<solidmesh_mesh::Mesh> {
    match hull {
        Some(ConvexHull::Hull(hull)) => hull,
        other => panic!("expected a built hull, got {other:?}"),
    }
}

#[test]
fn sync_hull_is_computed_and_cached() {
    let mesh = platonic_solids::create_cube(10.0, 10.0, 10.0);

    let hull = hull_arc(mesh.get_convex_hull(false));

    assert!(!hull.faces.is_empty());

    // the second call comes back from the cache on the mesh, not from a second hull build
    assert!(Arc::ptr_eq(&hull_arc(mesh.get_convex_hull(false)), &hull));
}

#[test]
fn sync_hull_does_not_wait_on_an_in_flight_hull() {
    let mesh = platonic_solids::create_cube(10.0, 10.0, 10.0);

    let timer = Instant::now();
    let hull = mesh.get_convex_hull(false);
    let elapsed = timer.elapsed();

    assert!(hull.is_some());
    assert!(elapsed.as_millis() < 500);
}

#[test]
fn changing_the_mesh_drops_the_cached_hull() {
    let mut mesh = platonic_solids::create_cube(10.0, 10.0, 10.0);

    let hull = mesh.get_convex_hull(false);
    assert!(hull.is_some());
    assert!(mesh.has_cached_convex_hull());

    mesh.vertices.push(Vector3Float::new(20.0, 20.0, 20.0));
    mesh.mark_as_changed();

    assert!(!mesh.has_cached_convex_hull());
}

#[test]
fn rust_only_flat_mesh_is_its_own_hull() {
    let mut mesh = platonic_solids::create_cube(10.0, 10.0, 0.0);
    mesh.clean_and_merge();
    assert!(mesh.faces.len() >= 4);
    match mesh.get_convex_hull(false) {
        Some(ConvexHull::Mesh(itself)) => assert!(std::ptr::eq(itself, &mesh)),
        other => panic!("expected the mesh itself, got {other:?}"),
    }
    assert!(!mesh.has_cached_convex_hull());
}

#[test]
fn rust_only_transformed_bounds_follow_the_mesh() {
    let mut mesh = platonic_solids::create_cube(10.0, 10.0, 10.0);

    let rotate = Matrix4X4::create_rotation_z(std::f64::consts::FRAC_PI_4);
    let bounds = mesh.get_axis_aligned_bounding_box_transformed(rotate);
    let half_diagonal = 50f64.sqrt();
    assert!((bounds.max_xyz.x - half_diagonal).abs() < 1e-5);
    assert!((bounds.min_xyz.y + half_diagonal).abs() < 1e-5);
    assert!((bounds.max_xyz.z - 5.0).abs() < 1e-6);
    // asking again gives the cached answer
    assert_eq!(
        mesh.get_axis_aligned_bounding_box_transformed(rotate),
        bounds
    );

    mesh.translate(Vector3::new(0.0, 0.0, 10.0));
    let moved = mesh.get_axis_aligned_bounding_box_transformed(rotate);
    assert!((moved.max_xyz.z - 15.0).abs() < 1e-6);
    assert!((moved.max_xyz.x - half_diagonal).abs() < 1e-5);
}

/// Whether every directed edge of `mesh` is walked once and its reverse once, and no face repeats
/// a corner: a closed solid on shared vertices, with no welding of equal positions.
fn closed_on_shared_vertices(mesh: &solidmesh_mesh::Mesh) -> bool {
    let mut walks = std::collections::HashMap::new();
    for face in mesh.faces.iter() {
        let [a, b, c] = [face.v0, face.v1, face.v2];
        if a == b || b == c || c == a {
            return false;
        }
        for edge in [(a, b), (b, c), (c, a)] {
            *walks.entry(edge).or_insert(0) += 1;
        }
    }
    walks
        .iter()
        .all(|(&(a, b), &count)| count == 1 && walks.get(&(b, a)) == Some(&1))
}

/// A hull is a closed solid whose faces share its corners - one vertex per hull point - so
/// whatever is built from it (the bevel's padded bound, lay flat, the bounds) works on a solid,
/// not on separate triangles that only meet by position.
#[test]
fn the_hull_is_a_closed_solid_on_one_vertex_per_hull_point() {
    let cube = platonic_solids::create_cube(10.0, 10.0, 10.0);
    let hull = hull_arc(cube.get_convex_hull(false));
    assert_eq!(hull.vertices.len(), 8, "a cube's hull has its 8 corners");
    assert_eq!(hull.faces.len(), 12);
    assert!(
        closed_on_shared_vertices(&hull),
        "the cube's hull is closed"
    );
    assert!(
        (hull.get_volume() - 1000.0).abs() < 1e-6,
        "the hull holds the cube"
    );

    let ball = solidmesh_mesh::solids::platonic_solids::create_icosahedron(10.0);
    let hull = hull_arc(ball.get_convex_hull(false));
    assert_eq!(
        hull.vertices.len(),
        12,
        "an icosahedron's hull has its 12 corners"
    );
    assert!(
        closed_on_shared_vertices(&hull),
        "the icosahedron's hull is closed"
    );
    assert!(
        (hull.get_volume() - ball.get_volume()).abs() < 1e-3,
        "the hull holds the icosahedron"
    );
}

/// A mesh over `points` with a few faces (enough for a hull to be built; the hull reads every
/// vertex whatever the faces are).
fn mesh_over(points: &[[f64; 3]]) -> solidmesh_mesh::Mesh {
    let mut mesh = solidmesh_mesh::Mesh::new();
    for p in points {
        mesh.vertices.push(Vector3Float::new(p[0], p[1], p[2]));
    }
    for i in 0..4 {
        mesh.faces.add(i, i + 1, i + 2, &mesh.vertices);
    }
    mesh
}

/// [`solidmesh_mesh::processors::ConvexHullSteps`] asked for a single vertex visit at a time (the
/// finest stepping, every piece cut as small as it can be) builds and caches the hull
/// `get_convex_hull` builds in one piece, vertex for vertex and face for face, on a scattered
/// cloud and on a grid full of coplanar points.
#[test]
fn a_hull_stepped_one_visit_at_a_time_is_the_one_piece_hull() {
    let cloud: Vec<[f64; 3]> = (0..200i64)
        .map(|i| {
            [
                ((i * 7919) % 1000) as f64 / 37.0,
                ((i * 104729) % 997) as f64 / 41.0,
                ((i * 1299709) % 991) as f64 / 43.0,
            ]
        })
        .collect();
    let mut grid = Vec::new();
    for x in 0..5 {
        for y in 0..5 {
            for z in 0..5 {
                grid.push([x as f64, y as f64, z as f64]);
            }
        }
    }
    for (name, points) in [("cloud", cloud), ("grid", grid)] {
        let whole_mesh = mesh_over(&points);
        let whole = hull_arc(whole_mesh.get_convex_hull(false));
        let stepped_mesh = mesh_over(&points);
        let mut steps = solidmesh_mesh::processors::ConvexHullSteps::new(&stepped_mesh);
        let mut count = 0;
        let stepped = loop {
            count += 1;
            if let Some(hull) = steps.step(1) {
                break hull_arc(hull);
            }
        };
        assert!(count > 10, "{name}: built in {count} steps");
        assert_eq!(stepped.vertices, whole.vertices, "{name}");
        assert_eq!(stepped.faces.len(), whole.faces.len(), "{name}");
        for (a, b) in stepped.faces.iter().zip(whole.faces.iter()) {
            assert_eq!((a.v0, a.v1, a.v2), (b.v0, b.v1, b.v2), "{name}");
        }
        assert!(stepped_mesh.has_cached_convex_hull(), "{name}: cached");
    }
}

/// The piece that reads a mesh's last vertices also copies every distinct vertex into the hull's
/// setup, and [`solidmesh_mesh::processors::ConvexHullSteps::next_work`] predicts that copy, so the
/// picture's camera charges it before it runs instead of finding it in the step's tally after
/// (about 190,000 visits on a 48,000-vertex mesh). What any piece collecting the vertices does
/// beyond the prediction is at most the few distinct vertices it adds itself.
#[test]
fn the_last_vertex_read_s_prediction_covers_the_hull_setup_copy() {
    // 200 distinct points, read 10 a piece (each read counts 6 visits): 20 pieces.
    const WORK: usize = 60;
    let points: Vec<[f64; 3]> = (0..200i64)
        .map(|i| {
            [
                ((i * 7919) % 1000) as f64 / 37.0,
                ((i * 104729) % 997) as f64 / 41.0,
                ((i * 1299709) % 991) as f64 / 43.0,
            ]
        })
        .collect();
    let mesh = mesh_over(&points);
    let mut steps = solidmesh_mesh::processors::ConvexHullSteps::new(&mesh);
    let mut largest_piece = 0;
    for piece in 0..20 {
        let predicted = steps.next_work(WORK);
        let before = steps.work();
        assert!(
            steps.step(WORK).is_none(),
            "piece {piece} finished the hull"
        );
        let did = steps.work() - before;
        largest_piece = largest_piece.max(did);
        assert!(
            did <= predicted + 10,
            "piece {piece} did {did} visits, {predicted} predicted"
        );
    }
    assert!(
        largest_piece >= WORK + 200,
        "the last piece copies the 200 points"
    );
}
