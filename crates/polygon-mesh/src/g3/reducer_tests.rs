//! Tests for the `Reducer` (`reducer`), `QuadricError` (`quadric_error`),
//! `IndexPriorityQueue` (`index_priority_queue`) and `MeshProjectionTarget`
//! (`mesh_projection_target`). The Reduce operation's own tests (`DecimateObject3D`) live in
//! mattercad-model.

use super::dmesh3::DMesh3;
use super::dmesh3_tests::check_valid;
use super::index_priority_queue::IndexPriorityQueue;
use super::mesh_constraints::ProjectionTarget;
use super::mesh_projection_target::MeshProjectionTarget;
use super::minimal_hole_fill_tests::open_box;
use super::quadric_error::QuadricError;
use super::reducer::{Reducer, TargetProjectionMode};
use super::tri_distance::{distance_sqr_closest, tri_distance_sqr};
use super::DMeshAABBTree3;

/// A closed UV sphere of radius `r`: `rings` latitude bands, `segments` around, poles shared.
fn uv_sphere(r: f64, rings: i32, segments: i32) -> DMesh3 {
    let mut m = DMesh3::default();
    let top = m.append_vertex([0.0, 0.0, r]);
    for i in 1..rings {
        let phi = std::f64::consts::PI * i as f64 / rings as f64;
        for j in 0..segments {
            let theta = 2.0 * std::f64::consts::PI * j as f64 / segments as f64;
            m.append_vertex([
                r * phi.sin() * theta.cos(),
                r * phi.sin() * theta.sin(),
                r * phi.cos(),
            ]);
        }
    }
    let bottom = m.append_vertex([0.0, 0.0, -r]);
    let ring = |i: i32, j: i32| 1 + (i - 1) * segments + (j % segments);
    for j in 0..segments {
        m.append_triangle(top, ring(1, j), ring(1, j + 1));
        m.append_triangle(bottom, ring(rings - 1, j + 1), ring(rings - 1, j));
    }
    for i in 1..rings - 1 {
        for j in 0..segments {
            let (a, b, c, d) = (
                ring(i, j),
                ring(i, j + 1),
                ring(i + 1, j),
                ring(i + 1, j + 1),
            );
            m.append_triangle(a, c, d);
            m.append_triangle(a, d, b);
        }
    }
    m
}

#[test]
fn rust_only_index_priority_queue_dequeues_lowest_first_and_updates_in_place() {
    let mut q = IndexPriorityQueue::new(10);
    for (id, p) in [(3, 5.0), (7, 1.0), (1, 3.0), (9, 4.0), (0, 2.0)] {
        q.insert(id, p);
    }
    assert!(q.is_valid_queue());
    assert!(q.contains(9) && !q.contains(2));
    q.update(9, 0.5);
    assert_eq!(q.first(), 9);
    assert_eq!(q.get_priority(9), 0.5);
    q.remove(1);
    let mut order = Vec::new();
    while q.count() > 0 {
        order.push(q.dequeue());
    }
    assert_eq!(order, vec![9, 7, 0, 3]);
}

/// A dequeued id is not reported as queued once an insert reuses its old slot; otherwise the
/// reducer would re-prioritize a stranger's edge and never re-queue the dequeued one.
#[test]
fn rust_only_a_dequeued_id_is_not_queued_after_its_slot_is_reused() {
    let mut q = IndexPriorityQueue::new(4);
    q.insert(0, 1.0);
    q.insert(1, 2.0);
    assert_eq!(q.dequeue(), 0);
    q.insert(2, 3.0); // takes slot 2, where id 0 was last mapped
    assert!(!q.contains(0), "id 0 was dequeued");
    assert!(q.contains(1) && q.contains(2));
    // so the Reducer re-inserts it instead of re-prioritizing id 2
    q.insert(0, 0.5);
    assert_eq!(q.dequeue(), 0);
    assert_eq!(q.dequeue(), 1);
    assert_eq!(q.dequeue(), 2);
}

#[test]
fn rust_only_a_plane_quadric_measures_squared_distance_and_three_planes_meet_at_a_point() {
    let q = QuadricError::from_plane([0.0, 0.0, 1.0], [5.0, -2.0, 3.0]);
    assert_eq!(q.evaluate([1.0, 1.0, 3.0]), 0.0);
    assert_eq!(q.evaluate([1.0, 1.0, 5.0]), 4.0);
    assert!(
        q.optimal_point().is_none(),
        "one plane has no single minimum"
    );

    let mut corner = QuadricError::ZERO;
    corner.add(1.0, &q);
    corner.add(
        2.0,
        &QuadricError::from_plane([1.0, 0.0, 0.0], [7.0, 0.0, 0.0]),
    );
    let corner = QuadricError::sum(
        &corner,
        &QuadricError::from_plane([0.0, 1.0, 0.0], [0.0, -4.0, 0.0]),
    );
    assert_eq!(corner.optimal_point(), Some([7.0, -4.0, 3.0]));
    assert_eq!(corner.evaluate([7.0, -4.0, 3.0]), 0.0);
}

#[test]
fn rust_only_closest_point_on_a_triangle_matches_its_distance() {
    let tri = [[0.0, 0.0, 0.0], [4.0, 0.0, 0.0], [0.0, 4.0, 0.0]];
    for (p, expect) in [
        ([1.0, 1.0, 2.0], [1.0, 1.0, 0.0]),   // above the interior
        ([-3.0, -3.0, 0.0], [0.0, 0.0, 0.0]), // past the corner
        ([3.0, 3.0, 1.0], [2.0, 2.0, 0.0]),   // past the long edge
        ([2.0, -5.0, 0.0], [2.0, 0.0, 0.0]),  // below an edge
    ] {
        let (d2, closest) = distance_sqr_closest(&tri, p);
        assert_eq!(closest, expect, "{p:?}");
        assert!((d2 - tri_distance_sqr(&tri, p)).abs() < 1e-12);
        let e = [p[0] - expect[0], p[1] - expect[1], p[2] - expect[2]];
        assert!((d2 - (e[0] * e[0] + e[1] * e[1] + e[2] * e[2])).abs() < 1e-12);
    }
}

#[test]
fn rust_only_a_mesh_projection_target_moves_points_onto_the_surface() {
    let sphere = uv_sphere(10.0, 12, 16);
    let tree = DMeshAABBTree3::new(&sphere);
    let target = MeshProjectionTarget::new(&tree);
    let projected = target.project([0.3, 0.2, 25.0], -1);
    let nearest = tree.find_nearest_triangle(projected, f64::MAX);
    assert!(tri_distance_sqr(&sphere.get_tri_vertices(nearest as usize), projected) < 1e-18);
    assert!(projected[2] > 9.0 && projected[2] <= 10.0, "{projected:?}");
}

#[test]
fn rust_only_reducing_a_sphere_reaches_the_triangle_count_and_stays_closed() {
    let mut sphere = uv_sphere(10.0, 16, 24);
    let start = sphere.triangle_count();
    assert_eq!(start, 2 * 24 * 15);
    let target = (start / 4) as i32;

    let mut reducer = Reducer::new(&mut sphere);
    reducer.reduce_to_triangle_count(target);
    let reduced = reducer.mesh();
    check_valid(reduced);
    assert!(reduced.is_closed());
    // every collapse of a closed manifold removes two triangles
    assert!(reduced.triangle_count() as i32 <= target);
    assert!(reduced.triangle_count() as i32 >= target - 1);
    // the quadric points stay near the sphere
    for vid in reduced.vertex_indices() {
        let v = reduced.vertex(vid);
        let r = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
        assert!((r - 10.0).abs() < 0.5, "vertex {vid} at radius {r}");
    }
}

#[test]
fn rust_only_reducing_by_vertex_count_and_edge_length() {
    let mut sphere = uv_sphere(10.0, 12, 16);
    let mut reducer = Reducer::new(&mut sphere);
    reducer.reduce_to_vertex_count(60);
    assert!(reducer.mesh().vertex_count() <= 60);
    check_valid(reducer.mesh());

    let mut sphere = uv_sphere(10.0, 12, 16);
    let start = sphere.triangle_count();
    let mut reducer = Reducer::new(&mut sphere);
    // the edges around the rings next to the poles are about 1.02 long, the edges to the poles
    // about 2.6
    reducer.reduce_to_edge_length(3.0);
    assert!(reducer.mesh().triangle_count() < start);
    check_valid(reducer.mesh());

    let mut sphere = uv_sphere(10.0, 12, 16);
    let mut reducer = Reducer::new(&mut sphere);
    reducer.fast_collapse_pass(3.0, 3, true);
    assert!(reducer.mesh().triangle_count() < start);
    check_valid(reducer.mesh());
}

#[test]
fn rust_only_an_open_mesh_keeps_its_boundary() {
    let mut open = open_box(4);
    let boundary_before: Vec<[f64; 3]> = open
        .boundary_edge_indices()
        .flat_map(|e| open.get_edge_v(e))
        .map(|v| open.vertex(v))
        .collect();
    let mut reducer = Reducer::new(&mut open);
    reducer.reduce_to_triangle_count(10);
    let reduced = reducer.mesh();
    check_valid(reduced);
    // boundary vertices are only ever collapsed onto boundary positions
    for e in reduced.boundary_edge_indices() {
        for v in reduced.get_edge_v(e) {
            assert!(
                boundary_before.contains(&reduced.vertex(v)),
                "boundary vertex {v} moved"
            );
        }
    }
}

#[test]
fn rust_only_inline_projection_keeps_every_vertex_on_the_original_surface() {
    let mut sphere = uv_sphere(10.0, 16, 24);
    let original = sphere.clone();
    let tree = DMeshAABBTree3::new(&original);
    let target = MeshProjectionTarget::new(&tree);
    let mut reducer = Reducer::new(&mut sphere);
    reducer.set_projection_target(&target);
    reducer.projection_mode = TargetProjectionMode::Inline;
    reducer.reduce_to_triangle_count(200);
    let reduced = reducer.mesh();
    check_valid(reduced);
    assert!(reduced.triangle_count() <= 200);
    for vid in reduced.vertex_indices() {
        let v = reduced.vertex(vid);
        let t = tree.find_nearest_triangle(v, f64::MAX);
        let d2 = tri_distance_sqr(&original.get_tri_vertices(t as usize), v);
        assert!(d2 < 1e-12, "vertex {vid} is {d2} off the surface");
    }
}
