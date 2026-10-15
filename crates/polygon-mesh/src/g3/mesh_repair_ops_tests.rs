//! Tests for the crack-closing pieces of the mesh repair (`merge_coincident_edges`,
//! `point_set_hashtable`, `dynamic_priority_queue`).

use super::dmesh3::DMesh3;
use super::dynamic_priority_queue::DynamicPriorityQueue;
use super::merge_coincident_edges::MergeCoincidentEdges;
use super::mesh_boundary_loops::MeshBoundaryLoops;
use super::point_set_hashtable::BoundaryEdgeMidpointHashtable;

/// A unit cube wound outward whose six faces each have their own four vertices, so every edge
/// between faces is a crack of two coincident boundary edges.
fn cracked_cube() -> DMesh3 {
    let mut m = DMesh3::default();
    let corner =
        |i: usize| -> [f64; 3] { [(i & 1) as f64, ((i >> 1) & 1) as f64, ((i >> 2) & 1) as f64] };
    // outward-wound quads over the corner numbering x + 2y + 4z
    let quads = [
        [0, 2, 3, 1],
        [4, 5, 7, 6],
        [0, 1, 5, 4],
        [2, 6, 7, 3],
        [0, 4, 6, 2],
        [1, 3, 7, 5],
    ];
    for q in quads {
        let v: Vec<i32> = q.iter().map(|&c| m.append_vertex(corner(c))).collect();
        m.append_triangle(v[0], v[1], v[2]);
        m.append_triangle(v[0], v[2], v[3]);
    }
    m
}

#[test]
fn rust_only_priority_queue_dequeues_lowest_first() {
    let mut q: DynamicPriorityQueue<i32> = DynamicPriorityQueue::new();
    for (value, priority) in [(10, 3.0), (11, 1.0), (12, 2.0), (13, 1.0), (14, 5.0)] {
        q.enqueue(value, priority);
    }
    assert_eq!(q.count(), 5);
    let mut out = Vec::new();
    while let Some(v) = q.dequeue() {
        out.push(v);
    }
    // sift-up only stops below a strictly lower priority, so 13 climbs past the equal 11
    // to the root and comes out first
    assert_eq!(out, vec![13, 11, 12, 10, 14]);
}

#[test]
fn rust_only_ball_query_finds_coincident_edge_midpoints() {
    let m = cracked_cube();
    let hash = BoundaryEdgeMidpointHashtable::build(&m, 64);
    // the bottom face's first edge [v0, v1] has its twin on a side face
    let eid = m.find_edge(0, 1);
    let hits = hash
        .find_in_ball(&m, [0.0, 0.5, 0.0], 1e-6)
        .expect("radius is below a cell");
    assert!(hits.contains(&eid));
    assert_eq!(hits.len(), 2);
    // a radius above the cell size is refused: the search only visits neighbouring cells
    assert!(hash.find_in_ball(&m, [0.0, 0.5, 0.0], 1.0).is_err());
}

#[test]
fn rust_only_merge_coincident_edges_closes_a_cube_split_into_faces() {
    let mut m = cracked_cube();
    assert_eq!(m.vertex_count(), 24);
    assert_eq!(MeshBoundaryLoops::new(&m).loops.len(), 6);
    let merge = MergeCoincidentEdges::default();
    assert!(merge.apply(&mut m).unwrap());
    assert!(m.is_closed());
    assert_eq!(m.vertex_count(), 8);
    assert_eq!(m.triangle_count(), 12);
}
