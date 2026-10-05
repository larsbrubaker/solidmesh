//! Rust-only tests for the dynamic `DMesh3` (`dmesh3*.rs`). geometry3Sharp ships no tests for
//! these edits in agg-sharp, so these pin the topology invariants and the C# id-reuse and
//! one-ring orders the repair depends on.

use super::dmesh3::{DMesh3, MeshResult, INVALID_ID, NON_MANIFOLD_ID};

/// Two triangles sharing edge [0,2]: a unit square in z = 0, wound +z.
fn square() -> DMesh3 {
    let mut m = DMesh3::default();
    for p in [
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [1.0, 1.0, 0.0],
        [0.0, 1.0, 0.0],
    ] {
        m.append_vertex(p);
    }
    assert_eq!(m.append_triangle(0, 1, 2), 0);
    assert_eq!(m.append_triangle(0, 2, 3), 1);
    m
}

/// A closed tetrahedron.
fn tetrahedron() -> DMesh3 {
    let mut m = DMesh3::default();
    for p in [
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
    ] {
        m.append_vertex(p);
    }
    m.append_triangle(0, 2, 1);
    m.append_triangle(0, 1, 3);
    m.append_triangle(1, 2, 3);
    m.append_triangle(2, 0, 3);
    m
}

/// Every edge's triangles contain both its vertices, every triangle's edges are its edges,
/// and every vertex's edge list holds exactly the edges on it.
pub(super) fn check_valid(m: &DMesh3) {
    for eid in m.edge_indices() {
        let [a, b, t0, t1] = m.get_edge(eid);
        assert!(a < b);
        assert!(m.vtx_edges(a).contains(&eid) && m.vtx_edges(b).contains(&eid));
        for t in [t0, t1] {
            if t != INVALID_ID {
                assert!(m.tri_has_v(t, a) && m.tri_has_v(t, b));
                assert!(m.get_tri_edges(t).contains(&eid));
            }
        }
    }
    for tid in m.triangle_indices() {
        let t = m.get_triangle(tid);
        for j in 0..3 {
            let e = m.get_tri_edges(tid)[j];
            assert_eq!(e, m.find_edge(t[j], t[(j + 1) % 3]));
        }
    }
    for vid in m.vertex_indices() {
        for eid in m.vtx_edges(vid) {
            assert!(m.is_edge(eid));
        }
    }
}

#[test]
fn rust_only_append_refuses_a_third_triangle_on_an_edge() {
    let mut m = square();
    m.append_vertex([1.0, 1.0, 1.0]);
    assert_eq!(m.append_triangle(0, 2, 4), NON_MANIFOLD_ID);
    assert_eq!(m.append_triangle(0, 0, 4), INVALID_ID);
    assert_eq!(m.edge_count(), 5);
    assert!(!m.is_closed());
    assert_eq!(m.boundary_edge_indices().count(), 4);
    check_valid(&m);
    assert!(tetrahedron().is_closed());
}

#[test]
fn rust_only_remove_triangle_frees_ids_and_reuses_them_last_freed_first() {
    let mut m = square();
    assert_eq!(m.remove_triangle(1, true, false), MeshResult::Ok);
    assert_eq!(m.triangle_count(), 1);
    // vertex 3 lost its last triangle and goes with it; edges [0,2] stays (on triangle 0)
    assert!(!m.is_vertex(3));
    assert_eq!(m.edge_count(), 3);
    assert!(m.is_boundary_edge(m.find_edge(0, 2)));
    check_valid(&m);
    // the freed vertex and triangle ids come back before new ones
    assert_eq!(m.append_vertex([0.0, 1.0, 0.0]), 3);
    assert_eq!(m.append_triangle(0, 2, 3), 1);
    check_valid(&m);
    let mesh = m.to_mesh();
    assert_eq!((mesh.vertices.len(), mesh.faces.len()), (4, 2));
}

#[test]
fn rust_only_flip_split_and_poke_keep_topology_valid() {
    let mut m = square();
    let e02 = m.find_edge(0, 2);
    let (result, flip) = m.flip_edge(e02).unwrap();
    assert_eq!(result, MeshResult::Ok);
    assert_eq!((flip.ov0, flip.ov1), (1, 3));
    assert_eq!(m.find_edge(0, 2), INVALID_ID);
    assert_ne!(m.find_edge(1, 3), INVALID_ID);
    check_valid(&m);

    let e13 = m.find_edge(1, 3);
    let (result, split) = m.split_edge(e13, 0.5);
    assert_eq!(result, MeshResult::Ok);
    assert_eq!(m.vertex(split.v_new), [0.5, 0.5, 0.0]);
    assert_eq!(m.triangle_count(), 4);
    check_valid(&m);

    let (result, poke) = m.poke_triangle(0, [1.0 / 3.0; 3]);
    assert_eq!(result, MeshResult::Ok);
    assert_eq!(m.triangle_count(), 6);
    assert_eq!(m.get_vtx_edge_count(poke.new_vid), 3);
    check_valid(&m);
}

#[test]
fn rust_only_collapse_edge_removes_the_vertex_and_its_two_triangles() {
    let mut m = square();
    let e02 = m.find_edge(0, 2);
    let (_, split) = m.split_edge(e02, 0.5);
    let center = split.v_new;
    assert_eq!(m.triangle_count(), 4);
    // the center onto a corner takes the two triangles on that edge and undoes the split
    let (result, info) = m.collapse_edge(0, center);
    assert_eq!(result, MeshResult::Ok);
    assert!(!info.is_boundary);
    assert!(!m.is_vertex(center));
    assert_eq!(m.triangle_count(), 2);
    assert_ne!(m.find_edge(0, 2), INVALID_ID);
    check_valid(&m);

    let mut t = tetrahedron();
    let (result, _) = t.collapse_edge(0, 3);
    assert_eq!(result, MeshResult::FailedCollapseTetrahedron);
    check_valid(&t);

    // a boundary edge collapse on the square takes one triangle
    let mut m = square();
    let (result, info) = m.collapse_edge(0, 1);
    assert_eq!(result, MeshResult::Ok);
    assert!(info.is_boundary);
    assert!(!m.is_vertex(1));
    assert_eq!(m.triangle_count(), 1);
    check_valid(&m);
}

#[test]
fn rust_only_merge_edges_closes_a_crack_between_two_triangles() {
    // two triangles whose shared edge was duplicated: [0,1,2] and [3,2',1'] with 4 == 2, 5 == 1
    let mut m = DMesh3::default();
    for p in [
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [1.0, 1.0, 0.0],
        [0.0, 1.0, 0.0],
        [1.0, 0.0, 0.0],
    ] {
        m.append_vertex(p);
    }
    m.append_triangle(0, 1, 2);
    m.append_triangle(3, 4, 5);
    let keep = m.find_edge(1, 2);
    let discard = m.find_edge(4, 5);
    let (result, info) = m.merge_edges(keep, discard);
    assert_eq!(result, MeshResult::Ok);
    assert_eq!(m.vertex_count(), 4);
    assert!(!m.is_boundary_edge(keep));
    assert!(!m.is_edge(discard));
    assert_eq!(info.v_kept, [1, 2]);
    check_valid(&m);
}

#[test]
fn rust_only_bowtie_and_compact_copy() {
    // two triangles meeting only at vertex 0
    let mut m = DMesh3::default();
    for p in [
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [-1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
    ] {
        m.append_vertex(p);
    }
    m.append_triangle(0, 1, 2);
    m.append_triangle(0, 3, 4);
    assert!(m.is_bowtie_vertex(0));
    assert!(!m.is_bowtie_vertex(1));
    assert!(!square().is_bowtie_vertex(0));

    m.remove_triangle(0, true, false);
    assert!(!m.is_compact());
    let c = m.compact_copy();
    assert!(c.is_compact());
    assert_eq!((c.vertex_count(), c.triangle_count()), (3, 1));
    assert_eq!(c.get_triangle(0), [0, 1, 2]);
    assert_eq!(c.vertex(1), [-1.0, 0.0, 0.0]);
}
