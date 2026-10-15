//! Mesh element lists (derived from geometry3Sharp) the mesh repair and hole fillers use:
//! boundary and interior vertices and edges, the vertices of boundary edges, and bowtie
//! vertices. Each walks ids `0..max_*_id()` in order, so results come out in id order.

use super::dmesh3::DMesh3;

#[allow(dead_code)] // mesh API only the tests call so far
/// The live vertices on the mesh boundary.
pub(crate) fn boundary_vertices(mesh: &DMesh3) -> Vec<i32> {
    (0..mesh.max_vertex_id())
        .filter(|&i| mesh.is_vertex(i) && mesh.is_boundary_vertex(i))
        .collect()
}

/// The live vertices not on the mesh boundary.
pub(crate) fn interior_vertices(mesh: &DMesh3) -> Vec<i32> {
    (0..mesh.max_vertex_id())
        .filter(|&i| mesh.is_vertex(i) && !mesh.is_boundary_vertex(i))
        .collect()
}

/// Both vertices of each boundary edge, so a vertex
/// comes once per boundary edge it is on.
pub(crate) fn boundary_edge_vertices(mesh: &DMesh3) -> Vec<i32> {
    let mut out = Vec::new();
    for i in 0..mesh.max_edge_id() {
        if mesh.is_edge(i) && mesh.is_boundary_edge(i) {
            let [a, b] = mesh.get_edge_v(i);
            out.push(a);
            out.push(b);
        }
    }
    out
}

/// The live boundary edges.
pub(crate) fn boundary_edges(mesh: &DMesh3) -> Vec<i32> {
    (0..mesh.max_edge_id())
        .filter(|&i| mesh.is_edge(i) && mesh.is_boundary_edge(i))
        .collect()
}

#[allow(dead_code)] // mesh API only the tests call so far
/// The live edges with a triangle on both sides.
pub(crate) fn interior_edges(mesh: &DMesh3) -> Vec<i32> {
    (0..mesh.max_edge_id())
        .filter(|&i| mesh.is_edge(i) && !mesh.is_boundary_edge(i))
        .collect()
}

/// The live vertices where separate fans of triangles meet.
pub(crate) fn bowtie_vertices(mesh: &DMesh3) -> Vec<i32> {
    (0..mesh.max_vertex_id())
        .filter(|&i| mesh.is_vertex(i) && mesh.is_bowtie_vertex(i))
        .collect()
}
