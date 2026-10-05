//! geometry3Sharp `mesh/MeshIterators.cs`, the iterators the mesh repair and hole fillers use:
//! boundary and interior vertices and edges, the vertices of boundary edges, and bowtie
//! vertices. Each walks ids `0..Max*ID` in order, as C# does.

// The mesh repair (geometry3Sharp `MeshAutoRepair`) is the consumer of these and lands in a
// later slice; until then only the tests reach some of them.
#![allow(dead_code)]

use super::dmesh3::DMesh3;

/// C# `MeshIterators.BoundaryVertices`.
pub(crate) fn boundary_vertices(mesh: &DMesh3) -> Vec<i32> {
    (0..mesh.max_vertex_id())
        .filter(|&i| mesh.is_vertex(i) && mesh.is_boundary_vertex(i))
        .collect()
}

/// C# `MeshIterators.InteriorVertices`.
pub(crate) fn interior_vertices(mesh: &DMesh3) -> Vec<i32> {
    (0..mesh.max_vertex_id())
        .filter(|&i| mesh.is_vertex(i) && !mesh.is_boundary_vertex(i))
        .collect()
}

/// C# `MeshIterators.BoundaryEdgeVertices`: both vertices of each boundary edge, so a vertex
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

/// C# `MeshIterators.BoundaryEdges`.
pub(crate) fn boundary_edges(mesh: &DMesh3) -> Vec<i32> {
    (0..mesh.max_edge_id())
        .filter(|&i| mesh.is_edge(i) && mesh.is_boundary_edge(i))
        .collect()
}

/// C# `MeshIterators.InteriorEdges`.
pub(crate) fn interior_edges(mesh: &DMesh3) -> Vec<i32> {
    (0..mesh.max_edge_id())
        .filter(|&i| mesh.is_edge(i) && !mesh.is_boundary_edge(i))
        .collect()
}

/// C# `MeshIterators.BowtieVertices`.
pub(crate) fn bowtie_vertices(mesh: &DMesh3) -> Vec<i32> {
    (0..mesh.max_vertex_id())
        .filter(|&i| mesh.is_vertex(i) && mesh.is_bowtie_vertex(i))
        .collect()
}
