//! geometry3Sharp's `EdgeLoop` and `EdgeSpan` (`mesh/EdgeLoop.cs`, `mesh/EdgeSpan.cs`) as
//! `MeshBoundaryLoops` builds them and the hole fillers read them - the vertex and edge lists,
//! and the conversions between them - plus `MeshValidation.IsBoundaryLoop`
//! (`queries/MeshValidation.cs`). The geometric helpers of those classes are not reached.

// The mesh repair (geometry3Sharp `MeshAutoRepair`) is the consumer of these and lands in a
// later slice; until then only the tests reach some of them.
#![allow(dead_code)]

use super::dmesh3::{DMesh3, INVALID_ID};

/// g3 `EdgeLoop`: a closed chain of vertices with the edge after each.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub(crate) struct EdgeLoop {
    pub(crate) vertices: Vec<i32>,
    pub(crate) edges: Vec<i32>,
    pub(crate) bowtie_vertices: Vec<i32>,
}

impl EdgeLoop {
    /// C# `VertexCount`.
    pub(crate) fn vertex_count(&self) -> usize {
        self.vertices.len()
    }

    /// C# `EdgeLoop.VertexLoopToEdgeLoop`.
    pub(crate) fn vertex_loop_to_edge_loop(mesh: &DMesh3, vertex_loop: &[i32]) -> Vec<i32> {
        let nv = vertex_loop.len();
        (0..nv)
            .map(|i| mesh.find_edge(vertex_loop[i], vertex_loop[(i + 1) % nv]))
            .collect()
    }
}

/// g3 `EdgeSpan`: an open chain, one more vertex than edges.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub(crate) struct EdgeSpan {
    pub(crate) vertices: Vec<i32>,
    pub(crate) edges: Vec<i32>,
    pub(crate) bowtie_vertices: Vec<i32>,
}

/// C# `IndexUtil.find_shared_edge_v`.
fn find_shared_edge_v(ev0: [i32; 2], ev1: [i32; 2]) -> i32 {
    if ev0[0] == ev1[0] || ev0[0] == ev1[1] {
        ev0[0]
    } else if ev0[1] == ev1[0] || ev0[1] == ev1[1] {
        ev0[1]
    } else {
        INVALID_ID
    }
}

/// C# `IndexUtil.find_edge_other_v`.
fn find_edge_other_v(ev: [i32; 2], v: i32) -> i32 {
    if ev[0] == v {
        ev[1]
    } else if ev[1] == v {
        ev[0]
    } else {
        INVALID_ID
    }
}

impl EdgeSpan {
    /// C# `EdgeSpan.FromEdges`: the vertices recovered from consecutive shared ends.
    pub(crate) fn from_edges(mesh: &DMesh3, edges: &[i32]) -> EdgeSpan {
        let edges = edges.to_vec();
        let mut vertices = vec![0; edges.len() + 1];
        let start_ev = mesh.get_edge_v(edges[0]);
        let mut prev_ev = start_ev;
        if edges.len() > 1 {
            for i in 1..edges.len() {
                let next_ev = mesh.get_edge_v(edges[i]);
                vertices[i] = find_shared_edge_v(prev_ev, next_ev);
                prev_ev = next_ev;
            }
            vertices[0] = find_edge_other_v(start_ev, vertices[1]);
            let n = vertices.len();
            vertices[n - 1] = find_edge_other_v(prev_ev, vertices[n - 2]);
        } else {
            vertices[0] = start_ev[0];
            vertices[1] = start_ev[1];
        }
        EdgeSpan {
            vertices,
            edges,
            bowtie_vertices: Vec::new(),
        }
    }

    /// C# `EdgeSpan.VerticesToEdges`.
    pub(crate) fn vertices_to_edges(mesh: &DMesh3, vertex_span: &[i32]) -> Vec<i32> {
        vertex_span
            .windows(2)
            .map(|w| mesh.find_edge(w[0], w[1]))
            .collect()
    }
}

/// g3 `ValidationStatus`, the values `IsBoundaryLoop` returns.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum ValidationStatus {
    Ok,
    NotBoundaryVertex,
    NotBoundaryEdge,
    VerticesNotConnectedByEdge,
    IncorrectLoopOrientation,
}

/// C# `MeshValidation.IsBoundaryLoop`: every vertex on the boundary, and each consecutive pair
/// a boundary edge running in the loop's direction.
pub(crate) fn is_boundary_loop(mesh: &DMesh3, lp: &EdgeLoop) -> ValidationStatus {
    let n = lp.vertices.len();
    if lp.vertices.iter().any(|&v| !mesh.is_boundary_vertex(v)) {
        return ValidationStatus::NotBoundaryVertex;
    }
    for i in 0..n {
        let a = lp.vertices[i];
        let b = lp.vertices[(i + 1) % n];
        let eid = mesh.find_edge(a, b);
        if eid == INVALID_ID {
            return ValidationStatus::VerticesNotConnectedByEdge;
        }
        if !mesh.is_boundary_edge(eid) {
            return ValidationStatus::NotBoundaryEdge;
        }
        if mesh.get_oriented_boundary_edge_v(eid) != [a, b] {
            return ValidationStatus::IncorrectLoopOrientation;
        }
    }
    ValidationStatus::Ok
}
