//! geometry3Sharp `DMesh3.SetTriangle` (`mesh/DMesh3_edge_operators.cs`), the vertex copy
//! `AppendVertex(DMesh3, int)` and the boundary-edge queries `GetOrientedBoundaryEdgeV` and
//! `VtxBoundaryEdges` / `VtxAllBoundaryEdges` (`mesh/DMesh3.cs`) - the pieces `MeshEditor`'s
//! bowtie split and `MeshBoundaryLoops` reach. Split from `dmesh3_edge_ops` for size.

use super::dmesh3::{DMesh3, MeshResult, INVALID_ID};
use super::dmesh3_queries::find_edge_index_in_tri;

impl DMesh3 {
    /// C# `AppendVertex(DMesh3 from, int fromVID)` with `from == this`: a new vertex at the
    /// same position (there are no normals, colors or UVs to copy here).
    pub(crate) fn append_vertex_copy(&mut self, from_vid: i32) -> i32 {
        let v = self.vertex(from_vid);
        self.append_vertex(v)
    }

    /// C# `SetTriangle(tID, newv, bRemoveIsolatedVertices)`: rewrites a triangle's vertices,
    /// rebuilding the edges that changed.
    pub(crate) fn set_triangle(
        &mut self,
        tid: i32,
        newv: [i32; 3],
        remove_isolated_vertices: bool,
    ) -> MeshResult {
        let tv = self.get_triangle(tid);
        let mut te = self.get_tri_edges(tid);
        if tv[0] == newv[0] && tv[1] == newv[1] {
            te[0] = -1;
        }
        if tv[1] == newv[1] && tv[2] == newv[2] {
            te[1] = -1;
        }
        if tv[2] == newv[2] && tv[0] == newv[0] {
            te[2] = -1;
        }

        if !self.triangles_refcount.is_valid(tid) {
            return MeshResult::FailedNotATriangle;
        }
        if !self.is_vertex(newv[0]) || !self.is_vertex(newv[1]) || !self.is_vertex(newv[2]) {
            return MeshResult::FailedNotAVertex;
        }
        if newv[0] == newv[1] || newv[0] == newv[2] || newv[1] == newv[2] {
            return MeshResult::FailedBrokenTopology;
        }
        // look up edges. if any already have two triangles, this would
        // create non-manifold geometry and so we do not allow it
        let e0 = self.find_edge(newv[0], newv[1]);
        let e1 = self.find_edge(newv[1], newv[2]);
        let e2 = self.find_edge(newv[2], newv[0]);
        if (te[0] != -1 && e0 != INVALID_ID && !self.is_boundary_edge(e0))
            || (te[1] != -1 && e1 != INVALID_ID && !self.is_boundary_edge(e1))
            || (te[2] != -1 && e2 != INVALID_ID && !self.is_boundary_edge(e2))
        {
            return MeshResult::FailedBrokenTopology;
        }

        // Remove triangle from its edges. if edge has no triangles left, then it is removed.
        for &eid in &te {
            if eid == -1 {
                // we don't need to modify this edge
                continue;
            }
            self.replace_edge_triangle(eid, tid, INVALID_ID);
            if self.edges[eid as usize][2] == INVALID_ID {
                let [a, b, _, _] = self.edges[eid as usize];
                self.vertex_edges.remove(a, eid);
                self.vertex_edges.remove(b, eid);
                self.edges_refcount.decrement(eid, 1);
            }
        }

        // Decrement vertex refcounts. If any hit 1 and we got remove-isolated flag,
        // we need to remove that vertex
        for j in 0..3 {
            let vid = tv[j];
            if vid == newv[j] {
                // we don't need to modify this vertex
                continue;
            }
            self.vertices_refcount.decrement(vid, 1);
            if remove_isolated_vertices && self.vertices_refcount.ref_count(vid) == 1 {
                self.vertices_refcount.decrement(vid, 1);
                self.vertex_edges.clear(vid);
            }
        }

        // ok now re-insert with new vertices
        for j in 0..3 {
            if newv[j] != tv[j] {
                self.triangles[tid as usize][j] = newv[j];
                self.vertices_refcount.increment(newv[j], 1);
            }
        }

        if te[0] != -1 {
            self.add_tri_edge(tid, newv[0], newv[1], 0, e0);
        }
        if te[1] != -1 {
            self.add_tri_edge(tid, newv[1], newv[2], 1, e1);
        }
        if te[2] != -1 {
            self.add_tri_edge(tid, newv[2], newv[0], 2, e2);
        }
        MeshResult::Ok
    }

    /// C# `GetOrientedBoundaryEdgeV`: a boundary edge's vertices in the order its triangle
    /// runs them, or `[INVALID_ID; 2]` for anything else.
    pub(crate) fn get_oriented_boundary_edge_v(&self, eid: i32) -> [i32; 2] {
        if self.edges_refcount.is_valid(eid) {
            let [a, b, t0, t1] = self.edges[eid as usize];
            if t1 == INVALID_ID {
                let tri = self.triangles[t0 as usize];
                let ai = find_edge_index_in_tri(a, b, &tri) as usize;
                return [tri[ai], tri[(ai + 1) % 3]];
            }
        }
        [INVALID_ID; 2]
    }

    /// C# `VtxAllBoundaryEdges`: the boundary edges at a vertex, in its edge-list order (the
    /// first two of these are C# `VtxBoundaryEdges`' `e0`, `e1`, and its count the length).
    pub(crate) fn vtx_all_boundary_edges(&self, vid: i32) -> Vec<i32> {
        self.vertex_edges
            .iter(vid)
            .filter(|&eid| self.edges[eid as usize][3] == INVALID_ID)
            .collect()
    }
}
