//! geometry3Sharp `DMesh3.CollapseEdge` and `DMesh3.MergeEdges` (`mesh/DMesh3_edge_operators.cs`):
//! the two edits that fuse vertices, and so the two the repair's crack closing and degenerate-edge
//! cleanup rest on. Split from `dmesh3_edge_ops` for size; same rules - C#'s order of list and
//! refcount updates, and C#'s DEBUG-only `debug_fail` checks dropped.

// The mesh repair (geometry3Sharp `MeshAutoRepair`) is the consumer of these edits and lands in
// the next slice; until then only the tests reach some of them.
#![allow(dead_code)]

use super::dmesh3::{DMesh3, MeshResult, INVALID_ID};
use super::dmesh3_edge_ops::{EdgeCollapseInfo, MergeEdgesInfo};
use super::dmesh3_queries::{find_tri_other_vtx, orient_tri_edge};

impl DMesh3 {
    /// C# `CollapseEdge(vKeep, vRemove, out collapse)`: removes `v_remove`, moving its edges
    /// and triangles onto `v_keep`; refuses a collapse that would leave the mesh non-manifold.
    pub(crate) fn collapse_edge(
        &mut self,
        v_keep: i32,
        v_remove: i32,
    ) -> (MeshResult, EdgeCollapseInfo) {
        let mut collapse = EdgeCollapseInfo::default();
        if !self.is_vertex(v_keep) || !self.is_vertex(v_remove) {
            return (MeshResult::FailedNotAnEdge, collapse);
        }

        let b = v_keep; // renaming for sanity. We remove a and keep b
        let a = v_remove;

        let eab = self.find_edge(a, b);
        if eab == INVALID_ID {
            return (MeshResult::FailedNotAnEdge, collapse);
        }

        let t0 = self.edges[eab as usize][2];
        if t0 == INVALID_ID {
            return (MeshResult::FailedBrokenTopology, collapse);
        }
        let t0tv = self.get_triangle(t0);
        let c = find_tri_other_vtx(a, b, &t0tv);

        // look up opposing triangle/vtx if we are not in boundary case
        let mut is_boundary_edge = false;
        let mut d = INVALID_ID;
        let t1 = self.edges[eab as usize][3];
        if t1 != INVALID_ID {
            let t1tv = self.get_triangle(t1);
            d = find_tri_other_vtx(a, b, &t1tv);
            if c == d {
                return (MeshResult::FailedFoundDuplicateTriangle, collapse);
            }
        } else {
            is_boundary_edge = true;
        }

        // We cannot collapse if edge lists of a and b share vertices other
        //  than c and d  (because then we will make a triangle [x b b].
        //  Unfortunately I cannot see a way to do this more efficiently than brute-force search
        //  [TODO] if we had tri iterator for a, couldn't we check each tri for b  (skipping t0 and t1) ?
        let a_edges = self.vertex_edges.values(a);
        let edges_a_count = a_edges.len();
        let mut eac = INVALID_ID;
        let mut ead = INVALID_ID;
        let mut ebc = INVALID_ID;
        let mut ebd = INVALID_ID;
        for &eid_a in &a_edges {
            let vax = self.edge_other_v(eid_a, a);
            if vax == c {
                eac = eid_a;
                continue;
            }
            if vax == d {
                ead = eid_a;
                continue;
            }
            if vax == b {
                continue;
            }
            if self
                .vertex_edges
                .iter(b)
                .any(|eid_b| self.edge_other_v(eid_b, b) == vax)
            {
                return (MeshResult::FailedInvalidNeighbourhood, collapse);
            }
        }

        // We cannot collapse if we have a tetrahedron. In this case a has 3 nbr edges,
        //  and edge cd exists. But that is not conclusive, we also have to check that
        //  cd is an internal edge, and that each of its tris contain a or b
        if edges_a_count == 3 && !is_boundary_edge {
            let edc = self.find_edge(d, c);
            if edc != INVALID_ID && self.edges[edc as usize][3] != INVALID_ID {
                let [_, _, edc_t0, edc_t1] = self.edges[edc as usize];
                if (self.tri_has_v(edc_t0, a) && self.tri_has_v(edc_t1, b))
                    || (self.tri_has_v(edc_t0, b) && self.tri_has_v(edc_t1, a))
                {
                    return (MeshResult::FailedCollapseTetrahedron, collapse);
                }
            }
        } else if is_boundary_edge && self.is_boundary_edge(eac) {
            // Cannot collapse edge if we are down to a single triangle
            ebc = self.find_edge_from_tri(b, c, t0);
            if self.is_boundary_edge(ebc) {
                return (MeshResult::FailedCollapseTriangle, collapse);
            }
        }

        // [RMS] this was added from C++ version...seems like maybe I never hit
        //   this case? Conceivably could be porting bug but looking at the
        //   C++ code I cannot see how we could possibly have caught this case...
        //
        // cannot collapse an edge where both vertices are boundary vertices
        // because that would create a bowtie
        //
        // NOTE: potentially scanning all edges here...couldn't we
        //  pick up eac/bc/ad/bd as we go? somehow?
        if !is_boundary_edge && self.is_boundary_vertex(a) && self.is_boundary_vertex(b) {
            return (MeshResult::FailedInvalidNeighbourhood, collapse);
        }

        // 1) remove edge ab from vtx b
        // 2) find edges ad and ac, and tris tad, tac across those edges  (will use later)
        // 3) for other edges, replace a with b, and add that edge to b
        // 4) replace a with b in all triangles connected to a
        let mut tad = INVALID_ID;
        let mut tac = INVALID_ID;
        for &eid in &a_edges {
            let o = self.edge_other_v(eid, a);
            if o == b {
                self.vertex_edges.remove(b, eid);
            } else if o == c {
                self.vertex_edges.remove(c, eid);
                tac = self.edge_other_t(eid, t0);
            } else if o == d {
                self.vertex_edges.remove(d, eid);
                tad = self.edge_other_t(eid, t1);
            } else {
                self.replace_edge_vertex(eid, a, b);
                self.vertex_edges.insert(b, eid);
            }

            // [TODO] perhaps we can already have unique tri list because of the manifold-nbrhood check we need to do...
            for j in 0..2 {
                let t_j = self.edges[eid as usize][2 + j];
                if t_j != INVALID_ID && t_j != t0 && t_j != t1 && self.tri_has_v(t_j, a) {
                    self.replace_tri_vertex(t_j, a, b);
                    self.vertices_refcount.increment(b, 1);
                    self.vertices_refcount.decrement(a, 1);
                }
            }
        }

        if !is_boundary_edge {
            // remove all edges from vtx a, then remove vtx a
            self.vertex_edges.clear(a);
            self.vertices_refcount.decrement(a, 3);

            // remove triangles T0 and T1, and update b/c/d refcounts
            self.triangles_refcount.decrement(t0, 1);
            self.triangles_refcount.decrement(t1, 1);
            self.vertices_refcount.decrement(c, 1);
            self.vertices_refcount.decrement(d, 1);
            self.vertices_refcount.decrement(b, 2);

            // remove edges ead, eab, eac
            self.edges_refcount.decrement(ead, 1);
            self.edges_refcount.decrement(eab, 1);
            self.edges_refcount.decrement(eac, 1);

            // replace t0 and t1 in edges ebd and ebc that we kept
            ebd = self.find_edge_from_tri(b, d, t1);
            if ebc == INVALID_ID {
                // we may have already looked this up
                ebc = self.find_edge_from_tri(b, c, t0);
            }

            self.replace_edge_triangle(ebd, t1, tad);
            self.replace_edge_triangle(ebc, t0, tac);

            // update tri-edge-nbrs in tad and tac
            if tad != INVALID_ID {
                self.replace_triangle_edge(tad, ead, ebd);
            }
            if tac != INVALID_ID {
                self.replace_triangle_edge(tac, eac, ebc);
            }
        } else {
            // boundary-edge path. this is basically same code as above, just not referencing t1/d

            // remove all edges from vtx a, then remove vtx a
            self.vertex_edges.clear(a);
            self.vertices_refcount.decrement(a, 2);

            // remove triangle T0 and update b/c refcounts
            self.triangles_refcount.decrement(t0, 1);
            self.vertices_refcount.decrement(c, 1);
            self.vertices_refcount.decrement(b, 1);

            // remove edges eab and eac
            self.edges_refcount.decrement(eab, 1);
            self.edges_refcount.decrement(eac, 1);

            // replace t0 in edge ebc that we kept
            ebc = self.find_edge_from_tri(b, c, t0);
            self.replace_edge_triangle(ebc, t0, tac);

            // update tri-edge-nbrs in tac
            if tac != INVALID_ID {
                self.replace_triangle_edge(tac, eac, ebc);
            }
        }

        collapse = EdgeCollapseInfo {
            v_kept: v_keep,
            v_removed: v_remove,
            is_boundary: is_boundary_edge,
            e_collapsed: eab,
            t_removed: [t0, t1],
            e_removed: [eac, ead],
            e_kept: [ebc, ebd],
        };
        (MeshResult::Ok, collapse)
    }

    /// C# `MergeEdges(eKeep, eDiscard, out merge_info)`: welds two boundary edges that run in
    /// opposite directions into one interior edge, fusing their vertices.
    pub(crate) fn merge_edges(
        &mut self,
        e_keep: i32,
        e_discard: i32,
    ) -> (MeshResult, MergeEdgesInfo) {
        let mut merge_info = MergeEdgesInfo::default();
        if !self.is_edge(e_keep) || !self.is_edge(e_discard) {
            return (MeshResult::FailedNotAnEdge, merge_info);
        }

        let edgeinfo_keep = self.get_edge(e_keep);
        let edgeinfo_discard = self.get_edge(e_discard);
        if edgeinfo_keep[3] != INVALID_ID || edgeinfo_discard[3] != INVALID_ID {
            return (MeshResult::FailedNotABoundaryEdge, merge_info);
        }

        let (mut a, mut b) = (edgeinfo_keep[0], edgeinfo_keep[1]);
        let tab = edgeinfo_keep[2];
        let eab = e_keep;
        let (mut c, mut d) = (edgeinfo_discard[0], edgeinfo_discard[1]);
        let tcd = edgeinfo_discard[2];
        let ecd = e_discard;

        // Need to correctly orient a,b and c,d and then check that
        // we will not join triangles with incompatible winding order
        // I can't see how to do this purely topologically.
        // So relying on closest-pairs testing.
        orient_tri_edge(&mut a, &mut b, &self.get_triangle(tab));
        orient_tri_edge(&mut c, &mut d, &self.get_triangle(tcd));
        // joinable bdry edges have opposing orientations, so flip to get ac and b/d correspondences
        std::mem::swap(&mut c, &mut d);
        let (va, vb, vc, vd) = (
            self.vertex(a),
            self.vertex(b),
            self.vertex(c),
            self.vertex(d),
        );
        if (distance_squared(va, vc) + distance_squared(vb, vd))
            > (distance_squared(va, vd) + distance_squared(vb, vc))
        {
            return (MeshResult::FailedSameOrientation, merge_info);
        }

        // alternative that detects normal flip of triangle tcd. This is a more
        // robust geometric test, but fails if tri is degenerate...also more expensive

        merge_info.e_kept = eab;
        merge_info.e_removed = ecd;

        // if a/c or b/d are connected by an existing edge, we can't merge
        if a != c && self.find_edge(a, c) != INVALID_ID {
            return (MeshResult::FailedInvalidNeighbourhood, merge_info);
        }
        if b != d && self.find_edge(b, d) != INVALID_ID {
            return (MeshResult::FailedInvalidNeighbourhood, merge_info);
        }

        // if vertices at either end already share a common neighbour vertex, and the
        // edges to that vertex are both boundary edges, we can't merge
        if a != c && self.shares_interior_neighbour(a, c, if b == d { b } else { -1 }) {
            return (MeshResult::FailedInvalidNeighbourhood, merge_info);
        }
        if b != d && self.shares_interior_neighbour(b, d, if a == c { a } else { -1 }) {
            return (MeshResult::FailedInvalidNeighbourhood, merge_info);
        }

        // [TODO] this acts on each interior tri twice. could avoid using vtx-tri iterator?
        if a != c {
            self.move_vertex_onto(c, a, e_discard);
            merge_info.v_removed[0] = c;
        } else {
            self.vertex_edges.remove(a, ecd);
            merge_info.v_removed[0] = INVALID_ID;
        }
        merge_info.v_kept[0] = a;

        if d != b {
            self.move_vertex_onto(d, b, e_discard);
            merge_info.v_removed[1] = d;
        } else {
            self.vertex_edges.remove(b, ecd);
            merge_info.v_removed[1] = INVALID_ID;
        }
        merge_info.v_kept[1] = b;

        // replace edge cd with edge ab in triangle tcd
        self.replace_triangle_edge(tcd, ecd, eab);
        self.edges_refcount.decrement(ecd, 1);

        // update edge-tri adjacency
        self.set_edge_triangles(eab, tab, tcd);

        // Once we merge ab to cd, there may be additional edges (now) connected
        // to either a or b that are connected to the same vertex on their 'other' side.
        // So we now have two boundary edges connecting the same two vertices - disaster!
        // We need to find and merge these edges.
        // Q: I don't think it is possible to have multiple such edge-pairs at a or b
        //    But I am not certain...is a bit tricky to handle because we modify edges_v...
        merge_info.e_removed_extra = [INVALID_ID, INVALID_ID];
        merge_info.e_kept_extra = merge_info.e_removed_extra;
        for vi in 0..2 {
            let (v1, v2) = if vi == 1 { (b, d) } else { (a, c) }; // vertices of merged edge
            if v1 == v2 {
                continue;
            }
            let edges_v = self.vertex_edges.values(v1);
            let n_edges = edges_v.len();
            let mut found = false;
            let mut i = 0;
            while i < n_edges && !found {
                let edge_1 = edges_v[i];
                if self.is_boundary_edge(edge_1) {
                    let vert_1 = self.edge_other_v(edge_1, v1);
                    for &edge_2 in &edges_v[i + 1..n_edges] {
                        let vert_2 = self.edge_other_v(edge_2, v1);
                        if vert_1 == vert_2 && self.is_boundary_edge(edge_2) {
                            // if ! boundary here, we are in deep trouble...
                            // replace edge_2 w/ edge_1 in tri, update edge and vtx-edge-nbr lists
                            let tri_1 = self.edges[edge_1 as usize][2];
                            let tri_2 = self.edges[edge_2 as usize][2];
                            self.replace_triangle_edge(tri_2, edge_2, edge_1);
                            self.set_edge_triangles(edge_1, tri_1, tri_2);
                            self.vertex_edges.remove(v1, edge_2);
                            self.vertex_edges.remove(vert_1, edge_2);
                            self.edges_refcount.decrement(edge_2, 1);
                            merge_info.e_removed_extra[vi] = edge_2;
                            merge_info.e_kept_extra[vi] = edge_1;
                            found = true; // exit outer i loop
                            break; // exit inner j loop
                        }
                    }
                }
                i += 1;
            }
        }
        (MeshResult::Ok, merge_info)
    }

    /// The C# `MergeEdges` neighbourhood check: does `from` share a neighbour (other than
    /// `other_v`) with `keep` through an edge that is not a boundary on both sides?
    fn shares_interior_neighbour(&self, keep: i32, from: i32, other_v: i32) -> bool {
        for nbr in self.vtx_vertices(from) {
            if nbr == other_v {
                continue;
            }
            let ea = self.find_edge(keep, nbr);
            if ea != INVALID_ID {
                let ec = self.find_edge(from, nbr);
                if !self.is_boundary_edge(ea) || !self.is_boundary_edge(ec) {
                    return true;
                }
            }
        }
        false
    }

    /// The C# `MergeEdges` vertex move: every edge of `from` but `e_discard` (and the
    /// triangles on it) re-pointed at `keep`, then `from` freed.
    fn move_vertex_onto(&mut self, from: i32, keep: i32, e_discard: i32) {
        for eid in self.vertex_edges.values(from) {
            if eid == e_discard {
                continue;
            }
            self.replace_edge_vertex(eid, from, keep);
            let mut rc = 0;
            if self.replace_tri_vertex(self.edges[eid as usize][2], from, keep) >= 0 {
                rc += 1;
            }
            if self.edges[eid as usize][3] != INVALID_ID
                && self.replace_tri_vertex(self.edges[eid as usize][3], from, keep) >= 0
            {
                rc += 1;
            }
            self.vertex_edges.insert(keep, eid);
            if rc > 0 {
                self.vertices_refcount.increment(keep, rc);
                self.vertices_refcount.decrement(from, rc);
            }
        }
        self.vertex_edges.clear(from);
        self.vertices_refcount.decrement(from, 1);
    }
}

fn distance_squared(a: [f64; 3], b: [f64; 3]) -> f64 {
    let (dx, dy, dz) = (a[0] - b[0], a[1] - b[1], a[2] - b[2]);
    dx * dx + dy * dy + dz * dz
}
