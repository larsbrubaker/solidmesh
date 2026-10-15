//! The topology queries of `DMesh3` and the index helpers they and the edits share: finding edges
//! and triangles, the vertex one-rings in the order the vertex's edge list holds them, and the
//! boundary, bowtie and neighbour tests the repair asks. Split from `dmesh3`, which holds the
//! storage.

use super::dmesh3::{DMesh3, INVALID_ID};

pub(crate) fn same_pair_unordered(a0: i32, a1: i32, b0: i32, b1: i32) -> bool {
    if a0 == b0 {
        a1 == b1
    } else {
        a0 == b1 && a1 == b0
    }
}

pub(crate) fn find_tri_index(a: i32, tri: &[i32; 3]) -> i32 {
    tri.iter()
        .position(|&v| v == a)
        .map_or(INVALID_ID, |i| i as i32)
}

pub(crate) fn find_edge_index_in_tri(a: i32, b: i32, tri: &[i32; 3]) -> i32 {
    for j in 0..3 {
        if same_pair_unordered(a, b, tri[j], tri[(j + 1) % 3]) {
            return j as i32;
        }
    }
    INVALID_ID
}

/// Index of `a` where `[a, b]` runs in order, or
/// [`INVALID_ID`].
pub(crate) fn find_tri_ordered_edge(a: i32, b: i32, tri: &[i32; 3]) -> i32 {
    for j in 0..3 {
        if tri[j] == a && tri[(j + 1) % 3] == b {
            return j as i32;
        }
    }
    INVALID_ID
}

pub(crate) fn find_tri_other_vtx(a: i32, b: i32, tri: &[i32; 3]) -> i32 {
    for j in 0..3 {
        if same_pair_unordered(a, b, tri[j], tri[(j + 1) % 3]) {
            return tri[(j + 2) % 3];
        }
    }
    INVALID_ID
}

/// Sets `[a, b]` to the order the triangle holds them in and returns the third vertex.
pub(crate) fn orient_tri_edge_and_find_other_vtx(a: &mut i32, b: &mut i32, tri: &[i32; 3]) -> i32 {
    for j in 0..3 {
        if same_pair_unordered(*a, *b, tri[j], tri[(j + 1) % 3]) {
            *a = tri[j];
            *b = tri[(j + 1) % 3];
            return tri[(j + 2) % 3];
        }
    }
    INVALID_ID
}

pub(crate) fn orient_tri_edge(a: &mut i32, b: &mut i32, tri: &[i32; 3]) {
    orient_tri_edge_and_find_other_vtx(a, b, tri);
}

impl DMesh3 {
    /// Edges are stored `(min, max)`, so only the smaller vertex's list is searched.
    pub(crate) fn find_edge(&self, va: i32, vb: i32) -> i32 {
        let vo = va.max(vb);
        let vi = va.min(vb);
        self.vertex_edges
            .iter(vi)
            .find(|&eid| self.edges[eid as usize][1] == vo)
            .unwrap_or(INVALID_ID)
    }

    pub(crate) fn find_edge_from_tri(&self, va: i32, vb: i32, tid: i32) -> i32 {
        let t = self.triangles[tid as usize];
        let te = self.triangle_edges[tid as usize];
        if same_pair_unordered(va, vb, t[0], t[1]) {
            return te[0];
        }
        if same_pair_unordered(va, vb, t[1], t[2]) {
            return te[1];
        }
        if same_pair_unordered(va, vb, t[2], t[0]) {
            return te[2];
        }
        INVALID_ID
    }

    pub(crate) fn find_tri_neighbour_edge(&self, tid: i32, va: i32, vb: i32) -> i32 {
        self.find_edge_from_tri(va, vb, tid)
    }

    #[allow(dead_code)] // only the tests call this
    /// The triangle made of any permutation of `[a, b, c]`.
    pub(crate) fn find_triangle(&self, a: i32, b: i32, c: i32) -> i32 {
        let eid = self.find_edge(a, b);
        if eid == INVALID_ID {
            return INVALID_ID;
        }
        let e = self.edges[eid as usize];
        if self.triangles[e[2] as usize].contains(&c) {
            return e[2];
        }
        if e[3] != INVALID_ID && self.triangles[e[3] as usize].contains(&c) {
            return e[3];
        }
        INVALID_ID
    }

    /// `[c, d]` for an edge on `[a,b,c]` and `[a,b,d]`, `d` invalid on a boundary edge.
    pub(crate) fn get_edge_opposing_v(&self, eid: i32) -> [i32; 2] {
        let [a, b, t0, t1] = self.edges[eid as usize];
        let c = find_tri_other_vtx(a, b, &self.triangles[t0 as usize]);
        if t1 != INVALID_ID {
            [c, find_tri_other_vtx(a, b, &self.triangles[t1 as usize])]
        } else {
            [c, INVALID_ID]
        }
    }

    pub(crate) fn is_boundary_edge(&self, eid: i32) -> bool {
        self.edges[eid as usize][3] == INVALID_ID
    }

    pub(crate) fn is_boundary_vertex(&self, vid: i32) -> bool {
        self.vertex_edges
            .iter(vid)
            .any(|e| self.edges[e as usize][3] == INVALID_ID)
    }

    #[allow(dead_code)] // only the tests call this
    pub(crate) fn is_boundary_triangle(&self, tid: i32) -> bool {
        self.triangle_edges[tid as usize]
            .iter()
            .any(|&e| self.is_boundary_edge(e))
    }

    pub(crate) fn edge_other_v(&self, eid: i32, vid: i32) -> i32 {
        let e = self.edges[eid as usize];
        if e[0] == vid {
            e[1]
        } else if e[1] == vid {
            e[0]
        } else {
            INVALID_ID
        }
    }

    pub(crate) fn edge_other_t(&self, eid: i32, tid: i32) -> i32 {
        let e = self.edges[eid as usize];
        if e[2] == tid {
            e[3]
        } else if e[3] == tid {
            e[2]
        } else {
            INVALID_ID
        }
    }

    pub(crate) fn tri_has_v(&self, tid: i32, vid: i32) -> bool {
        self.triangles[tid as usize].contains(&vid)
    }

    pub(crate) fn tri_has_sequential_v(&self, tid: i32, va: i32, vb: i32) -> bool {
        find_tri_ordered_edge(va, vb, &self.triangles[tid as usize]) != INVALID_ID
    }

    /// The edges at a vertex (empty for a dead vertex).
    pub(crate) fn vtx_edges(&self, vid: i32) -> Vec<i32> {
        if self.is_vertex(vid) {
            self.vertex_edges.values(vid)
        } else {
            Vec::new()
        }
    }

    pub(crate) fn get_vtx_edge_count(&self, vid: i32) -> i32 {
        if self.is_vertex(vid) {
            self.vertex_edges.count(vid) as i32
        } else {
            -1
        }
    }

    /// The vertices one edge away from a vertex.
    pub(crate) fn vtx_vertices(&self, vid: i32) -> Vec<i32> {
        self.vtx_edges(vid)
            .into_iter()
            .map(|eid| self.edge_other_v(eid, vid))
            .collect()
    }

    /// Each triangle once, found through the edge it leaves the vertex along.
    pub(crate) fn vtx_triangles(&self, vid: i32) -> Vec<i32> {
        let mut out = Vec::new();
        if !self.is_vertex(vid) {
            return out;
        }
        for eid in self.vertex_edges.iter(vid) {
            let other = self.edge_other_v(eid, vid);
            let [_, _, et0, et1] = self.edges[eid as usize];
            if self.tri_has_sequential_v(et0, vid, other) {
                out.push(et0);
            }
            if et1 != INVALID_ID && self.tri_has_sequential_v(et1, vid, other) {
                out.push(et1);
            }
        }
        out
    }

    pub(crate) fn get_tri_neighbour_tris(&self, tid: i32) -> [i32; 3] {
        if !self.is_triangle(tid) {
            return [INVALID_ID; 3];
        }
        let te = self.triangle_edges[tid as usize];
        std::array::from_fn(|j| {
            let e = self.edges[te[j] as usize];
            if e[2] == tid {
                e[3]
            } else {
                e[2]
            }
        })
    }

    /// Walks the one-ring from a boundary edge (or any edge) and reports whether it missed edges. A
    /// dead vertex is not one.
    pub(crate) fn is_bowtie_vertex(&self, vid: i32) -> bool {
        if !self.is_vertex(vid) {
            return false;
        }
        let n_edges = self.vertex_edges.count(vid);
        if n_edges == 0 {
            return false;
        }

        // find a boundary edge to start at
        let boundary_start = self
            .vertex_edges
            .iter(vid)
            .find(|&eid| self.edges[eid as usize][3] == INVALID_ID);
        let start_at_boundary = boundary_start.is_some();
        // if no boundary edge, start at arbitrary edge
        let start_eid = boundary_start.unwrap_or_else(|| self.vertex_edges.first(vid));

        // initial triangle
        let mut prev_tid = self.edges[start_eid as usize][2];
        let mut prev_eid = start_eid;

        // walk forward to next edge. if we hit start edge or boundary edge,
        // we are done the walk. count number of edges as we go.
        let mut count = 1;
        loop {
            let tv = self.triangles[prev_tid as usize];
            let te = self.triangle_edges[prev_tid as usize];
            let vert_idx = find_tri_index(vid, &tv) as usize;
            let e1 = te[vert_idx];
            let e2 = te[(vert_idx + 2) % 3];
            let next_eid = if e1 == prev_eid { e2 } else { e1 };
            if next_eid == start_eid {
                break;
            }
            let [_, _, nt0, nt1] = self.edges[next_eid as usize];
            let next_tid = if nt0 == prev_tid { nt1 } else { nt0 };
            if next_tid == INVALID_ID {
                break;
            }
            prev_eid = next_eid;
            prev_tid = next_tid;
            count += 1;
        }

        // if we did not see all edges at vertex, we have a bowtie
        let target_count = if start_at_boundary {
            n_edges - 1
        } else {
            n_edges
        };
        target_count != count
    }
}
