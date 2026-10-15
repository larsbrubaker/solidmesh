//! The topology edits of `DMesh3`: reversing triangles, removing triangles and vertices, splitting,
//! flipping and collapsing edges, merging two boundary edges, and poking a triangle. Each follows a
//! fixed order of list and refcount updates, because that order decides which ids the next
//! allocation reuses and the order every later one-ring walk sees.

use super::dmesh3::{DMesh3, G3Exception, MeshResult, INVALID_ID};
use super::dmesh3_queries::{
    find_edge_index_in_tri, find_tri_index, find_tri_other_vtx, orient_tri_edge_and_find_other_vtx,
};

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub(crate) struct EdgeSplitInfo {
    pub(crate) is_boundary: bool,
    pub(crate) v_new: i32,
    pub(crate) e_new_bn: i32,
    pub(crate) e_new_cn: i32,
    pub(crate) e_new_dn: i32,
    pub(crate) e_new_t2: i32,
    pub(crate) e_new_t3: i32,
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub(crate) struct EdgeFlipInfo {
    pub(crate) eid: i32,
    pub(crate) v0: i32,
    pub(crate) v1: i32,
    pub(crate) ov0: i32,
    pub(crate) ov1: i32,
    pub(crate) t0: i32,
    pub(crate) t1: i32,
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub(crate) struct EdgeCollapseInfo {
    pub(crate) v_kept: i32,
    pub(crate) v_removed: i32,
    pub(crate) is_boundary: bool,
    pub(crate) e_collapsed: i32,
    pub(crate) t_removed: [i32; 2],
    pub(crate) e_removed: [i32; 2],
    pub(crate) e_kept: [i32; 2],
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub(crate) struct MergeEdgesInfo {
    pub(crate) e_kept: i32,
    pub(crate) e_removed: i32,
    pub(crate) v_kept: [i32; 2],
    pub(crate) v_removed: [i32; 2],
    pub(crate) e_removed_extra: [i32; 2],
    pub(crate) e_kept_extra: [i32; 2],
}

#[allow(dead_code)] // only the tests call this
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub(crate) struct PokeTriangleInfo {
    pub(crate) new_vid: i32,
    pub(crate) new_t1: i32,
    pub(crate) new_t2: i32,
    pub(crate) new_edges: [i32; 3],
}

fn lerp(a: [f64; 3], b: [f64; 3], t: f64) -> [f64; 3] {
    let s = 1.0 - t;
    [
        s * a[0] + t * b[0],
        s * a[1] + t * b[1],
        s * a[2] + t * b[2],
    ]
}

impl DMesh3 {
    pub(crate) fn reverse_tri_orientation(&mut self, tid: i32) -> MeshResult {
        if !self.is_triangle(tid) {
            return MeshResult::FailedNotATriangle;
        }
        self.internal_reverse_tri_orientation(tid);
        MeshResult::Ok
    }

    fn internal_reverse_tri_orientation(&mut self, tid: i32) {
        let t = self.triangles[tid as usize];
        self.triangles[tid as usize] = [t[1], t[0], t[2]];
        let te = self.triangle_edges[tid as usize];
        self.triangle_edges[tid as usize] = [te[0], te[2], te[1]];
    }

    #[allow(dead_code)] // only the tests call this
    /// Reverses every triangle (there are no normals to flip here).
    pub(crate) fn reverse_orientation(&mut self) {
        let tids: Vec<i32> = self.triangle_indices().collect();
        for tid in tids {
            self.internal_reverse_tri_orientation(tid);
        }
    }

    /// The vertex and (with `remove_all_triangles`) every triangle on it.
    pub(crate) fn remove_vertex(
        &mut self,
        vid: i32,
        remove_all_triangles: bool,
        preserve_manifold: bool,
    ) -> Result<MeshResult, G3Exception> {
        if !self.vertices_refcount.is_valid(vid) {
            return Ok(MeshResult::FailedNotAVertex);
        }
        if remove_all_triangles {
            // if any one-ring vtx is a boundary vtx and one of its outer-ring edges is an
            // interior edge then we will create a bowtie if we remove that triangle
            if preserve_manifold {
                for tid in self.vtx_triangles(vid) {
                    let tri = self.get_triangle(tid);
                    let j = find_tri_index(vid, &tri) as usize;
                    let (oa, ob) = (tri[(j + 1) % 3], tri[(j + 2) % 3]);
                    let eid = self.find_edge(oa, ob);
                    if self.is_boundary_edge(eid) {
                        continue;
                    }
                    if self.is_boundary_vertex(oa) || self.is_boundary_vertex(ob) {
                        return Ok(MeshResult::FailedWouldCreateBowtie);
                    }
                }
            }
            for tid in self.vtx_triangles(vid) {
                let result = self.remove_triangle(tid, false, preserve_manifold);
                if result != MeshResult::Ok {
                    return Ok(result);
                }
            }
        }
        if self.vertices_refcount.ref_count(vid) != 1 {
            return Err(G3Exception(
                "DMesh3.RemoveVertex: vertex is still referenced",
            ));
        }
        self.vertices_refcount.decrement(vid, 1);
        self.vertex_edges.clear(vid);
        Ok(MeshResult::Ok)
    }

    /// Also removes edges left with no triangle, and (with `remove_isolated_vertices`) vertices
    /// left with none.
    pub(crate) fn remove_triangle(
        &mut self,
        tid: i32,
        remove_isolated_vertices: bool,
        preserve_manifold: bool,
    ) -> MeshResult {
        if !self.triangles_refcount.is_valid(tid) {
            return MeshResult::FailedNotATriangle;
        }
        let tv = self.get_triangle(tid);
        let te = self.get_tri_edges(tid);

        // if any tri vtx is a boundary vtx connected to two interior edges, then
        // we cannot remove this triangle because it would create a bowtie vertex!
        // (that vtx already has 2 boundary edges, and we would add two more)
        if preserve_manifold {
            for j in 0..3 {
                if self.is_boundary_vertex(tv[j])
                    && !self.is_boundary_edge(te[j])
                    && !self.is_boundary_edge(te[(j + 2) % 3])
                {
                    return MeshResult::FailedWouldCreateBowtie;
                }
            }
        }

        // Remove triangle from its edges. if edge has no triangles left,
        // then it is removed.
        for &eid in &te {
            self.replace_edge_triangle(eid, tid, INVALID_ID);
            if self.edges[eid as usize][2] == INVALID_ID {
                let [a, b, _, _] = self.edges[eid as usize];
                self.vertex_edges.remove(a, eid);
                self.vertex_edges.remove(b, eid);
                self.edges_refcount.decrement(eid, 1);
            }
        }

        // free this triangle
        self.triangles_refcount.decrement(tid, 1);

        // Decrement vertex refcounts. If any hit 1 and we got remove-isolated flag,
        // we need to remove that vertex
        for &vid in &tv {
            self.vertices_refcount.decrement(vid, 1);
            if remove_isolated_vertices && self.vertices_refcount.ref_count(vid) == 1 {
                self.vertices_refcount.decrement(vid, 1);
                self.vertex_edges.clear(vid);
            }
        }
        MeshResult::Ok
    }

    pub(crate) fn split_edge(&mut self, eab: i32, split_t: f64) -> (MeshResult, EdgeSplitInfo) {
        let mut split = EdgeSplitInfo::default();
        if !self.is_edge(eab) {
            return (MeshResult::FailedNotAnEdge, split);
        }

        // look up primary edge & triangle
        let [mut a, mut b, t0, t1] = self.edges[eab as usize];
        if t0 == INVALID_ID {
            return (MeshResult::FailedBrokenTopology, split);
        }
        let t0tv = self.get_triangle(t0);
        let c = orient_tri_edge_and_find_other_vtx(&mut a, &mut b, &t0tv);
        if self.vertices_refcount.raw_ref_count(c) > 32764 {
            return (MeshResult::FailedHitValenceLimit, split);
        }
        let mut split_t = split_t;
        if a != self.edges[eab as usize][0] {
            split_t = 1.0 - split_t; // if we flipped a/b order we need to reverse t
        }

        // quite a bit of code is duplicated between boundary and non-boundary case, but it
        // seems to make it inconvenient to share...
        if self.is_boundary_edge(eab) {
            let v_new = lerp(self.vertex(a), self.vertex(b), split_t);
            let f = self.append_vertex(v_new);

            // look up edge bc, which needs to be modified
            let t0te = self.get_tri_edges(t0);
            let ebc = t0te[find_edge_index_in_tri(b, c, &t0tv) as usize];

            // rewrite existing triangle
            self.replace_tri_vertex(t0, b, f);

            // add new triangle
            let t2 = self.add_triangle_only([f, b, c], [INVALID_ID; 3]);
            self.copy_triangle_group(t0, t2);

            // rewrite edge bc, create edge af
            self.replace_edge_triangle(ebc, t0, t2);
            let eaf = eab;
            self.replace_edge_vertex(eaf, b, f);
            self.vertex_edges.remove(b, eab);
            self.vertex_edges.insert(f, eaf);

            // create edges fb and fc
            let efb = self.add_edge(f, b, t2, INVALID_ID);
            let efc = self.add_edge(f, c, t0, t2);

            // update triangle edge-nbrs
            self.replace_triangle_edge(t0, ebc, efc);
            self.triangle_edges[t2 as usize] = [efb, ebc, efc];

            // update vertex refcounts
            self.vertices_refcount.increment(c, 1);
            self.vertices_refcount.increment(f, 2);

            split = EdgeSplitInfo {
                is_boundary: true,
                v_new: f,
                e_new_bn: efb,
                e_new_cn: efc,
                e_new_dn: INVALID_ID,
                e_new_t2: t2,
                e_new_t3: INVALID_ID,
            };
            (MeshResult::Ok, split)
        } else {
            // interior triangle branch

            // look up other triangle
            let t1tv = self.get_triangle(t1);
            let d = find_tri_other_vtx(a, b, &t1tv);
            if self.vertices_refcount.raw_ref_count(d) > 32764 {
                return (MeshResult::FailedHitValenceLimit, split);
            }

            let v_new = lerp(self.vertex(a), self.vertex(b), split_t);
            let f = self.append_vertex(v_new);

            // look up edges that we are going to need to update
            // [TODO OPT] could use ordering to reduce # of compares here
            let t0te = self.get_tri_edges(t0);
            let ebc = t0te[find_edge_index_in_tri(b, c, &t0tv) as usize];
            let t1te = self.get_tri_edges(t1);
            let edb = t1te[find_edge_index_in_tri(d, b, &t1tv) as usize];

            // rewrite existing triangles
            self.replace_tri_vertex(t0, b, f);
            self.replace_tri_vertex(t1, b, f);

            // add two new triangles to close holes
            let t2 = self.add_triangle_only([f, b, c], [INVALID_ID; 3]);
            let t3 = self.add_triangle_only([f, d, b], [INVALID_ID; 3]);
            self.copy_triangle_group(t0, t2);
            self.copy_triangle_group(t1, t3);

            // update the edges we found above, to point to new triangles
            self.replace_edge_triangle(ebc, t0, t2);
            self.replace_edge_triangle(edb, t1, t3);

            // edge eab became eaf
            let eaf = eab;
            self.replace_edge_vertex(eaf, b, f);

            // update a/b/f vertex-edges
            self.vertex_edges.remove(b, eab);
            self.vertex_edges.insert(f, eaf);

            // create new edges fb and fc
            let efb = self.add_edge(f, b, t2, t3);
            let efc = self.add_edge(f, c, t0, t2);
            let edf = self.add_edge(d, f, t1, t3);

            // update triangle edge-nbrs
            self.replace_triangle_edge(t0, ebc, efc);
            self.replace_triangle_edge(t1, edb, edf);
            self.triangle_edges[t2 as usize] = [efb, ebc, efc];
            self.triangle_edges[t3 as usize] = [edf, edb, efb];

            // update vertex refcounts
            self.vertices_refcount.increment(c, 1);
            self.vertices_refcount.increment(d, 1);
            self.vertices_refcount.increment(f, 4);

            split = EdgeSplitInfo {
                is_boundary: false,
                v_new: f,
                e_new_bn: efb,
                e_new_cn: efc,
                e_new_dn: edf,
                e_new_t2: t2,
                e_new_t3: t3,
            };
            (MeshResult::Ok, split)
        }
    }

    fn copy_triangle_group(&mut self, from: i32, to: i32) {
        if let Some(groups) = &mut self.triangle_groups {
            let g = groups[from as usize];
            super::dmesh3::insert_at(groups, to, g);
        }
    }

    /// Replaces `[a,b]` between `[a,b,c]` and `[b,a,d]` with `[c,d]`.
    pub(crate) fn flip_edge(
        &mut self,
        eab: i32,
    ) -> Result<(MeshResult, EdgeFlipInfo), G3Exception> {
        let mut flip = EdgeFlipInfo::default();
        if !self.is_edge(eab) {
            return Ok((MeshResult::FailedNotAnEdge, flip));
        }
        if self.is_boundary_edge(eab) {
            return Ok((MeshResult::FailedIsBoundaryEdge, flip));
        }

        // find oriented edge [a,b], tris t0,t1, and other verts c in t0, d in t1
        let [mut a, mut b, t0, t1] = self.edges[eab as usize];
        let t0tv = self.get_triangle(t0);
        let t1tv = self.get_triangle(t1);
        let c = orient_tri_edge_and_find_other_vtx(&mut a, &mut b, &t0tv);
        let d = find_tri_other_vtx(a, b, &t1tv);
        if c == INVALID_ID || d == INVALID_ID {
            return Ok((MeshResult::FailedBrokenTopology, flip));
        }

        let flipped = self.find_edge(c, d);
        if flipped != INVALID_ID {
            return Ok((MeshResult::FailedFlippedEdgeExists, flip));
        }

        // find edges bc, ca, ad, db
        let ebc = self.find_tri_neighbour_edge(t0, b, c);
        let eca = self.find_tri_neighbour_edge(t0, c, a);
        let ead = self.find_tri_neighbour_edge(t1, a, d);
        let edb = self.find_tri_neighbour_edge(t1, d, b);

        // update triangles
        self.triangles[t0 as usize] = [c, d, b];
        self.triangles[t1 as usize] = [d, c, a];

        // update edge AB, which becomes flipped edge CD
        self.set_edge_vertices(eab, c, d);
        self.set_edge_triangles(eab, t0, t1);
        let ecd = eab;

        // update the two other edges whose triangle nbrs have changed
        if self.replace_edge_triangle(eca, t0, t1) == -1 {
            return Err(G3Exception(
                "DMesh3.FlipEdge: first replace_edge_triangle failed",
            ));
        }
        if self.replace_edge_triangle(edb, t1, t0) == -1 {
            return Err(G3Exception(
                "DMesh3.FlipEdge: second replace_edge_triangle failed",
            ));
        }

        // update triangle nbr lists (these are edges)
        self.triangle_edges[t0 as usize] = [ecd, edb, ebc];
        self.triangle_edges[t1 as usize] = [ecd, eca, ead];

        // remove old eab from verts a and b, and decrement ref counts
        if !self.vertex_edges.remove(a, eab) {
            return Err(G3Exception(
                "DMesh3.FlipEdge: first edge list remove failed",
            ));
        }
        if !self.vertex_edges.remove(b, eab) {
            return Err(G3Exception(
                "DMesh3.FlipEdge: second edge list remove failed",
            ));
        }
        self.vertices_refcount.decrement(a, 1);
        self.vertices_refcount.decrement(b, 1);
        if !self.is_vertex(a) || !self.is_vertex(b) {
            return Err(G3Exception(
                "DMesh3.FlipEdge: either a or b is not a vertex?",
            ));
        }

        // add new edge ecd to verts c and d, and increment ref counts
        self.vertex_edges.insert(c, ecd);
        self.vertex_edges.insert(d, ecd);
        self.vertices_refcount.increment(c, 1);
        self.vertices_refcount.increment(d, 1);

        // success! collect up results
        flip = EdgeFlipInfo {
            eid: eab,
            v0: a,
            v1: b,
            ov0: c,
            ov1: d,
            t0,
            t1,
        };
        Ok((MeshResult::Ok, flip))
    }

    #[allow(dead_code)] // only the tests call this
    /// A new vertex at the barycentric point, joined to the three corners.
    pub(crate) fn poke_triangle(
        &mut self,
        tid: i32,
        bary: [f64; 3],
    ) -> (MeshResult, PokeTriangleInfo) {
        let mut result = PokeTriangleInfo::default();
        if !self.is_triangle(tid) {
            return (MeshResult::FailedNotATriangle, result);
        }
        let tv = self.get_triangle(tid);
        let te = self.get_tri_edges(tid);

        // create vertex with interpolated vertex attribs
        let [va, vb, vc] = [self.vertex(tv[0]), self.vertex(tv[1]), self.vertex(tv[2])];
        let p = std::array::from_fn(|k| bary[0] * va[k] + bary[1] * vb[k] + bary[2] * vc[k]);
        let center = self.append_vertex(p);

        // add in edges to center vtx, do not connect to triangles yet
        let ea_c = self.add_edge(tv[0], center, -1, -1);
        let eb_c = self.add_edge(tv[1], center, -1, -1);
        let ec_c = self.add_edge(tv[2], center, -1, -1);
        self.vertices_refcount.increment(tv[0], 1);
        self.vertices_refcount.increment(tv[1], 1);
        self.vertices_refcount.increment(tv[2], 1);
        self.vertices_refcount.increment(center, 3);

        // old triangle becomes tri along first edge
        self.triangles[tid as usize] = [tv[0], tv[1], center];
        self.triangle_edges[tid as usize] = [te[0], eb_c, ea_c];

        // add two new triangles
        let t1 = self.add_triangle_only([tv[1], tv[2], center], [te[1], ec_c, eb_c]);
        let t2 = self.add_triangle_only([tv[2], tv[0], center], [te[2], ea_c, ec_c]);

        // second and third edges of original tri have neighbours
        self.replace_edge_triangle(te[1], tid, t1);
        self.replace_edge_triangle(te[2], tid, t2);

        // set the triangles for the edges we created above
        self.set_edge_triangles(ea_c, tid, t2);
        self.set_edge_triangles(eb_c, tid, t1);
        self.set_edge_triangles(ec_c, t1, t2);

        // transfer groups
        self.copy_triangle_group(tid, t1);
        self.copy_triangle_group(tid, t2);

        result = PokeTriangleInfo {
            new_vid: center,
            new_t1: t1,
            new_t2: t2,
            new_edges: [ea_c, eb_c, ec_c],
        };
        (MeshResult::Ok, result)
    }

    pub(super) fn replace_tri_vertex(&mut self, tid: i32, v_old: i32, v_new: i32) -> i32 {
        let t = &mut self.triangles[tid as usize];
        for (j, v) in t.iter_mut().enumerate() {
            if *v == v_old {
                *v = v_new;
                return j as i32;
            }
        }
        -1
    }

    pub(super) fn set_edge_vertices(&mut self, eid: i32, a: i32, b: i32) {
        let e = &mut self.edges[eid as usize];
        e[0] = a.min(b);
        e[1] = a.max(b);
    }

    pub(super) fn set_edge_triangles(&mut self, eid: i32, t0: i32, t1: i32) {
        let e = &mut self.edges[eid as usize];
        e[2] = t0;
        e[3] = t1;
    }

    pub(super) fn replace_edge_vertex(&mut self, eid: i32, v_old: i32, v_new: i32) -> i32 {
        let e = &mut self.edges[eid as usize];
        let (a, b) = (e[0], e[1]);
        if a == v_old {
            e[0] = b.min(v_new);
            e[1] = b.max(v_new);
            0
        } else if b == v_old {
            e[0] = a.min(v_new);
            e[1] = a.max(v_new);
            1
        } else {
            -1
        }
    }

    /// Removing `t0` moves `t1` into its slot.
    pub(super) fn replace_edge_triangle(&mut self, eid: i32, t_old: i32, t_new: i32) -> i32 {
        let e = &mut self.edges[eid as usize];
        let (a, b) = (e[2], e[3]);
        if a == t_old {
            if t_new == INVALID_ID {
                e[2] = b;
                e[3] = INVALID_ID;
            } else {
                e[2] = t_new;
            }
            0
        } else if b == t_old {
            e[3] = t_new;
            1
        } else {
            -1
        }
    }

    pub(super) fn replace_triangle_edge(&mut self, tid: i32, e_old: i32, e_new: i32) -> i32 {
        let te = &mut self.triangle_edges[tid as usize];
        for (j, e) in te.iter_mut().enumerate() {
            if *e == e_old {
                *e = e_new;
                return j as i32;
            }
        }
        -1
    }
}
