//! geometry3Sharp's `DMesh3`: the dynamic, edge-based triangle mesh the signed distance grids and
//! the mesh repair (`mesh_auto_repair`) work on. This file holds the storage - vertices, triangles,
//! edges, each behind a [`RefCountVector`] so freed ids are reused in a fixed order, and the
//! per-vertex edge lists in a [`SmallListSet`] - with appending, the per-element getters, the
//! closed test, the compact copy, and the conversions to and from
//! [`Mesh`]. The one-ring and boundary queries are in
//! `dmesh3_queries`, the topology edits (remove, reverse, split, flip, collapse, merge, poke) in
//! `dmesh3_edge_ops`, and the `AxisAlignedBox3d` subset and g3's point-triangle distance here
//! and in `tri_distance`.
//!
//! g3's per-vertex normals, colors and UVs are not stored: nothing that reaches a [`Mesh`] reads
//! them (the conversion back copies positions only), so no normals are computed at the end of a
//! repair.

use crate::Mesh;

use super::ref_count_vector::RefCountVector;
use super::small_list_set::SmallListSet;
pub(crate) use super::tri_distance::tri_distance_sqr;

pub(crate) const INVALID_ID: i32 = -1;
pub(crate) const NON_MANIFOLD_ID: i32 = -2;

/// g3 `MeshResult`, the subset the edits return.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum MeshResult {
    Ok,
    FailedNotAVertex,
    FailedNotATriangle,
    FailedNotAnEdge,
    FailedBrokenTopology,
    FailedHitValenceLimit,
    FailedIsBoundaryEdge,
    FailedFlippedEdgeExists,
    FailedFoundDuplicateTriangle,
    FailedInvalidNeighbourhood,
    FailedCollapseTetrahedron,
    FailedCollapseTriangle,
    FailedNotABoundaryEdge,
    FailedSameOrientation,
    FailedWouldCreateBowtie,
}

/// An exception g3 throws from inside an edit (`FlipEdge`'s broken-topology checks, `RemoveVertex`
/// on a still-referenced vertex). It is passed up to the nearest handler, which in the repair is
/// `repair_cracks`'; the mesh keeps whatever the edit had already changed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct G3Exception(pub(crate) &'static str);

/// g3 `DMesh3`.
#[derive(Debug, Clone, Default)]
pub(crate) struct DMesh3 {
    pub(super) vertices: Vec<[f64; 3]>,
    pub(super) vertices_refcount: RefCountVector,
    pub(super) vertex_edges: SmallListSet,
    pub(super) triangles: Vec<[i32; 3]>,
    pub(super) triangle_edges: Vec<[i32; 3]>,
    pub(super) triangles_refcount: RefCountVector,
    pub(super) triangle_groups: Option<Vec<i32>>,
    pub(super) max_group_id: i32,
    /// `[a, b, t0, t1]` with `a < b`; `t1` is [`INVALID_ID`] on a boundary edge.
    pub(super) edges: Vec<[i32; 4]>,
    pub(super) edges_refcount: RefCountVector,
}

/// Sets slot `index`, growing the vector to reach it.
pub(super) fn insert_at<T: Copy + Default>(v: &mut Vec<T>, index: i32, value: T) {
    let index = index as usize;
    if index >= v.len() {
        v.resize(index + 1, T::default());
    }
    v[index] = value;
}

/// g3 `AxisAlignedBox3d`, the subset used here.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct AxisAlignedBox3d {
    pub(crate) min: [f64; 3],
    pub(crate) max: [f64; 3],
}

impl AxisAlignedBox3d {
    /// Zero inside.
    pub(crate) fn distance_squared(&self, v: [f64; 3]) -> f64 {
        let axis = |a: usize| {
            if v[a] < self.min[a] {
                self.min[a] - v[a]
            } else if v[a] > self.max[a] {
                v[a] - self.max[a]
            } else {
                0.0
            }
        };
        let (dx, dy, dz) = (axis(0), axis(1), axis(2));
        dx * dx + dy * dy + dz * dz
    }

    /// Inverted to the extremes, so any point grows it.
    pub(crate) const EMPTY: AxisAlignedBox3d = AxisAlignedBox3d {
        min: [f64::MAX; 3],
        max: [f64::MIN; 3],
    };

    /// The extent along x (never negative).
    pub(crate) fn width(&self) -> f64 {
        (self.max[0] - self.min[0]).max(0.0)
    }

    /// The extent along y (never negative).
    pub(crate) fn height(&self) -> f64 {
        (self.max[1] - self.min[1]).max(0.0)
    }

    /// The extent along z (never negative).
    pub(crate) fn depth(&self) -> f64 {
        (self.max[2] - self.min[2]).max(0.0)
    }

    pub(crate) fn max_dim(&self) -> f64 {
        self.width().max(self.height().max(self.depth()))
    }

    /// Grows every side by `radius`.
    pub(crate) fn expand(&mut self, radius: f64) {
        for a in 0..3 {
            self.min[a] -= radius;
            self.max[a] += radius;
        }
    }

    /// Grows to cover `other`.
    pub(crate) fn contain(&mut self, other: &AxisAlignedBox3d) {
        for a in 0..3 {
            self.min[a] = self.min[a].min(other.min[a]);
            self.max[a] = self.max[a].max(other.max[a]);
        }
    }

    /// The overlap, or [`AxisAlignedBox3d::EMPTY`] when it has no volume.
    pub(crate) fn intersect(&self, other: &AxisAlignedBox3d) -> AxisAlignedBox3d {
        let overlap = AxisAlignedBox3d {
            min: std::array::from_fn(|a| self.min[a].max(other.min[a])),
            max: std::array::from_fn(|a| self.max[a].min(other.max[a])),
        };
        if overlap.height() <= 0.0 || overlap.width() <= 0.0 || overlap.depth() <= 0.0 {
            AxisAlignedBox3d::EMPTY
        } else {
            overlap
        }
    }
}

/// g3's min/max scan: seeded with the first point, each axis tested `<` then `else if >`.
fn bounds_of(mut points: impl Iterator<Item = [f64; 3]>) -> AxisAlignedBox3d {
    let first = points.next().unwrap_or([0.0; 3]);
    let (mut min, mut max) = (first, first);
    for p in points {
        for a in 0..3 {
            if p[a] < min[a] {
                min[a] = p[a];
            } else if p[a] > max[a] {
                max[a] = p[a];
            }
        }
    }
    AxisAlignedBox3d { min, max }
}

impl DMesh3 {
    /// Turns on per-triangle groups, every existing triangle in `initial_group`.
    pub(crate) fn enable_triangle_groups(&mut self, initial_group: i32) {
        if self.triangle_groups.is_some() {
            return;
        }
        let groups = vec![initial_group; self.triangles.len()];
        self.triangle_groups = Some(groups);
        self.max_group_id = 0;
    }

    pub(crate) fn append_vertex(&mut self, v: [f64; 3]) -> i32 {
        let vid = self.vertices_refcount.allocate();
        insert_at(&mut self.vertices, vid, v);
        self.vertex_edges.allocate_at(vid);
        vid
    }

    /// Refuses (returns [`INVALID_ID`] or
    /// [`NON_MANIFOLD_ID`]) a triangle with a missing or repeated vertex, or one that would put a
    /// third triangle on an edge.
    pub(crate) fn append_triangle(&mut self, v0: i32, v1: i32, v2: i32) -> i32 {
        self.append_triangle_g(v0, v1, v2, -1)
    }

    pub(crate) fn append_triangle_g(&mut self, v0: i32, v1: i32, v2: i32, gid: i32) -> i32 {
        if !self.is_vertex(v0) || !self.is_vertex(v1) || !self.is_vertex(v2) {
            return INVALID_ID;
        }
        if v0 == v1 || v0 == v2 || v1 == v2 {
            return INVALID_ID;
        }

        // look up edges. if any already have two triangles, this would
        // create non-manifold geometry and so we do not allow it
        let e0 = self.find_edge(v0, v1);
        let e1 = self.find_edge(v1, v2);
        let e2 = self.find_edge(v2, v0);
        if (e0 != INVALID_ID && !self.is_boundary_edge(e0))
            || (e1 != INVALID_ID && !self.is_boundary_edge(e1))
            || (e2 != INVALID_ID && !self.is_boundary_edge(e2))
        {
            return NON_MANIFOLD_ID;
        }

        // now safe to insert triangle
        let tid = self.triangles_refcount.allocate();
        insert_at(&mut self.triangles, tid, [v0, v1, v2]);
        insert_at(&mut self.triangle_edges, tid, [INVALID_ID; 3]);
        if let Some(groups) = &mut self.triangle_groups {
            insert_at(groups, tid, gid);
            self.max_group_id = self.max_group_id.max(gid + 1);
        }

        // increment ref counts and update/create edges
        self.vertices_refcount.increment(v0, 1);
        self.vertices_refcount.increment(v1, 1);
        self.vertices_refcount.increment(v2, 1);

        self.add_tri_edge(tid, v0, v1, 0, e0);
        self.add_tri_edge(tid, v1, v2, 1, e1);
        self.add_tri_edge(tid, v2, v0, 2, e2);
        tid
    }

    pub(super) fn add_tri_edge(&mut self, tid: i32, v0: i32, v1: i32, j: usize, eid: i32) {
        if eid != INVALID_ID {
            self.edges[eid as usize][3] = tid;
            self.triangle_edges[tid as usize][j] = eid;
        } else {
            let eid = self.add_edge(v0, v1, tid, INVALID_ID);
            self.triangle_edges[tid as usize][j] = eid;
        }
    }

    pub(super) fn add_edge(&mut self, va: i32, vb: i32, ta: i32, tb: i32) -> i32 {
        let (va, vb) = if vb < va { (vb, va) } else { (va, vb) };
        let eid = self.edges_refcount.allocate();
        insert_at(&mut self.edges, eid, [va, vb, ta, tb]);
        self.vertex_edges.insert(va, eid);
        self.vertex_edges.insert(vb, eid);
        eid
    }

    pub(super) fn add_triangle_only(&mut self, t: [i32; 3], te: [i32; 3]) -> i32 {
        let tid = self.triangles_refcount.allocate();
        insert_at(&mut self.triangles, tid, t);
        insert_at(&mut self.triangle_edges, tid, te);
        tid
    }

    pub(crate) fn is_vertex(&self, vid: i32) -> bool {
        self.vertices_refcount.is_valid(vid)
    }

    pub(crate) fn is_triangle(&self, tid: i32) -> bool {
        self.triangles_refcount.is_valid(tid)
    }

    pub(crate) fn is_edge(&self, eid: i32) -> bool {
        self.edges_refcount.is_valid(eid)
    }

    pub(crate) fn triangle_count(&self) -> usize {
        self.triangles_refcount.count()
    }

    pub(crate) fn vertex_count(&self) -> usize {
        self.vertices_refcount.count()
    }

    pub(crate) fn edge_count(&self) -> usize {
        self.edges_refcount.count()
    }

    pub(crate) fn max_vertex_id(&self) -> i32 {
        self.vertices_refcount.max_index() as i32
    }

    pub(crate) fn max_triangle_id(&self) -> i32 {
        self.triangles_refcount.max_index() as i32
    }

    pub(crate) fn max_edge_id(&self) -> i32 {
        self.edges_refcount.max_index() as i32
    }

    #[allow(dead_code)] // only the tests call this
    pub(crate) fn is_compact(&self) -> bool {
        self.vertices_refcount.is_dense()
            && self.edges_refcount.is_dense()
            && self.triangles_refcount.is_dense()
    }

    pub(crate) fn vertex_indices(&self) -> impl Iterator<Item = i32> + '_ {
        self.vertices_refcount.indices()
    }

    pub(crate) fn triangle_indices(&self) -> impl Iterator<Item = i32> + '_ {
        self.triangles_refcount.indices()
    }

    pub(crate) fn edge_indices(&self) -> impl Iterator<Item = i32> + '_ {
        self.edges_refcount.indices()
    }

    pub(crate) fn boundary_edge_indices(&self) -> impl Iterator<Item = i32> + '_ {
        self.edges_refcount
            .indices()
            .filter(|&eid| self.edges[eid as usize][3] == INVALID_ID)
    }

    pub(crate) fn vertex(&self, vid: i32) -> [f64; 3] {
        self.vertices[vid as usize]
    }

    /// A vertex position, by the index the distance grids iterate with.
    pub(crate) fn get_vertex(&self, vid: usize) -> [f64; 3] {
        self.vertices[vid]
    }

    pub(crate) fn set_vertex(&mut self, vid: i32, v: [f64; 3]) {
        self.vertices[vid as usize] = v;
    }

    pub(crate) fn get_triangle(&self, tid: i32) -> [i32; 3] {
        self.triangles[tid as usize]
    }

    pub(crate) fn get_tri_edges(&self, tid: i32) -> [i32; 3] {
        self.triangle_edges[tid as usize]
    }

    /// `[a, b, t0, t1]`.
    pub(crate) fn get_edge(&self, eid: i32) -> [i32; 4] {
        self.edges[eid as usize]
    }

    pub(crate) fn get_edge_v(&self, eid: i32) -> [i32; 2] {
        let e = self.edges[eid as usize];
        [e[0], e[1]]
    }

    pub(crate) fn get_edge_t(&self, eid: i32) -> [i32; 2] {
        let e = self.edges[eid as usize];
        [e[2], e[3]]
    }

    pub(crate) fn get_triangle_group(&self, tid: i32) -> i32 {
        match &self.triangle_groups {
            None => -1,
            Some(groups) if self.is_triangle(tid) => groups[tid as usize],
            Some(_) => 0,
        }
    }

    #[allow(dead_code)] // only the tests call this
    pub(crate) fn set_triangle_group(&mut self, tid: i32, group_id: i32) {
        if let Some(groups) = &mut self.triangle_groups {
            groups[tid as usize] = group_id;
            self.max_group_id = self.max_group_id.max(group_id + 1);
        }
    }

    pub(crate) fn allocate_triangle_group(&mut self) -> i32 {
        self.max_group_id += 1;
        self.max_group_id
    }

    /// A triangle's three corner positions, by the index the distance grids iterate with.
    pub(crate) fn get_tri_vertices(&self, tid: usize) -> [[f64; 3]; 3] {
        let [a, b, c] = self.triangles[tid];
        [
            self.vertices[a as usize],
            self.vertices[b as usize],
            self.vertices[c as usize],
        ]
    }

    pub(crate) fn get_tri_centroid(&self, tid: usize) -> [f64; 3] {
        let [a, b, c] = self.get_tri_vertices(tid);
        let f = 1.0 / 3.0;
        [
            (a[0] + b[0] + c[0]) * f,
            (a[1] + b[1] + c[1]) * f,
            (a[2] + b[2] + c[2]) * f,
        ]
    }

    pub(crate) fn get_tri_bounds(&self, tid: usize) -> AxisAlignedBox3d {
        let corners = self.get_tri_vertices(tid);
        bounds_of(corners.iter().copied())
    }

    /// The box of every live vertex, zero when there are none.
    pub(crate) fn cached_bounds(&self) -> AxisAlignedBox3d {
        if self.vertex_count() == 0 {
            return AxisAlignedBox3d {
                min: [0.0; 3],
                max: [0.0; 3],
            };
        }
        bounds_of(self.vertex_indices().map(|vid| self.vertices[vid as usize]))
    }

    /// Has triangles and no boundary edge.
    pub(crate) fn is_closed(&self) -> bool {
        if self.triangle_count() == 0 {
            return false;
        }
        self.boundary_edge_indices().next().is_none()
    }

    #[allow(dead_code)] // only the tests call this
    /// Live vertices re-appended in id order, then live triangles; a mesh that is already compact
    /// is copied as is.
    pub(crate) fn compact_copy(&self) -> DMesh3 {
        if self.is_compact() {
            return self.clone();
        }
        let mut out = DMesh3::default();
        if self.triangle_groups.is_some() {
            out.triangle_groups = Some(Vec::new());
        }
        let mut map_v = vec![0; self.max_vertex_id() as usize];
        for vid in self.vertex_indices() {
            map_v[vid as usize] = out.append_vertex(self.vertices[vid as usize]);
        }
        for tid in self.triangle_indices() {
            let t = self.get_triangle(tid);
            let g = if self.triangle_groups.is_some() {
                self.get_triangle_group(tid)
            } else {
                INVALID_ID
            };
            out.append_triangle_g(
                map_v[t[0] as usize],
                map_v[t[1] as usize],
                map_v[t[2] as usize],
                g,
            );
            out.max_group_id = out.max_group_id.max(g + 1);
        }
        out
    }
}

impl Mesh {
    /// Every vertex, then every face in order (faces `DMesh3` refuses are dropped).
    pub(crate) fn to_dmesh3(&self) -> DMesh3 {
        let mut out = DMesh3::default();
        for vertex in &self.vertices {
            out.append_vertex([vertex.x as f64, vertex.y as f64, vertex.z as f64]);
        }
        for face in self.faces.iter() {
            out.append_triangle(face.v0, face.v1, face.v2);
        }
        out
    }
}

impl DMesh3 {
    /// Live vertices, then live triangles, in id order, with the vertex ids compacted.
    pub(crate) fn to_mesh(&self) -> Mesh {
        let mut out = Mesh::new();
        let mut map_v = vec![0; self.max_vertex_id() as usize];
        for (accum, vid) in self.vertex_indices().enumerate() {
            map_v[vid as usize] = accum as i32;
            let v = self.vertices[vid as usize];
            out.vertices
                .push(solidmesh_math::Vector3Float::new(v[0], v[1], v[2]));
        }
        for tid in self.triangle_indices() {
            let t = self.get_triangle(tid);
            out.faces.add(
                map_v[t[0] as usize],
                map_v[t[1] as usize],
                map_v[t[2] as usize],
                &out.vertices,
            );
        }
        out
    }
}
