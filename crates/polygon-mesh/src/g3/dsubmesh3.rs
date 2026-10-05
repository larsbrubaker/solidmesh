//! geometry3Sharp `mesh/DSubmesh3.cs`: a copy of a set of a base mesh's triangles as its own
//! `DMesh3`, with the vertex maps between the two and the region's border vertices on the base
//! mesh, so `RegionOperator` (`region_operator`) can edit the copy and splice it back.
//!
//! The subset `MinimalHoleFill` reaches: `Compute`, `ComputeBoundaryInfo` and the vertex maps.
//! Nothing ported reads the triangle maps (`ComputeTriMaps`, `BaseToSubT` / `SubToBaseT`) or the
//! border/boundary edge sets (`BaseBorderE`, `BaseBoundaryE`), so they are not kept; only the
//! vertex positions are copied, as `DMesh3` stores no normals, colors or UVs.
//! C#'s dense `IndexMap` / `IndexFlagSet` are `Vec`s (the sparse `IndexMap` variant returns the
//! same values for every index that is read), its `DVector<int>` a `Vec<i32>`.

use std::collections::HashSet;

use super::dmesh3::{DMesh3, G3Exception, INVALID_ID};

/// C# `IndexMap.InvalidIndex`.
pub(crate) const INVALID_INDEX: i32 = i32::MIN;

/// g3 `DSubmesh3` (subset). The base mesh is passed to each method rather than held, so the
/// caller can edit the submesh while it reads the base mesh.
#[derive(Debug, Clone, Default)]
pub(crate) struct DSubmesh3 {
    /// C# `SubMesh`.
    pub(crate) sub_mesh: DMesh3,
    /// C# `OverrideGroupID`: when `>= 0`, every submesh triangle gets this group.
    pub(crate) override_group_id: i32,
    /// C# `BaseSubmeshV`: base vertices that are in the submesh.
    pub(crate) base_submesh_v: Vec<bool>,
    /// C# `BaseToSubV` (dense; [`INVALID_INDEX`] where unset).
    pub(crate) base_to_sub_v: Vec<i32>,
    /// C# `SubToBaseV`.
    pub(crate) sub_to_base_v: Vec<i32>,
    /// C# `BaseBorderV`: base vertices on the region's border with the rest of the base mesh
    /// (not mesh-boundary vertices). `None` until [`DSubmesh3::compute_boundary_info`].
    pub(crate) base_border_v: Option<HashSet<i32>>,
}

impl DSubmesh3 {
    /// C# `new DSubmesh3(mesh)`.
    pub(crate) fn new() -> Self {
        Self {
            override_group_id: -1,
            ..Self::default()
        }
    }

    /// C# `Compute(int[] subTriangles)` (`compute`): appends the triangles' vertices in first-use
    /// order and the triangles in the given order. Throws in C# on a dead triangle id, after
    /// the triangles before it were copied.
    pub(crate) fn compute(&mut self, base: &DMesh3, triangles: &[i32]) -> Result<(), G3Exception> {
        // C# `new DMesh3(BaseMesh.Components & WantComponents)`: groups if the base has them.
        self.sub_mesh = DMesh3::default();
        if base.triangle_groups.is_some() {
            self.sub_mesh.enable_triangle_groups(0);
        }

        let max_vid = base.max_vertex_id() as usize;
        self.base_submesh_v = vec![false; max_vid];
        self.base_to_sub_v = vec![INVALID_INDEX; max_vid];
        self.sub_to_base_v = Vec::new();

        for &tid in triangles {
            if !base.is_triangle(tid) {
                return Err(G3Exception(
                    "DSubmesh3.compute: triangle does not exist in BaseMesh!",
                ));
            }

            let base_t = base.get_triangle(tid);
            let mut new_t = [0; 3];
            let mut gid = base.get_triangle_group(tid);

            for j in 0..3 {
                let base_v = base_t[j];
                let sub_v;
                if !self.base_submesh_v[base_v as usize] {
                    sub_v = self.sub_mesh.append_vertex(base.vertex(base_v));
                    self.base_submesh_v[base_v as usize] = true;
                    self.base_to_sub_v[base_v as usize] = sub_v;
                    super::dmesh3::insert_at(&mut self.sub_to_base_v, sub_v, base_v);
                } else {
                    sub_v = self.base_to_sub_v[base_v as usize];
                }
                new_t[j] = sub_v;
            }

            if self.override_group_id >= 0 {
                gid = self.override_group_id;
            }

            self.sub_mesh
                .append_triangle_g(new_t[0], new_t[1], new_t[2], gid);
        }
        Ok(())
    }

    #[allow(dead_code)] // geometry3Sharp API no ported caller reaches yet; the tests do
    /// C# `MapVertexToSubmesh`.
    pub(crate) fn map_vertex_to_submesh(&self, base_vid: i32) -> i32 {
        self.base_to_sub_v[base_vid as usize]
    }

    /// C# `MapVertexToBaseMesh`: [`INVALID_ID`] for a vertex added to the submesh since it was
    /// computed.
    pub(crate) fn map_vertex_to_base_mesh(&self, sub_vid: i32) -> i32 {
        if (sub_vid as usize) < self.sub_to_base_v.len() {
            return self.sub_to_base_v[sub_vid as usize];
        }
        INVALID_ID
    }

    /// C# `ComputeBoundaryInfo(int[] subTriangles)`: the base vertices of every base edge with
    /// exactly one of its two triangles in the region.
    pub(crate) fn compute_boundary_info(&mut self, base: &DMesh3, triangles: &[i32]) {
        // set of base-mesh triangles that are in submesh
        let mut sub_tris = vec![false; base.max_triangle_id() as usize];
        for &ti in triangles {
            sub_tris[ti as usize] = true;
        }

        let mut border_v = HashSet::new();

        // Iterate through edges in submesh roi on base mesh. If
        // one of the tris of the edge is not in submesh roi, then this
        // is a boundary edge.
        //
        // (edge iteration via triangle iteration processes each internal edge twice...)
        for &ti in triangles {
            let tedges = base.get_tri_edges(ti);
            for eid in tedges {
                let tris = base.get_edge_t(eid);
                if tris[1] == INVALID_ID {
                    // this is a boundary edge (C# `BaseBoundaryE`, not kept)
                } else if sub_tris[tris[0] as usize] != sub_tris[tris[1] as usize] {
                    // this is a border edge
                    let ve = base.get_edge_v(eid);
                    border_v.insert(ve[0]);
                    border_v.insert(ve[1]);
                }
            }
        }
        self.base_border_v = Some(border_v);
    }

    /// C# `BaseBorderV[vid]`.
    pub(crate) fn is_base_border_v(&self, vid: i32) -> bool {
        self.base_border_v
            .as_ref()
            .is_some_and(|set| set.contains(&vid))
    }
}
