//! The region operator (derived from geometry3Sharp): extracts a set of triangles as a
//! `DSubmesh3` (`dsubmesh3`), lets the caller edit that submesh, then replaces the region in
//! the base mesh with the edited copy (`back_propropagate`, via `mesh_editor::reinsert_submesh`)
//! as long as the region's boundary was kept. `MinimalHoleFill` runs its fill optimisation
//! this way.
//!
//! Only the base-mesh triangle ids of the re-inserted region are kept; nothing reads the
//! submesh-to-base vertex and triangle maps, so they are not stored. A duplicate triangle on
//! re-insert is refused by the mesh, not checked for in advance.

use std::collections::HashSet;

use super::dmesh3::{DMesh3, G3Exception, INVALID_ID};
use super::dsubmesh3::DSubmesh3;
use super::mesh_editor::{reinsert_submesh, remove_triangles_list};

/// A region of a base mesh, extracted for editing and spliced back.
pub(crate) struct RegionOperator<'a> {
    /// The mesh the region belongs to.
    pub(crate) base_mesh: &'a mut DMesh3,
    /// The region as an editable submesh.
    pub(crate) region: DSubmesh3,
    cur_base_tris: Vec<i32>,
}

impl<'a> RegionOperator<'a> {
    /// `Err` when a region triangle is dead,
    /// which `DSubmesh3.compute` throws.
    pub(crate) fn new(mesh: &'a mut DMesh3, region_tris: &[i32]) -> Result<Self, G3Exception> {
        let mut region = DSubmesh3::new();
        region.compute(mesh, region_tris)?;
        region.compute_boundary_info(mesh, region_tris);
        Ok(Self {
            base_mesh: mesh,
            region,
            cur_base_tris: region_tris.to_vec(),
        })
    }

    /// The input region triangles, or after
    /// [`RegionOperator::back_propropagate`] the base-mesh ids of the re-inserted submesh.
    pub(crate) fn current_base_triangles(&self) -> &[i32] {
        &self.cur_base_tris
    }

    /// The region's base-mesh vertices not on its border, in
    /// first-seen order, so the hole fill visits them the same way on every run. A negative
    /// id in the current triangles (a refused re-insert) has no vertices; that is `Err`.
    pub(crate) fn current_base_interior_vertices(&self) -> Result<Vec<i32>, G3Exception> {
        let mut verts = Vec::new();
        let mut seen = HashSet::new();
        for &tid in &self.cur_base_tris {
            if tid < 0 {
                return Err(G3Exception(
                    "RegionOperator.CurrentBaseInteriorVertices: Index was outside the bounds of the array.",
                ));
            }
            let tv = self.base_mesh.get_triangle(tid);
            for v in tv {
                if !self.region.is_base_border_v(v) && seen.insert(v) {
                    verts.push(v);
                }
            }
        }
        Ok(verts)
    }

    /// Splits each interior submesh edge whose two
    /// (boundary) vertices map to base vertices already joined by an edge, which re-insertion
    /// could otherwise not add without a non-manifold edge.
    pub(crate) fn repair_possible_non_manifold_edges(&mut self) {
        // repair submesh
        let sub = &self.region.sub_mesh;
        let ne = sub.max_edge_id();
        let mut split_edges = Vec::new();
        for eid in 0..ne {
            if !sub.is_edge(eid) {
                continue;
            }
            if sub.is_boundary_edge(eid) {
                continue;
            }

            let edgev = sub.get_edge_v(eid);
            if sub.is_boundary_vertex(edgev[0]) && sub.is_boundary_vertex(edgev[1]) {
                // ok, we have an internal edge where both verts are on the boundary
                // now check if it is an edge in the base mesh
                let base_a = self.region.map_vertex_to_base_mesh(edgev[0]);
                let base_b = self.region.map_vertex_to_base_mesh(edgev[1]);
                if base_a != INVALID_ID && base_b != INVALID_ID {
                    // both vertices in base mesh...right?
                    let base_eid = self.base_mesh.find_edge(base_a, base_b);
                    if base_eid != INVALID_ID {
                        split_edges.push(eid);
                    }
                }
            }
        }

        // split any problem edges we found and repeat this loop
        for &eid in &split_edges {
            let _ = self.region.sub_mesh.split_edge(eid, 0.5);
        }
    }

    /// Removes the current region triangles from
    /// the base mesh and inserts the submesh in their place. False if any submesh triangle was
    /// refused; changes that succeeded are kept.
    pub(crate) fn back_propropagate(
        &mut self,
        allow_submesh_repairs: bool,
    ) -> Result<bool, G3Exception> {
        if allow_submesh_repairs {
            self.repair_possible_non_manifold_edges();
        }

        // remove existing submesh triangles
        remove_triangles_list(self.base_mesh, &self.cur_base_tris, true);

        // insert new submesh
        let mut new_tris = vec![0; self.region.sub_mesh.triangle_count()];
        let ok = reinsert_submesh(self.base_mesh, &self.region, &mut new_tris)?;

        self.cur_base_tris = new_tris;
        Ok(ok)
    }
}
