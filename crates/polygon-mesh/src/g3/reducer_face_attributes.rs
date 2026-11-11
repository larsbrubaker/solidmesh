//! The attribute quadrics `ReducerFaceData` (its parent, `reducer_face_data.rs`) keeps for the
//! reducer: each region's UV-to-model scale `λ` = √(surface area / UV area), and per free vertex
//! of a textured region the area-weighted sum of its triangles' lifted plane quadrics
//! (`attribute_quadric.rs`). A collapse of two free vertices adds theirs; seam vertices carry
//! none, since their corners hold several UVs.

use super::super::attribute_quadric::{AttributeQuadric, Point5};
use super::super::dmesh3::DMesh3;
use super::{ReducerFaceData, FREE};

/// Triangle `tid`'s area.
fn triangle_area(mesh: &DMesh3, tid: i32) -> f64 {
    let [p0, p1, p2] = mesh.get_tri_vertices(tid as usize);
    let e1 = [p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]];
    let e2 = [p2[0] - p0[0], p2[1] - p0[1], p2[2] - p0[2]];
    let n = [
        e1[1] * e2[2] - e1[2] * e2[1],
        e1[2] * e2[0] - e1[0] * e2[2],
        e1[0] * e2[1] - e1[1] * e2[0],
    ];
    0.5 * (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt()
}

impl ReducerFaceData {
    /// Each region's UV-to-model scale, summed over its triangles in id order.
    pub(super) fn region_scales(&self, mesh: &DMesh3) -> Vec<f64> {
        let mut areas = vec![(0.0f64, 0.0f64); self.regions.len()];
        for tid in mesh.triangle_indices() {
            let region = self.tri_region[tid as usize] as usize;
            let [u0, u1, u2] = self.tri_uv[tid as usize];
            let uv_cross = (u1[0] - u0[0]) * (u2[1] - u0[1]) - (u1[1] - u0[1]) * (u2[0] - u0[0]);
            areas[region].0 += triangle_area(mesh, tid);
            areas[region].1 += 0.5 * uv_cross.abs();
        }
        areas
            .iter()
            .zip(&self.regions)
            .map(|(&(surface, uv), region)| {
                if region.is_textured() && uv > 0.0 && surface > 0.0 {
                    (surface / uv).sqrt()
                } else {
                    0.0
                }
            })
            .collect()
    }

    /// Triangle `tid` lifted into the attribute space, corner by corner.
    fn lifted_corners(&self, mesh: &DMesh3, tid: i32) -> [Point5; 3] {
        let lambda = self.region_lambda[self.tri_region[tid as usize] as usize];
        let positions = mesh.get_tri_vertices(tid as usize);
        let uvs = self.tri_uv[tid as usize];
        [0, 1, 2].map(|c| {
            let p = positions[c];
            [p[0], p[1], p[2], lambda * uvs[c][0], lambda * uvs[c][1]]
        })
    }

    /// Builds the attribute quadric of every free vertex in a textured region: the area-weighted
    /// sum of its triangles' lifted plane quadrics, added in triangle id order. Seam vertices get
    /// none: their corners carry several UVs, and their collapses are placed by the seam rules. A
    /// mesh with no textured region builds nothing.
    pub(crate) fn init_attributes(&mut self, mesh: &DMesh3) {
        if self.region_lambda.iter().all(|&lambda| lambda <= 0.0) {
            return;
        }
        let nv = mesh.max_vertex_id() as usize;
        self.attributes = vec![AttributeQuadric::ZERO; nv];
        self.has_attribute = (0..nv)
            .map(|vid| mesh.is_vertex(vid as i32) && self.vertex_kind[vid] == FREE)
            .collect();
        for tid in mesh.triangle_indices() {
            let tri = mesh.get_triangle(tid);
            if self.region_lambda[self.tri_region[tid as usize] as usize] <= 0.0 {
                for v in tri {
                    self.has_attribute[v as usize] = false;
                }
                continue;
            }
            let [p0, p1, p2] = self.lifted_corners(mesh, tid);
            let quadric = AttributeQuadric::from_triangle(&p0, &p1, &p2, triangle_area(mesh, tid));
            for v in tri {
                if self.has_attribute[v as usize] {
                    self.attributes[v as usize].add(&quadric);
                }
            }
        }
    }

    /// Vertex `vid`'s attribute quadric, when it is a free vertex in a textured region.
    pub(crate) fn attribute(&self, vid: i32) -> Option<&AttributeQuadric> {
        if self
            .has_attribute
            .get(vid as usize)
            .copied()
            .unwrap_or(false)
        {
            Some(&self.attributes[vid as usize])
        } else {
            None
        }
    }

    /// The UV-to-model scale of the region around free vertex `vid`, read off triangle `tid`.
    pub(crate) fn lambda_of(&self, tid: i32) -> f64 {
        self.region_lambda[self.tri_region[tid as usize] as usize]
    }

    /// Vertex `vid` lifted into the attribute space with its UV in triangle `tid`.
    pub(crate) fn lifted_vertex(&self, mesh: &DMesh3, tid: i32, vid: i32) -> Point5 {
        let lambda = self.lambda_of(tid);
        let p = mesh.vertex(vid);
        let uv = self.corner_uv(mesh, tid, vid);
        [p[0], p[1], p[2], lambda * uv[0], lambda * uv[1]]
    }

    /// After `r` collapsed into `k`: two free vertices' attribute quadrics add up.
    pub(crate) fn merge_attributes(&mut self, k: i32, r: i32) {
        match (self.attribute(k), self.attribute(r)) {
            (Some(qk), Some(qr)) => {
                self.attributes[k as usize] = AttributeQuadric::sum(qk, qr);
            }
            _ => {
                if let Some(has) = self.has_attribute.get_mut(k as usize) {
                    *has = false;
                }
            }
        }
        if let Some(has) = self.has_attribute.get_mut(r as usize) {
            *has = false;
        }
    }
}
