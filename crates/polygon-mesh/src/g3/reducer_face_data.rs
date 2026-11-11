//! The face data the reducer (`reducer.rs`) carries through its edge collapses: each triangle's
//! region (the picture it wears, or none, and its face colour) and the UV of each of its corners,
//! in a per-triangle corner table beside the `DMesh3`, indexed by triangle id. `DMesh3`'s collapse
//! keeps the ids of the triangles it does not remove and rewrites the removed vertex in place, so
//! a row stays with its triangle and its corners stay in order.
//!
//! **Seams.** An edge is a seam when its two triangles disagree at either shared endpoint: a
//! different region (another picture, a different colour, or one textured and the other not) or
//! different UV bits at that corner. Seams are feature lines in the refiner's constraint set
//! (`mesh_constraints.rs`): every seam edge carries the seam target, a vertex on exactly two seam
//! edges (and not on the mesh boundary) is held to that target, and any other seam vertex - where
//! three or more seams meet, where a seam ends, or where it reaches the boundary - is pinned and
//! on the seam target too. `MeshRefinerBase::can_collapse_vtx` then lets a seam edge collapse
//! only along itself (the reducer puts its collapse point on the edge), collapses an edge from a
//! free vertex onto its seam end and a seam vertex onto a pinned neighbour on its seam; the
//! reducer (`reducer_seams.rs`) refuses an edge joining two pinned vertices, or two seam vertices
//! without being a seam. So every triangle stays inside one region of the input, and each output
//! face takes its region exactly.
//!
//! **UVs ride the collapses.** The corners around a vertex fall into wedges - the fans between
//! its seam edges - and a wedge shares one UV at that vertex. When a seam edge `[r, k]` collapses
//! (`r` removed) to a point at parameter `s` along it, each wedge through the edge takes, at the
//! kept vertex, `(1 - s) * uv_r + s * uv_k` of its two endpoint UVs (read off the triangle beside
//! the edge on that wedge's side); a collapse onto `k` itself gives `r`'s corners `k`'s UV on their
//! side exactly. Between two free vertices the point takes the UV of the attribute quadric's
//! optimum, or, where that quadric has none, of the nearest point of their fan. Wedges of `k` the
//! edge does not reach are not touched.
//!
//! The seam kinds are also kept in dense per-vertex and per-edge arrays, which the reducer's
//! scoring (`reducer_seams.rs`) and the wedge walks read instead of the constraint maps.
//!
//! **Attribute quadrics** (the child module `reducer_face_attributes.rs`). Each free vertex of a
//! textured region carries the area-weighted sum of its triangles' lifted plane quadrics
//! (`attribute_quadric.rs`), with the region's UV-to-model scale `λ` = √(surface area / UV
//! area); a collapse of two free vertices adds them. Seam vertices carry none: their corners hold
//! several UVs, and their collapses are placed by the seam rules. A mesh with no textured region
//! (colours only) builds none.
//!
//! A mesh with neither textures nor face colours never builds this table, so its reduction runs
//! exactly as it did without one.

use std::collections::HashMap;
use std::rc::Rc;
use std::sync::Arc;

use solidmesh_math::texture::TexturePixels;
use solidmesh_math::{Color, Vector2Float};

use super::attribute_quadric::AttributeQuadric;
use super::dmesh3::{DMesh3, INVALID_ID};
use super::mesh_constraints::{
    edge_refine_flags, EdgeConstraint, MeshConstraints, ProjectionTarget, VertexConstraint,
};
use super::tri_distance::distance_sqr_closest_params;
use crate::face_data::FaceTextureData;
use crate::Mesh;

#[path = "reducer_face_attributes.rs"]
mod attributes;

/// The picture and colour one group of faces wears.
#[derive(Clone)]
struct Region {
    /// `None` for a face with no texture entry; `Some(image)` for one with an entry (whose image
    /// may itself be absent).
    texture: Option<Option<Arc<dyn TexturePixels>>>,
    /// The face colour, when the mesh has face colours.
    color: Option<Color>,
}

impl Region {
    fn is_textured(&self) -> bool {
        self.texture.is_some()
    }
}

/// The target seam vertices are held to. Points reaching it already lie on the seam - the
/// reducer places a seam edge's collapse point on the edge itself - so projecting keeps them.
struct SeamTarget;

impl ProjectionTarget for SeamTarget {
    fn project(&self, point: [f64; 3], _identifier: i32) -> [f64; 3] {
        point
    }
}

/// A UV pair, in f64 so interpolation is exact to the input's f32 values at the ends.
type Uv = [f64; 2];

/// Every triangle's region and corner UVs while a reduce runs.
pub(crate) struct ReducerFaceData {
    /// Region per triangle id.
    tri_region: Vec<u32>,
    /// Corner UVs per triangle id, in the triangle's vertex order.
    tri_uv: Vec<[Uv; 3]>,
    regions: Vec<Region>,
    /// Whether the source mesh had face colours (so the result gets them too).
    has_colors: bool,
    /// Per region, the scale from UV units to model units: the square root of its surface area
    /// over its UV area (0 for an untextured region or one with no UV area).
    region_lambda: Vec<f64>,
    /// Per vertex id, the attribute quadric of a free vertex in a textured region (empty when
    /// no region is textured), and whether the vertex has one.
    attributes: Vec<AttributeQuadric>,
    has_attribute: Vec<bool>,
    /// Per vertex id, [`FREE`], [`ON_SEAM`] or [`PINNED`] - the seam constraints in a dense
    /// array, so scoring an edge looks them up without hashing.
    vertex_kind: Vec<u8>,
    /// Per edge id, whether the edge is a seam.
    edge_seam: Vec<bool>,
}

/// A vertex on no seam.
pub(crate) const FREE: u8 = 0;
/// A vertex on exactly two seam edges, held to the seam.
pub(crate) const ON_SEAM: u8 = 1;
/// A seam junction, seam end or seam reaching the boundary: it stays where it is.
pub(crate) const PINNED: u8 = 2;

/// The UV each triangle around the kept vertex takes once a collapse succeeds.
pub(crate) struct CollapseUvPlan {
    updates: Vec<(i32, Uv)>,
}

fn uv_bits(uv: Uv) -> [u64; 2] {
    [uv[0].to_bits(), uv[1].to_bits()]
}

impl Mesh {
    /// The mesh as a `DMesh3` (as [`Mesh::to_dmesh3`] builds it) with its face data, or no face
    /// data when the mesh has neither face textures nor face colours.
    pub(crate) fn to_dmesh3_with_face_data(&self) -> (DMesh3, Option<ReducerFaceData>) {
        if self.face_textures.is_empty() && self.face_colors.is_none() {
            return (self.to_dmesh3(), None);
        }
        let mut out = DMesh3::default();
        for vertex in &self.vertices {
            out.append_vertex([vertex.x as f64, vertex.y as f64, vertex.z as f64]);
        }
        let mut data = ReducerFaceData {
            tri_region: Vec::new(),
            tri_uv: Vec::new(),
            regions: Vec::new(),
            has_colors: self.face_colors.is_some(),
            region_lambda: Vec::new(),
            attributes: Vec::new(),
            has_attribute: Vec::new(),
            vertex_kind: Vec::new(),
            edge_seam: Vec::new(),
        };
        // Region ids are handed out in face order, so the table is the same on every run; the
        // map is only looked up.
        let mut region_ids: HashMap<(bool, usize, Option<[u8; 4]>), u32> = HashMap::new();
        for (face_index, face) in self.faces.iter().enumerate() {
            let tid = out.append_triangle(face.v0, face.v1, face.v2);
            if tid < 0 {
                continue;
            }
            let entry = self.face_textures.get(&(face_index as i32));
            let color = self.face_colors.as_ref().map(|colors| {
                colors
                    .get(face_index)
                    .copied()
                    .unwrap_or(Mesh::UNKNOWN_FACE_COLOR)
            });
            let image_address = entry
                .and_then(|t| t.image.as_ref())
                .map_or(0, |image| Arc::as_ptr(image) as *const u8 as usize);
            let key = (
                entry.is_some(),
                image_address,
                color.map(|c| [c.red, c.green, c.blue, c.alpha]),
            );
            let next = data.regions.len() as u32;
            let region = *region_ids.entry(key).or_insert(next);
            if region == next {
                data.regions.push(Region {
                    texture: entry.map(|t| t.image.clone()),
                    color,
                });
            }
            let uv = entry.map_or([[0.0; 2]; 3], |t| {
                [t.uv0, t.uv1, t.uv2].map(|u| [u.x as f64, u.y as f64])
            });
            let slot = tid as usize;
            if data.tri_region.len() <= slot {
                data.tri_region.resize(slot + 1, 0);
                data.tri_uv.resize(slot + 1, [[0.0; 2]; 3]);
            }
            data.tri_region[slot] = region;
            data.tri_uv[slot] = uv;
        }
        data.region_lambda = data.region_scales(&out);
        (out, Some(data))
    }
}

impl ReducerFaceData {
    /// Triangle `tid`'s UV at its corner on vertex `vid`.
    fn corner_uv(&self, mesh: &DMesh3, tid: i32, vid: i32) -> Uv {
        let tri = mesh.get_triangle(tid);
        let corner = tri.iter().position(|&v| v == vid).unwrap_or(0);
        self.tri_uv[tid as usize][corner]
    }

    /// Whether edge `eid`'s two triangles disagree at either shared endpoint.
    fn is_seam(&self, mesh: &DMesh3, eid: i32) -> bool {
        let [a, b, t0, t1] = mesh.get_edge(eid);
        if t1 == INVALID_ID {
            return false;
        }
        let r0 = &self.regions[self.tri_region[t0 as usize] as usize];
        if self.tri_region[t0 as usize] != self.tri_region[t1 as usize] {
            return true;
        }
        if !r0.is_textured() {
            return false;
        }
        uv_bits(self.corner_uv(mesh, t0, a)) != uv_bits(self.corner_uv(mesh, t1, a))
            || uv_bits(self.corner_uv(mesh, t0, b)) != uv_bits(self.corner_uv(mesh, t1, b))
    }

    /// The seams as refiner constraints: each seam edge on the seam target, each vertex on two
    /// seam edges held to it, every other seam vertex pinned.
    pub(crate) fn seam_constraints(&mut self, mesh: &DMesh3) -> MeshConstraints {
        let target: Rc<dyn ProjectionTarget> = Rc::new(SeamTarget);
        let mut constraints = MeshConstraints::default();
        let mut seam_count = vec![0u32; mesh.max_vertex_id() as usize];
        self.vertex_kind = vec![FREE; mesh.max_vertex_id() as usize];
        self.edge_seam = vec![false; mesh.max_edge_id() as usize];
        for eid in 0..mesh.max_edge_id() {
            if mesh.is_edge(eid) && self.is_seam(mesh, eid) {
                self.edge_seam[eid as usize] = true;
                constraints.set_or_update_edge_constraint(
                    eid,
                    EdgeConstraint::with_target(edge_refine_flags::NO_FLIP, Some(target.clone())),
                );
                let [a, b] = mesh.get_edge_v(eid);
                seam_count[a as usize] += 1;
                seam_count[b as usize] += 1;
            }
        }
        for (vid, &count) in seam_count.iter().enumerate() {
            let vid = vid as i32;
            if count == 0 {
                continue;
            }
            if count == 2 && !mesh.is_boundary_vertex(vid) {
                self.vertex_kind[vid as usize] = ON_SEAM;
                constraints.set_or_update_vertex_constraint(
                    vid,
                    VertexConstraint::new_target(Some(target.clone())),
                );
            } else {
                // Pinned, and on the seam: a seam vertex beside it may collapse onto it.
                self.vertex_kind[vid as usize] = PINNED;
                let mut pinned = VertexConstraint::pinned();
                pinned.target = Some(target.clone());
                constraints.set_or_update_vertex_constraint(vid, pinned);
            }
        }
        constraints
    }

    /// Vertex `vid`'s seam kind: [`FREE`], [`ON_SEAM`] or [`PINNED`].
    pub(crate) fn vertex_kind(&self, vid: i32) -> u8 {
        self.vertex_kind[vid as usize]
    }

    /// Whether edge `eid` is a seam.
    pub(crate) fn is_seam_edge(&self, eid: i32) -> bool {
        self.edge_seam[eid as usize]
    }

    /// The triangles around `vid` reached from `start` without crossing a seam edge (the wedge
    /// `start` is in), `start` first.
    fn wedge(&self, mesh: &DMesh3, vid: i32, start: i32) -> Vec<i32> {
        let mut found = vec![start];
        let mut next = 0;
        while next < found.len() {
            let tid = found[next];
            next += 1;
            for eid in mesh.get_tri_edges(tid) {
                let ev = mesh.get_edge_v(eid);
                if (ev[0] != vid && ev[1] != vid) || self.edge_seam[eid as usize] {
                    continue;
                }
                let other = mesh.edge_other_t(eid, tid);
                if other != INVALID_ID && !found.contains(&other) {
                    found.push(other);
                }
            }
        }
        found
    }

    /// The UV at the point of `fan` (triangles of one wedge) nearest `point`; the first of
    /// equally near triangles wins.
    fn uv_at(&self, mesh: &DMesh3, fan: &[i32], point: [f64; 3]) -> Uv {
        let mut best = (f64::INFINITY, [0.0; 2]);
        for &tid in fan {
            let corners = mesh.get_tri_vertices(tid as usize);
            let (d, _, [s, t]) = distance_sqr_closest_params(&corners, point);
            if d < best.0 || best.0.is_infinite() {
                let uv = self.tri_uv[tid as usize];
                let w0 = 1.0 - s - t;
                best = (
                    d,
                    [
                        w0 * uv[0][0] + s * uv[1][0] + t * uv[2][0],
                        w0 * uv[0][1] + s * uv[1][1] + t * uv[2][1],
                    ],
                );
            }
        }
        best.1
    }

    /// What collapsing edge `eid` = `[r, k]` (removing `r`, triangles `t0` and `t1` beside it)
    /// to `point`, at parameter `s` along `r -> k`, does to the UVs. A collapse onto `k` itself
    /// (`onto_kept`) gives `r`'s corners `k`'s UV on their side; on a seam edge (where `point` lies
    /// on the edge) each wedge through it takes the interpolated UV at the kept vertex; between
    /// two free vertices the point takes `uv`, the attribute quadric's optimum, or else the UV of
    /// the nearest point of their fan.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn plan_collapse(
        &self,
        mesh: &DMesh3,
        eid: i32,
        (r, k): (i32, i32),
        (t0, t1): (i32, i32),
        s: f64,
        point: [f64; 3],
        uv: Option<Uv>,
        onto_kept: bool,
    ) -> CollapseUvPlan {
        let mut updates = Vec::new();
        let on_seam = self.edge_seam[eid as usize];
        if !on_seam && !onto_kept {
            // Both ends are free, so one wedge holds every triangle around them: the collapse
            // point takes the UV of the nearest point of that fan as it is now.
            let mut fan = self.wedge(mesh, r, t0);
            for tid in self.wedge(mesh, k, t0) {
                if !fan.contains(&tid) {
                    fan.push(tid);
                }
            }
            let uv = uv.unwrap_or_else(|| self.uv_at(mesh, &fan, point));
            for tid in fan {
                if tid != t0 && tid != t1 {
                    updates.push((tid, uv));
                }
            }
            return CollapseUvPlan { updates };
        }
        for side in [t0, t1] {
            if side == INVALID_ID {
                continue;
            }
            let uv_r = self.corner_uv(mesh, side, r);
            let uv_k = self.corner_uv(mesh, side, k);
            let uv = if onto_kept {
                uv_k
            } else {
                [
                    (1.0 - s) * uv_r[0] + s * uv_k[0],
                    (1.0 - s) * uv_r[1] + s * uv_k[1],
                ]
            };
            let mut fans = self.wedge(mesh, r, side);
            if !onto_kept {
                fans.extend(self.wedge(mesh, k, side));
            }
            for tid in fans {
                if tid != t0 && tid != t1 && !updates.iter().any(|&(t, _)| t == tid) {
                    updates.push((tid, uv));
                }
            }
        }
        CollapseUvPlan { updates }
    }

    /// Applies a collapse's plan once `k` has replaced `r` in its triangles.
    pub(crate) fn apply_collapse(&mut self, mesh: &DMesh3, k: i32, plan: CollapseUvPlan) {
        for (tid, uv) in plan.updates {
            let tri = mesh.get_triangle(tid);
            if let Some(corner) = tri.iter().position(|&v| v == k) {
                self.tri_uv[tid as usize][corner] = uv;
            }
        }
    }

    /// Writes each live triangle's region and UVs onto `out`, the mesh `DMesh3::to_mesh` built
    /// from `mesh` (one face per live triangle, in id order).
    pub(crate) fn write_faces(&self, mesh: &DMesh3, out: &mut Mesh) {
        let mut colors = Vec::new();
        for (face, tid) in mesh.triangle_indices().enumerate() {
            let region = &self.regions[self.tri_region[tid as usize] as usize];
            if let Some(image) = &region.texture {
                let [uv0, uv1, uv2] =
                    self.tri_uv[tid as usize].map(|u| Vector2Float::new(u[0], u[1]));
                out.face_textures.insert(
                    face as i32,
                    FaceTextureData::new(image.clone(), uv0, uv1, uv2),
                );
            }
            colors.push(region.color.unwrap_or(Mesh::UNKNOWN_FACE_COLOR));
        }
        if self.has_colors {
            out.face_colors = Some(colors);
        }
    }
}
