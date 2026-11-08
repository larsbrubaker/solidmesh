//! Texturing faces that came without a texture from the textured faces around them: the caps a
//! repair adds over a hole ([`Mesh::texture_new_faces_from_neighbors`], after
//! `face_data.rs`'s corner restore has put back the faces that did not move) and, for a part sent
//! somewhere that needs a picture on every face, every untextured face
//! ([`Mesh::texture_every_face`]).
//!
//! A face is filled corner by corner: each corner takes the UV a textured face uses at that same
//! vertex index, so the picture runs on across the seam without a jump. The face takes the
//! picture most of its corners already carry; a corner with no UV on that picture takes the mean
//! of the corners that have one. Faces filled this way count as neighbours for the faces after
//! them, so a cap of several triangles fills from its rim inward.

use std::sync::Arc;

use solidmesh_math::texture::TexturePixels;
use solidmesh_math::{Vector2Float, Vector3Float};

use crate::face_data::{corner_key, FaceTextureData};
use crate::mesh::Mesh;

/// A face's picture: the image allocation (`None` is a texture entry with no image).
type Picture = Option<Arc<dyn TexturePixels>>;

/// Whether two pictures are the same allocation (both absent counts as the same).
fn same_picture(a: &Picture, b: &Picture) -> bool {
    match (a, b) {
        (Some(a), Some(b)) => Arc::ptr_eq(a, b),
        (None, None) => true,
        _ => false,
    }
}

/// For each vertex, the UV each picture's textured faces give it: one entry per picture, taken
/// from the lowest-indexed face with that picture at the vertex. Lowest index because a vertex on
/// a UV seam has several UVs on one picture, and the choice must not depend on which faces happen
/// to be filled first.
struct CornerUvs {
    by_vertex: Vec<Vec<(Picture, Vector2Float)>>,
}

impl CornerUvs {
    fn of(mesh: &Mesh) -> Self {
        let mut uvs = Self {
            by_vertex: vec![Vec::new(); mesh.vertices.len()],
        };
        // `face_textures` iterates in face-index order, so the first entry kept is the lowest.
        for (&face_index, texture) in &mesh.face_textures {
            let Some(face) = usize::try_from(face_index)
                .ok()
                .and_then(|i| mesh.faces.get(i))
            else {
                continue;
            };
            for (vertex, uv) in [
                (face.v0, texture.uv0),
                (face.v1, texture.uv1),
                (face.v2, texture.uv2),
            ] {
                uvs.add(vertex, &texture.image, uv);
            }
        }
        uvs
    }

    /// Records `uv` for `picture` at `vertex` unless that picture already has one there.
    fn add(&mut self, vertex: i32, picture: &Picture, uv: Vector2Float) {
        let Some(list) = usize::try_from(vertex)
            .ok()
            .and_then(|v| self.by_vertex.get_mut(v))
        else {
            return;
        };
        if !list.iter().any(|(p, _)| same_picture(p, picture)) {
            list.push((picture.clone(), uv));
        }
    }

    fn at(&self, vertex: i32) -> &[(Picture, Vector2Float)] {
        usize::try_from(vertex)
            .ok()
            .and_then(|v| self.by_vertex.get(v))
            .map_or(&[], Vec::as_slice)
    }
}

/// The texture a face gets from the UVs offered at its three corners (`offers[k]` for corner
/// `k`), or `None` when no corner is offered any. The picture is the one offered at the most
/// corners; a tie goes to the first picture met walking the corners in order, so the result never
/// depends on hashing. A corner offered nothing on that picture takes the mean of the corners
/// that were, summed in corner order.
fn texture_from_offers(offers: [&[(Picture, Vector2Float)]; 3]) -> Option<FaceTextureData> {
    let uv_on = |offer: &[(Picture, Vector2Float)], picture: &Picture| {
        offer
            .iter()
            .find(|(p, _)| same_picture(p, picture))
            .map(|(_, uv)| *uv)
    };
    let mut best: Option<(&Picture, usize)> = None;
    for offer in offers {
        for (picture, _) in offer {
            let count = offers
                .iter()
                .filter(|o| uv_on(o, picture).is_some())
                .count();
            if best.is_none_or(|(_, most)| count > most) {
                best = Some((picture, count));
            }
        }
    }
    let (picture, _) = best?;
    let known = offers.map(|offer| uv_on(offer, picture));
    let mut sum = Vector2Float::ZERO;
    let mut count = 0.0f32;
    for uv in known.iter().flatten() {
        sum += *uv;
        count += 1.0;
    }
    let mean = sum / count;
    let [uv0, uv1, uv2] = known.map(|uv| uv.unwrap_or(mean));
    Some(FaceTextureData::new(picture.clone(), uv0, uv1, uv2))
}

impl Mesh {
    /// The corner positions of every untextured face, when any face is textured (an empty list
    /// otherwise, as there is nothing to fill from). Saved beside
    /// [`Mesh::save_face_corner_textures`] before an operation that renumbers faces, so
    /// [`Mesh::texture_new_faces_from_neighbors`] can tell a face that was left bare on purpose
    /// from one the operation added.
    pub fn save_untextured_face_corners(&self) -> Vec<[Vector3Float; 3]> {
        if self.face_textures.is_empty() {
            return Vec::new();
        }
        self.faces
            .iter()
            .enumerate()
            .filter(|(index, _)| !self.face_textures.contains_key(&(*index as i32)))
            .filter_map(|(_, face)| {
                let corner = |v: i32| self.vertices.get(usize::try_from(v).ok()?).copied();
                Some([corner(face.v0)?, corner(face.v1)?, corner(face.v2)?])
            })
            .collect()
    }

    /// After [`Mesh::restore_face_textures_from_corners`]: textures every face that matches no
    /// face of the mesh as it was (a cap over a hole, or a face whose corner a weld moved) from
    /// the textured faces sharing its corners. Faces that were untextured before
    /// (`untextured_before`, from [`Mesh::save_untextured_face_corners`]) stay untextured, and so
    /// does a new face no textured face reaches through shared corners.
    pub fn texture_new_faces_from_neighbors(&mut self, untextured_before: &[[Vector3Float; 3]]) {
        if self.face_textures.is_empty() {
            return;
        }
        let bare: std::collections::HashSet<[[u32; 3]; 3]> =
            untextured_before.iter().map(|c| corner_key(*c)).collect();
        let candidates: Vec<usize> = (0..self.faces.len())
            .filter(|&index| !self.face_textures.contains_key(&(index as i32)))
            .filter(|&index| {
                let face = self.faces[index];
                let corners = [face.v0, face.v1, face.v2].map(|v| self.vertices[v as usize]);
                !bare.contains(&corner_key(corners))
            })
            .collect();
        self.texture_faces_from_neighbors(&candidates);
    }

    /// Textures each untextured face in `faces` (taken in index order) from the textured faces
    /// sharing its corners, repeating until a pass fills nothing more, so a run of new faces
    /// fills inward from the faces that touch the textured ones. Returns how many faces were
    /// textured.
    pub fn texture_faces_from_neighbors(&mut self, faces: &[usize]) -> usize {
        if self.face_textures.is_empty() {
            return 0;
        }
        let mut corner_uvs = CornerUvs::of(self);
        let mut pending: Vec<usize> = faces
            .iter()
            .copied()
            .filter(|&f| f < self.faces.len() && !self.face_textures.contains_key(&(f as i32)))
            .collect();
        pending.sort_unstable();
        pending.dedup();
        let mut filled = 0;
        loop {
            let before = pending.len();
            pending.retain(|&index| {
                let face = self.faces[index];
                let corners = [face.v0, face.v1, face.v2];
                let offers = corners.map(|v| corner_uvs.at(v));
                let Some(texture) = texture_from_offers(offers) else {
                    return true;
                };
                for (vertex, uv) in corners
                    .into_iter()
                    .zip([texture.uv0, texture.uv1, texture.uv2])
                {
                    corner_uvs.add(vertex, &texture.image, uv);
                }
                self.face_textures.insert(index as i32, texture);
                filled += 1;
                false
            });
            if pending.is_empty() || pending.len() == before {
                break;
            }
        }
        if filled > 0 {
            self.mark_as_changed();
        }
        filled
    }

    /// Gives every face a texture when any face has one: first from the textured faces sharing
    /// its corners ([`Mesh::texture_faces_from_neighbors`]), then, for faces no textured face
    /// reaches that way (a separate piece), each corner the UV of the nearest vertex a textured
    /// face uses, on that vertex's picture. For sending a part to a service that needs a picture
    /// on every face; the faces that had a texture keep it unchanged.
    pub fn texture_every_face(&mut self) {
        if self.face_textures.is_empty() {
            return;
        }
        let all: Vec<usize> = (0..self.faces.len()).collect();
        self.texture_faces_from_neighbors(&all);
        if self.face_textures.len() == self.faces.len() {
            return;
        }
        let corner_uvs = CornerUvs::of(self);
        let nearest = NearestTexturedVertex::new(&self.vertices, &corner_uvs);
        let mut filled = false;
        for index in 0..self.faces.len() {
            if self.face_textures.contains_key(&(index as i32)) {
                continue;
            }
            let face = self.faces[index];
            let offers = [face.v0, face.v1, face.v2].map(|v| {
                let position = self.vertices[v as usize];
                nearest
                    .find(position)
                    .map_or(&[][..], |found| &corner_uvs.at(found)[..1])
            });
            if let Some(texture) = texture_from_offers(offers) {
                self.face_textures.insert(index as i32, texture);
                filled = true;
            }
        }
        if filled {
            self.mark_as_changed();
        }
    }
}

/// The vertices some textured face uses, sorted by x, for finding the one nearest a point.
struct NearestTexturedVertex<'a> {
    vertices: &'a [Vector3Float],
    /// (x, vertex index), sorted by x then index.
    by_x: Vec<(f32, i32)>,
}

impl<'a> NearestTexturedVertex<'a> {
    fn new(vertices: &'a [Vector3Float], corner_uvs: &CornerUvs) -> Self {
        let mut by_x: Vec<(f32, i32)> = corner_uvs
            .by_vertex
            .iter()
            .enumerate()
            .filter(|(_, offers)| !offers.is_empty())
            // A mesh has far fewer than i32::MAX vertices.
            .map(|(v, _)| (vertices[v].x, v as i32))
            .collect();
        by_x.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
        Self { vertices, by_x }
    }

    /// The textured vertex nearest `point`; of equally near ones the lowest index, so the answer
    /// does not depend on which side of the sorted list the search reaches first.
    fn find(&self, point: Vector3Float) -> Option<i32> {
        let start = self.by_x.partition_point(|(x, _)| *x < point.x);
        let mut best: Option<(f64, i32)> = None;
        let distance_sq = |v: i32| {
            let p = self.vertices[v as usize];
            let (dx, dy, dz) = (
                f64::from(p.x) - f64::from(point.x),
                f64::from(p.y) - f64::from(point.y),
                f64::from(p.z) - f64::from(point.z),
            );
            (dx * dx) + (dy * dy) + (dz * dz)
        };
        let consider = |v: i32, best: &mut Option<(f64, i32)>| {
            let d = distance_sq(v);
            if best.is_none_or(|(bd, bv)| d < bd || (d == bd && v < bv)) {
                *best = Some((d, v));
            }
        };
        // Walk outward on both sides while the x gap alone is no farther than the best so far.
        let within = |x: f32, best: &Option<(f64, i32)>| {
            let dx = f64::from(x) - f64::from(point.x);
            best.is_none_or(|(bd, _)| dx * dx <= bd)
        };
        for &(x, v) in &self.by_x[start..] {
            if !within(x, &best) {
                break;
            }
            consider(v, &mut best);
        }
        for &(x, v) in self.by_x[..start].iter().rev() {
            if !within(x, &best) {
                break;
            }
            consider(v, &mut best);
        }
        best.map(|(_, v)| v)
    }
}
