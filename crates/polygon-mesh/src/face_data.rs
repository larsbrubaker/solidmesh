//! Per-face colors and textures on a [`Mesh`]: [`FaceTextureData`], remapping face data when the
//! face list is rebuilt, interpolating a texture onto split faces, the uncolored and unknown face
//! colors, saving face colors by centroid and restoring them by nearest centroid, saving face
//! textures with their corner positions and restoring them onto faces with the same corners, and
//! placing and
//! removing textures with an explicit mapping, plus the projection picking that stretches an image
//! over a face's plane.
//!
//! The fields themselves (`Mesh::face_colors`, `Mesh::face_textures`) live on `Mesh` in
//! `mesh.rs`; the operations that rebuild the face list (`mesh.rs`, `mesh_editing.rs`,
//! `mesh_extensions.rs`, `t_junctions.rs`) call [`Mesh::remap_face_data`] so the data follows
//! the faces.

use std::collections::{BTreeMap, HashMap};
use std::sync::Arc;

use agg_path::image::ImageBuffer;
use agg_path::{Color, RectangleDouble};
use vector_math::{Matrix4X4, Quaternion, Vector2, Vector2Float, Vector3, Vector3Float};

use crate::face::FaceList;
use crate::mesh::{f32_key, Mesh};

/// The texture on one face: the image and the UV of each corner.
///
/// The image is shared (`Arc`) rather than copied per face; two entries carry "the same texture"
/// when their images are the same allocation ([`FaceTextureData::same_image`]).
#[derive(Debug, Clone)]
pub struct FaceTextureData {
    pub image: Option<Arc<ImageBuffer>>,
    pub uv0: Vector2Float,
    pub uv1: Vector2Float,
    pub uv2: Vector2Float,
}

impl FaceTextureData {
    pub fn new(
        texture_to_use: Option<Arc<ImageBuffer>>,
        uv0: Vector2Float,
        uv1: Vector2Float,
        uv2: Vector2Float,
    ) -> Self {
        Self {
            image: texture_to_use,
            uv0,
            uv1,
            uv2,
        }
    }

    /// Whether the `image` fields are the same allocation (both absent counts as the same).
    pub fn same_image(&self, texture: Option<&Arc<ImageBuffer>>) -> bool {
        match (&self.image, texture) {
            (Some(a), Some(b)) => Arc::ptr_eq(a, b),
            (None, None) => true,
            _ => false,
        }
    }
}

/// Lookup by face index into the UVs and image for a face, ordered by face index: a deterministic
/// order, the one any writer of per-face data wants.
pub type FaceTextures = BTreeMap<i32, FaceTextureData>;

/// One saved face color with the face's centroid.
pub type CentroidColor = (Vector3, Color);

/// One saved face texture with the positions of the face's three corners, in UV order.
pub type CornerTexture = ([Vector3Float; 3], FaceTextureData);

/// A face's corner positions as an order-free key: each position's exact bits, sorted, so a face
/// found again with its corners rotated or its winding flipped still matches.
fn corner_key(corners: [Vector3Float; 3]) -> [[u32; 3]; 3] {
    let mut key = corners.map(|p| [f32_key(p.x), f32_key(p.y), f32_key(p.z)]);
    key.sort_unstable();
    key
}

impl Mesh {
    /// The color a face gets when a mesh with no per-face colors is merged into one that has
    /// them.
    ///
    /// White because the renderer uses face colors as the vertex color against a white
    /// material, so a white face is an unpainted surface. It is not `Color::default()`, which
    /// is rgba(0, 0, 0, 0) and draws as nothing - faces left at it are invisible rather than
    /// merely uncolored.
    pub const UNCOLORED_FACE: Color = Color::WHITE;

    /// The placeholder a face gets when its colour is asked for and the data that should have
    /// held it is malformed: a face-color array (or a saved centroid list) shorter than the face
    /// list it describes.
    ///
    /// It is not the answer to "this face's colour could not be worked out". A boolean result
    /// whose triangles cannot be traced back to an operand comes back with no face colors at
    /// all, so the object's own colour shows; painting this grey there reads as the part having
    /// turned grey, because it is a colour nothing in the scene is wearing.
    pub const UNKNOWN_FACE_COLOR: Color = Color::rgba(200, 200, 200, 255);

    /// Moves the per-face data (colors and textures) onto a face list that has just been rebuilt.
    ///
    /// `source_face_index_per_new_face` gives, for each face in the new list, the index it had
    /// before the rebuild, or -1 for a face with no source face. Pass `sources` (the face list
    /// as it was and the vertices it indexes) when faces were cut into pieces rather than merely
    /// dropped or reordered: the UVs are then re-derived for each new face from the barycentric
    /// position of its corners inside its source triangle, so a piece does not carry its
    /// parent's corner UVs onto vertices the parent never had.
    pub(crate) fn remap_face_data(
        &mut self,
        source_face_index_per_new_face: &[i32],
        sources: Option<(&FaceList, &[Vector3Float])>,
    ) {
        if let Some(old_colors) = &self.face_colors {
            let new_colors = source_face_index_per_new_face
                .iter()
                .map(|&source| {
                    if source >= 0 && (source as usize) < old_colors.len() {
                        old_colors[source as usize]
                    } else {
                        Self::UNCOLORED_FACE
                    }
                })
                .collect();
            self.face_colors = Some(new_colors);
        }

        if !self.face_textures.is_empty() {
            let old_textures = std::mem::take(&mut self.face_textures);
            let mut new_textures = FaceTextures::new();
            for (i, &source) in source_face_index_per_new_face.iter().enumerate() {
                if source < 0 {
                    continue;
                }
                let Some(texture_data) = old_textures.get(&source) else {
                    continue;
                };
                let entry = match sources {
                    Some((source_faces, source_vertices)) => {
                        let source_face = source_faces[source as usize];
                        let new_face = self.faces[i];
                        let sv = |v: i32| source_vertices[v as usize];
                        let nv = |v: i32| self.vertices[v as usize];
                        interpolate_face_texture(
                            texture_data,
                            [sv(source_face.v0), sv(source_face.v1), sv(source_face.v2)],
                            [nv(new_face.v0), nv(new_face.v1), nv(new_face.v2)],
                        )
                    }
                    None => texture_data.clone(),
                };
                new_textures.insert(i as i32, entry);
            }
            self.face_textures = new_textures;
        }
    }

    /// Each face's centroid with its color, or `None` when the mesh has no face colors. Used to
    /// save color data before an operation that loses it.
    pub fn save_face_centroid_colors(&self) -> Option<Vec<CentroidColor>> {
        let face_colors = self.face_colors.as_ref()?;
        let result = self
            .faces
            .iter()
            .enumerate()
            .map(|(i, face)| {
                let color = face_colors
                    .get(i)
                    .copied()
                    .unwrap_or(Self::UNKNOWN_FACE_COLOR);
                (self.face_centroid(face.v0, face.v1, face.v2), color)
            })
            .collect();
        Some(result)
    }

    /// The color of the saved centroid nearest `point`; the first of equally near ones.
    /// [`Mesh::UNKNOWN_FACE_COLOR`] for an empty list.
    pub fn find_nearest_centroid_color(point: Vector3, saved_colors: &[CentroidColor]) -> Color {
        let mut best_dist_sq = f64::MAX;
        // Only survives an empty saved_colors, the same malformed-data case
        // save_face_centroid_colors falls back on.
        let mut best_color = Self::UNKNOWN_FACE_COLOR;
        for &(centroid, color) in saved_colors {
            let dist_sq = (point - centroid).length_squared();
            if dist_sq < best_dist_sq {
                best_dist_sq = dist_sq;
                best_color = color;
            }
        }
        best_color
    }

    /// Restores per-face colors by matching each face's centroid to the nearest saved centroid.
    /// Does nothing for `None`.
    pub fn restore_face_colors_from_centroids(&mut self, saved_colors: Option<&[CentroidColor]>) {
        let Some(saved_colors) = saved_colors else {
            return;
        };
        let face_colors = self
            .faces
            .iter()
            .map(|face| {
                let centroid = self.face_centroid(face.v0, face.v1, face.v2);
                Self::find_nearest_centroid_color(centroid, saved_colors)
            })
            .collect();
        self.face_colors = Some(face_colors);
    }

    /// Each textured face's texture with its corner positions, to put back with
    /// [`Mesh::restore_face_textures_from_corners`] after an operation that renumbers faces.
    pub fn save_face_corner_textures(&self) -> Vec<CornerTexture> {
        self.face_textures
            .iter()
            .filter_map(|(&face, texture)| {
                let face = self.faces.get(usize::try_from(face).ok()?)?;
                let corner = |v: i32| self.vertices.get(usize::try_from(v).ok()?).copied();
                let corners = [corner(face.v0)?, corner(face.v1)?, corner(face.v2)?];
                Some((corners, texture.clone()))
            })
            .collect()
    }

    /// Gives every face whose three corners sit exactly where a saved face's did that face's
    /// texture, each corner taking the UV of the saved corner at its position, so a face that
    /// came back rotated or turned over still shows the picture the same way. Faces matching no
    /// saved face (a cap over a hole) stay untextured. Exact positions, not nearest ones: a face
    /// that moved is a different face, and stretching a picture onto it would be a guess.
    pub fn restore_face_textures_from_corners(&mut self, saved: &[CornerTexture]) {
        if saved.is_empty() {
            return;
        }
        // Only looked up, never iterated; the first saved face wins a shared key.
        let mut by_corners: HashMap<[[u32; 3]; 3], usize> = HashMap::new();
        for (index, (corners, _)) in saved.iter().enumerate() {
            by_corners.entry(corner_key(*corners)).or_insert(index);
        }
        let mut restored = FaceTextures::new();
        for (face_index, face) in self.faces.iter().enumerate() {
            let corners = [face.v0, face.v1, face.v2].map(|v| self.vertices[v as usize]);
            let Some(&index) = by_corners.get(&corner_key(corners)) else {
                continue;
            };
            let (saved_corners, texture) = &saved[index];
            let saved_uvs = [texture.uv0, texture.uv1, texture.uv2];
            let uv_at = |p: Vector3Float| {
                let key = [f32_key(p.x), f32_key(p.y), f32_key(p.z)];
                saved_corners
                    .iter()
                    .position(|c| [f32_key(c.x), f32_key(c.y), f32_key(c.z)] == key)
                    .map_or(saved_uvs[0], |i| saved_uvs[i])
            };
            restored.insert(
                face_index as i32,
                FaceTextureData::new(
                    texture.image.clone(),
                    uv_at(corners[0]),
                    uv_at(corners[1]),
                    uv_at(corners[2]),
                ),
            );
        }
        self.face_textures = restored;
    }

    /// The centroid of a face, summed and divided in f32, then widened, so saved and restored
    /// centroids compare exactly.
    pub(crate) fn face_centroid(&self, v0: i32, v1: i32, v2: i32) -> Vector3 {
        let v = |i: i32| self.vertices[i as usize];
        Vector3::from_vector3_float((v(v0) + v(v1) + v(v2)) / 3.0)
    }

    /// Textures face `face` with `texture_to_use`, each corner's UV the x and y of its position
    /// under `texture_coordinate_mapping`.
    pub fn place_texture_on_face(
        &mut self,
        face: usize,
        texture_to_use: Option<Arc<ImageBuffer>>,
        texture_coordinate_mapping: Matrix4X4,
        mark_as_change: bool,
    ) {
        let uvs = self.face_uvs(face, texture_coordinate_mapping);
        self.face_textures.insert(
            face as i32,
            FaceTextureData::new(texture_to_use, uvs[0], uvs[1], uvs[2]),
        );
        if mark_as_change {
            self.mark_as_changed();
        }
    }

    /// Planar-projects `texture_to_use` along the normal of face `face`, stretched to cover that
    /// one face ([`Mesh::get_max_plane_projection`] over it alone).
    pub fn place_texture_on_face_projected(
        &mut self,
        face: usize,
        texture_to_use: Option<Arc<ImageBuffer>>,
    ) {
        //// planar project along the normal of this face
        let mapping = self.get_max_plane_projection(&[face], None);
        self.place_texture_on_face(face, texture_to_use, mapping, true);
    }

    /// Textures every face in `faces` as [`Mesh::place_texture_on_face`] does, except that a face
    /// already textured keeps its existing texture (the existing entries are added in last).
    pub fn place_texture_on_faces(
        &mut self,
        faces: &[usize],
        texture_to_use: Option<Arc<ImageBuffer>>,
        texture_coordinate_mapping: Matrix4X4,
    ) {
        let mut face_textures = FaceTextures::new();
        for &face in faces {
            let uvs = self.face_uvs(face, texture_coordinate_mapping);
            face_textures.insert(
                face as i32,
                FaceTextureData::new(texture_to_use.clone(), uvs[0], uvs[1], uvs[2]),
            );
        }
        // add in any existing face textures
        for (key, value) in std::mem::take(&mut self.face_textures) {
            face_textures.insert(key, value);
        }
        self.face_textures = face_textures;
        self.mark_as_changed();
    }

    /// Textures every face with the same mapping.
    pub fn place_texture(
        &mut self,
        texture_to_use: Option<Arc<ImageBuffer>>,
        texture_coordinate_mapping: Matrix4X4,
    ) {
        for i in 0..self.faces.len() {
            self.place_texture_on_face(
                i,
                texture_to_use.clone(),
                texture_coordinate_mapping,
                false,
            );
        }
        self.mark_as_changed();
    }

    /// Removes `texture` from every face that carries it.
    pub fn remove_texture(&mut self, texture: Option<&Arc<ImageBuffer>>) {
        for i in 0..self.faces.len() {
            self.remove_texture_from_face(i, texture);
        }
    }

    /// Removes `texture` from face `face_index` if that face carries it.
    pub fn remove_texture_from_face(
        &mut self,
        face_index: usize,
        texture: Option<&Arc<ImageBuffer>>,
    ) {
        let key = face_index as i32;
        if self
            .face_textures
            .get(&key)
            .is_some_and(|data| data.same_image(texture))
        {
            self.face_textures.remove(&key);
            self.mark_as_changed();
        }
    }

    /// The mapping that rotates the first face's normal onto +Z and then stretches the faces'
    /// rotated bounds to the unit square, so a texture placed with it covers exactly those faces.
    /// The image does not affect the mapping, so it is not a parameter.
    pub fn get_max_plane_projection(
        &self,
        faces: &[usize],
        initial_transform: Option<Matrix4X4>,
    ) -> Matrix4X4 {
        // If not set than make it identity
        let first_transform = initial_transform.unwrap_or(Matrix4X4::IDENTITY);

        let texture_coordinate_mapping = Matrix4X4::create_rotation_quaternion(
            Quaternion::from_directions(self.faces[faces[0]].normal.as_vector3(), Vector3::UNIT_Z),
        );

        let mut bounds = RectangleDouble::ZERO_INTERSECTION;

        for &face in faces {
            let f = self.faces[face];
            for vertex_index in [f.v0, f.v1, f.v2] {
                let edge_start_position = self.vertices[vertex_index as usize];
                let texture_uv = edge_start_position.transform(texture_coordinate_mapping);
                bounds.expand_to_include(Vector2::from_vector3_float(texture_uv));
            }
        }

        let centering = Matrix4X4::create_translation(-bounds.left, -bounds.bottom, 0.0);
        let scaling = Matrix4X4::create_scale(1.0 / bounds.width(), 1.0 / bounds.height(), 1.0);

        texture_coordinate_mapping * first_transform * centering * scaling
    }

    /// Planar-projects `texture_to_use` along the normal of face `face` over every face coplanar
    /// with it, stretched to cover them ([`Mesh::get_max_plane_projection`]).
    pub fn place_texture_on_coplanar_faces(
        &mut self,
        face: usize,
        texture_to_use: Option<Arc<ImageBuffer>>,
    ) {
        //// planar project along the normal of this face
        let faces = self.get_coplanar_faces_of_face(face);
        if !faces.is_empty() {
            let mapping = self.get_max_plane_projection(&faces, None);
            self.place_texture_on_faces(&faces, texture_to_use, mapping);
        }
    }

    fn face_uvs(&self, face: usize, mapping: Matrix4X4) -> [Vector2Float; 3] {
        let f = self.faces[face];
        [f.v0, f.v1, f.v2].map(|vertex_index| {
            let texture_uv = self.vertices[vertex_index as usize].transform(mapping);
            Vector2Float::from_vector3_float(texture_uv)
        })
    }
}

/// The texture data a triangle cut out of a source triangle should carry: the same image, with
/// each corner's UV blended from the source UVs by that corner's barycentric coordinates in the
/// source triangle. A degenerate (or sliver thin) source triangle has no usable barycentric
/// coordinates, so its data comes across unchanged rather than as NaNs or wild extrapolations.
fn interpolate_face_texture(
    source: &FaceTextureData,
    s: [Vector3Float; 3],
    p: [Vector3Float; 3],
) -> FaceTextureData {
    let s0 = s[0];
    let edge1 = Vector3::from_vector3_float(s[1] - s0);
    let edge2 = Vector3::from_vector3_float(s[2] - s0);
    let d00 = edge1.dot(edge1);
    let d01 = edge1.dot(edge2);
    let d11 = edge2.dot(edge2);
    let denominator = (d00 * d11) - (d01 * d01);
    // The denominator is the squared area term of the source triangle (|e1 x e2|^2), so on a
    // sliver it is a catastrophic cancellation of two nearly equal products. Test it relative to
    // the edge lengths instead: the ratio is sin(theta)^2 between the edges, so this rejects
    // anything thinner than about a millionth of a radian, and !(>) also rejects a NaN.
    #[allow(clippy::neg_cmp_op_on_partial_ord)]
    if !(denominator > (d00 * d11) * 1e-12) {
        return source.clone();
    }

    let uv_at = |position: Vector3Float| -> Vector2Float {
        let to_point = Vector3::from_vector3_float(position - s0);
        let d20 = to_point.dot(edge1);
        let d21 = to_point.dot(edge2);
        let v = ((d11 * d20) - (d01 * d21)) / denominator;
        let w = ((d00 * d21) - (d01 * d20)) / denominator;
        let u = 1.0 - v - w;
        (source.uv0 * u as f32) + (source.uv1 * v as f32) + (source.uv2 * w as f32)
    };

    FaceTextureData::new(source.image.clone(), uv_at(p[0]), uv_at(p[1]), uv_at(p[2]))
}
