//! `Mesh`: a triangle mesh of f32 vertices and indexed faces, with its constructors, equality,
//! winding, welding, hashing, change tracking, transforms, bounds and face creation. The other
//! mesh helpers live in `mesh_extensions.rs`; measurements in `mesh_measure.rs`.
//!
//! Vertex merging, the vertex BVH and splitting faces and meshes are in `mesh_editing.rs`.
//!
//! The per-face colors and textures are fields here; remapping them, the centroid colors and
//! the texture helpers live in `face_data.rs`. A mesh has no property bag or change event:
//! caches and owners watch `changed_count`. The convex hull and transformed-bounds caches live
//! in `processors::mesh_convex_hull` and `transformed_aabb_cache`; they key on `changed_count`.

use std::collections::HashMap;
use std::fmt;
use std::sync::atomic::{AtomicI32, Ordering};
use std::sync::OnceLock;

use vector_math::Color;
use vector_math::{
    long_hash, AxisAlignedBoundingBox, Matrix4X4, Vector3, Vector3Float, LONG_HASH_SEED,
};

use crate::face::FaceList;
use crate::face_data::{FaceTextureData, FaceTextures};
use crate::processors::mesh_convex_hull::ConvexHullCache;
use crate::transformed_aabb_cache::TransformedAabbCache;
use crate::vector3_list;

/// The id counter behind [`Mesh::get_id`].
static NEXT_ID: AtomicI32 = AtomicI32::new(0);

/// A triangle mesh.
#[derive(Debug, Clone, Default)]
pub struct Mesh {
    pub vertices: Vec<Vector3Float>,
    pub faces: FaceList,
    /// Lookup by face index into the UVs and image for a face.
    pub face_textures: FaceTextures,
    /// Optional per-face colors. When `Some`, `face_colors[face_index]` gives the color for
    /// that face; `None` means no per-face coloring (the object's own color paints every face).
    /// Used to preserve per-mesh colors through booleans.
    pub face_colors: Option<Vec<Color>>,
    changed_count: i32,
    cached_aabb: OnceLock<AxisAlignedBoundingBox>,
    long_hash_before_clean: OnceLock<u64>,
    transformed_aabb_cache: TransformedAabbCache,
    pub(crate) convex_hull_cache: ConvexHullCache,
}

/// FNV-1a over the int's 4 little-endian bytes.
pub(crate) fn int_long_hash(value: i32, hash: u64) -> u64 {
    long_hash::compute_hash(&value.to_le_bytes(), hash)
}

/// FNV-1a over a picture's size and up to ~4096 of its bytes, evenly sampled, so hashing a large
/// texture stays cheap while two different pictures of one size still almost always differ.
fn image_long_hash(image: Option<&agg_path::image::ImageBuffer>, hash: u64) -> u64 {
    let Some(image) = image else {
        return int_long_hash(-1, hash);
    };
    let mut hash = int_long_hash(image.width(), hash);
    hash = int_long_hash(image.height(), hash);
    let bytes = image.get_buffer();
    let step = (bytes.len() / 4096).max(1);
    let sampled: Vec<u8> = bytes.iter().step_by(step).copied().collect();
    long_hash::compute_hash(&sampled, hash)
}

/// A dictionary key under which 0.0 and -0.0 are alike and every NaN is alike, so welding
/// treats equal positions as equal however they were computed.
pub(crate) fn f32_key(value: f32) -> u32 {
    if value == 0.0 {
        0
    } else if value.is_nan() {
        f32::NAN.to_bits()
    } else {
        value.to_bits()
    }
}

/// A dictionary key under which 0.0 and -0.0 are alike and every NaN is alike (the f64 form
/// of [`f32_key`]).
pub(crate) fn f64_key(value: f64) -> u64 {
    if value == 0.0 {
        0
    } else if value.is_nan() {
        f64::NAN.to_bits()
    } else {
        value.to_bits()
    }
}

impl Mesh {
    /// An empty mesh.
    pub fn new() -> Self {
        Self::default()
    }

    /// A mesh holding its own copies of the given f64 vertices (narrowed) and faces.
    pub fn from_vector3_list(v: &[Vector3], f: &FaceList) -> Self {
        Mesh {
            vertices: v.iter().map(|&p| Vector3Float::from_vector3(p)).collect(),
            faces: f.clone(),
            ..Self::default()
        }
    }

    /// A mesh holding its own copies of the given vertices and faces.
    ///
    /// The faces are copied: several mesh operations edit a face in place
    /// ([`Mesh::reverse_face`], [`Mesh::calculate_normals`]), so a mesh sharing the source's
    /// faces would not be independent of it.
    pub fn from_vertices_and_faces(v: &[Vector3Float], f: &FaceList) -> Self {
        Mesh {
            vertices: v.to_vec(),
            faces: f.clone(),
            ..Self::default()
        }
    }

    /// A mesh from a 3xN array of vertex coordinates and a 3xM array of face vertex indexes.
    pub fn from_arrays(v: &[f64], f: &[i32]) -> Self {
        let mut mesh = Mesh::new();
        for c in v.as_chunks::<3>().0 {
            mesh.vertices
                .push(Vector3Float::from_vector3(Vector3::new(c[0], c[1], c[2])));
        }
        for c in f.as_chunks::<3>().0 {
            mesh.faces.add(c[0], c[1], c[2], &mesh.vertices);
        }
        mesh
    }

    /// A mesh from f64 vertices (narrowed) and a flat face index list.
    pub fn from_vector3_list_and_indices(v: &[Vector3], f: &[i32]) -> Self {
        let mut mesh = Mesh::new();
        mesh.vertices = v.iter().map(|&p| Vector3Float::from_vector3(p)).collect();
        for c in f.as_chunks::<3>().0 {
            mesh.faces.add(c[0], c[1], c[2], &mesh.vertices);
        }
        mesh
    }

    /// How many times [`Mesh::mark_as_changed`] has run (wrapping; caches only compare it for
    /// equality).
    pub fn changed_count(&self) -> i32 {
        self.changed_count
    }

    /// The mesh's long hash the first time it was asked for; later edits do not change it.
    pub fn long_hash_before_clean(&self) -> u64 {
        *self
            .long_hash_before_clean
            .get_or_init(|| self.get_long_hash_code())
    }

    /// Reverses the winding of every face.
    pub fn reverse_faces(&mut self) {
        for i in 0..self.faces.len() {
            self.reverse_face(i, false);
        }
        self.mark_as_changed();
    }

    /// Reverses the winding of one face (swaps v0 and v2) and negates its normal.
    pub fn reverse_face(&mut self, face_index: usize, mark_as_change: bool) {
        let face = &mut self.faces[face_index];
        std::mem::swap(&mut face.v0, &mut face.v2);
        face.normal *= -1.0;

        // The UVs are per corner, so the winding flip has to take them along or the face's
        // texture comes out mirrored. Replaced rather than edited in place, because a
        // FaceTextureData can be shared with another face or mesh.
        if let Some(texture_data) = self.face_textures.get_mut(&(face_index as i32)) {
            *texture_data = FaceTextureData::new(
                texture_data.image.clone(),
                texture_data.uv2,
                texture_data.uv1,
                texture_data.uv0,
            );
        }

        if mark_as_change {
            self.mark_as_changed();
        }
    }

    /// [`Mesh::reverse_face`], marking the mesh changed.
    pub fn flip_face(&mut self, face_index: usize) {
        self.reverse_face(face_index, true);
    }

    /// Merges vertices that share the exact same coordinates.
    ///
    /// This is a repair pass for meshes that do not have topology yet - generated triangle
    /// soup, or unwelded seams coming back from a kernel readback. A mesh that is already a
    /// closed 2-manifold has nothing to gain from it: every positional merge either changes
    /// nothing or fuses two vertices the topology deliberately kept apart, which is what turns
    /// a valid solid into one with 4-faces-per-edge junctions. The boolean kernel returns
    /// exactly such a mesh - its coincident-but-distinct vertices are intentional
    /// epsilon-valid self-touching geometry (two solids meeting at a seam plane), not an
    /// error. So an already-manifold mesh is left alone.
    ///
    /// Note that this only bypasses the weld, it does not repair it: on a mesh that is both
    /// non-manifold AND self-touching the weld below still runs and is still non-idempotent,
    /// and can still drop faces.
    pub fn clean_and_merge(&mut self) {
        // A mesh with no faces has no edges, so it passes the manifold test vacuously - but it
        // is precisely the case that still needs this pass, because dropping every face is what
        // leaves vertices orphaned. Let it fall through and compact them away.
        if !self.faces.is_empty() && self.is_manifold() {
            return;
        }

        let mut new_vertices: Vec<Vector3Float> = Vec::new();
        let mut new_faces = FaceList::new();
        let mut source_face_index: Vec<i32> = Vec::with_capacity(self.faces.len());
        let mut position_to_index: HashMap<(u32, u32, u32), i32> = HashMap::new();
        let mut get_index = |position: Vector3Float, new_vertices: &mut Vec<Vector3Float>| {
            let key = (
                f32_key(position.x),
                f32_key(position.y),
                f32_key(position.z),
            );
            *position_to_index.entry(key).or_insert_with(|| {
                new_vertices.push(position);
                new_vertices.len() as i32 - 1
            })
        };

        for (face_idx, face) in self.faces.iter().enumerate() {
            let iv0 = get_index(self.vertices[face.v0 as usize], &mut new_vertices);
            let iv1 = get_index(self.vertices[face.v1 as usize], &mut new_vertices);
            let iv2 = get_index(self.vertices[face.v2 as usize], &mut new_vertices);
            if iv0 != iv1 && iv1 != iv2 && iv2 != iv0 {
                new_faces.add(iv0, iv1, iv2, &new_vertices);
                source_face_index.push(face_idx as i32);
            }
        }

        if self.faces.len() != new_faces.len() || self.vertices.len() != new_vertices.len() {
            self.faces = new_faces;
            self.vertices = new_vertices;

            // The welding only moves vertices onto each other, so the faces that survive keep
            // their geometry and their UVs come across unchanged.
            self.remap_face_data(&source_face_index, None);

            self.mark_as_changed();
        }
    }

    /// FNV-1a over the vertex and face counts, up to ~1000 sampled vertices and ~10000 sampled
    /// faces, the face colors and the face textures, starting from `hash`.
    pub fn get_long_hash_code_with(&self, hash: u64) -> u64 {
        let mut hash = int_long_hash(self.vertices.len() as i32, hash);
        hash = int_long_hash(self.faces.len() as i32, hash);

        // we want to at most consider 100000 vertices
        let vertex_step = (self.vertices.len() / 1000).max(1);
        for vertex in self.vertices.iter().step_by(vertex_step) {
            hash = vertex.get_long_hash_code_with(hash);
        }

        // we want to at most consider 100000 faces
        let face_step = (self.faces.len() / 10000).max(1);
        for face in self.faces.iter().step_by(face_step) {
            hash = int_long_hash(face.v0, hash);
            hash = int_long_hash(face.v1, hash);
            hash = int_long_hash(face.v2, hash);
        }

        // Include the face colors so meshes with different per-face colors produce different
        // hashes and aren't served from stale cache
        if let Some(face_colors) = &self.face_colors {
            hash = int_long_hash(face_colors.len() as i32, hash);
            let color_step = (face_colors.len() / 1000).max(1);
            for color in face_colors.iter().step_by(color_step) {
                hash = color.get_long_hash_code(hash);
            }
        }

        // Include the face textures (which faces, their UVs and their pictures) so a textured
        // mesh never shares a stored asset with the same shape untextured or wearing another
        // picture. A mesh without textures hashes as it always has.
        if !self.face_textures.is_empty() {
            hash = int_long_hash(self.face_textures.len() as i32, hash);
            let texture_step = (self.face_textures.len() / 1000).max(1);
            let mut last_image: Option<&std::sync::Arc<agg_path::image::ImageBuffer>> = None;
            for (&face, texture) in self.face_textures.iter().step_by(texture_step) {
                hash = int_long_hash(face, hash);
                for uv in [texture.uv0, texture.uv1, texture.uv2] {
                    hash = int_long_hash(uv.x.to_bits() as i32, hash);
                    hash = int_long_hash(uv.y.to_bits() as i32, hash);
                }
                let same_as_last = matches!(
                    (last_image, &texture.image),
                    (Some(a), Some(b)) if std::sync::Arc::ptr_eq(a, b)
                );
                if !same_as_last {
                    hash = image_long_hash(texture.image.as_deref(), hash);
                    last_image = texture.image.as_ref();
                }
            }
        }

        hash
    }

    /// [`Mesh::get_long_hash_code_with`] from the default FNV seed.
    pub fn get_long_hash_code(&self) -> u64 {
        self.get_long_hash_code_with(LONG_HASH_SEED)
    }

    /// Invalidates the cached bounds and bumps [`Mesh::changed_count`].
    pub fn mark_as_changed(&mut self) {
        self.transformed_aabb_cache.changed();
        self.cached_aabb.take();
        // rolls over rather than panicking: caches only compare it for equality.
        self.changed_count = self.changed_count.wrapping_add(1);
    }

    /// Transforms every vertex and recomputes every face normal (a no-op for the identity).
    pub fn transform(&mut self, matrix: Matrix4X4) {
        if matrix != Matrix4X4::IDENTITY {
            vector3_list::transform_f32(&mut self.vertices, matrix);
            let vertices = &self.vertices;
            for face in self.faces.iter_mut() {
                face.calculate_normal(vertices);
            }
            self.mark_as_changed();
        }
    }

    /// Recomputes every face normal from the vertex positions.
    pub fn calculate_normals(&mut self) {
        let vertices = &self.vertices;
        for face in self.faces.iter_mut() {
            face.calculate_normal(vertices);
        }
        self.mark_as_changed();
    }

    /// Moves every vertex by `(x, y, z)`.
    pub fn translate_xyz(&mut self, x: f64, y: f64, z: f64) {
        self.translate(Vector3::new(x, y, z));
    }

    /// Moves every vertex by `offset` (a no-op for zero).
    pub fn translate(&mut self, offset: Vector3) {
        if offset != Vector3::ZERO {
            vector3_list::transform_f32(
                &mut self.vertices,
                Matrix4X4::create_translation_vector(offset),
            );
            self.mark_as_changed();
        }
    }

    /// A process-wide increasing id.
    pub fn get_id() -> i32 {
        NEXT_ID.fetch_add(1, Ordering::Relaxed)
    }

    /// The bounds of the vertices (cached until the next [`Mesh::mark_as_changed`]); the zero
    /// box for a mesh with no vertices.
    pub fn get_axis_aligned_bounding_box(&self) -> AxisAlignedBoundingBox {
        if self.vertices.is_empty() {
            return AxisAlignedBoundingBox::zero();
        }
        *self
            .cached_aabb
            .get_or_init(|| vector3_list::bounds_f32(&self.vertices))
    }

    /// The bounds of the mesh under `transform`, from its convex hull where it has one (cached
    /// per transform until the next [`Mesh::mark_as_changed`]).
    pub fn get_axis_aligned_bounding_box_transformed(
        &self,
        transform: Matrix4X4,
    ) -> AxisAlignedBoundingBox {
        self.transformed_aabb_cache
            .get_axis_aligned_bounding_box(self, transform)
    }

    /// Adds the distinct positions as new vertices and fans faces across them.
    pub fn create_face(&mut self, positions_in: &[Vector3]) {
        let mut positions: Vec<Vector3> = Vec::with_capacity(positions_in.len());
        for &p in positions_in {
            if !positions.contains(&p) {
                positions.push(p);
            }
        }
        let floats: Vec<Vector3Float> = positions
            .into_iter()
            .map(Vector3Float::from_vector3)
            .collect();
        self.add_fan(&floats);
    }

    /// Adds the distinct f32 positions as new vertices and fans faces across them.
    pub fn create_face_f32(&mut self, positions_in: &[Vector3Float]) {
        let mut positions: Vec<Vector3Float> = Vec::with_capacity(positions_in.len());
        for &p in positions_in {
            if !positions.contains(&p) {
                positions.push(p);
            }
        }
        self.add_fan(&positions);
    }

    fn add_fan(&mut self, positions: &[Vector3Float]) {
        let first_vertex = self.vertices.len() as i32;
        self.vertices.extend_from_slice(positions);
        let added_positions = positions.len() as i32;
        for i in 0..(added_positions - 2).max(0) {
            self.faces.add(
                first_vertex,
                first_vertex + i + 1,
                first_vertex + i + 2,
                &self.vertices,
            );
        }
    }

    /// Adds a polygon over existing vertices, fanned from its first corner across its distinct
    /// corners. A builder that places each point once gives a collapsed side (a tip scaled to a
    /// point, a ring on the axis) one vertex, so a corner repeating an earlier one is dropped, and
    /// a polygon left with fewer than three distinct corners encloses nothing and adds no face.
    pub fn add_polygon(&mut self, corners: &[i32]) {
        let mut distinct: Vec<i32> = Vec::with_capacity(corners.len());
        for &corner in corners {
            if !distinct.contains(&corner) {
                distinct.push(corner);
            }
        }
        for k in 1..distinct.len().saturating_sub(1) {
            self.faces
                .add(distinct[0], distinct[k], distinct[k + 1], &self.vertices);
        }
    }

    /// Appends every face of `mesh`, transformed by `matrix`, as new welded-per-face geometry,
    /// with its textures and colors.
    ///
    /// Per-face color is all-or-nothing per mesh: `face_colors` is either `None` (the object's
    /// own color paints every face) or one entry per face. Merging a painted mesh with an
    /// unpainted one therefore has to invent entries for the unpainted side, and those get
    /// [`Mesh::UNCOLORED_FACE`] - leaving them at `Color::default()` made them transparent, and
    /// dropping the array made the painted side lose its colors. Two unpainted meshes stay
    /// unpainted so their objects' own colors keep doing the work.
    pub fn copy_all_faces(&mut self, mesh: &Mesh, matrix: Matrix4X4) {
        let f_start = self.faces.len();

        // create_face welds duplicate positions and then fans what is left, so a source face
        // does not reliably produce exactly one face here: a degenerate triangle produces none.
        // Record which source face actually produced each new face instead of assuming a flat
        // f_start + source_index offset.
        let mut source_face_of_new_face: Vec<usize> = Vec::with_capacity(mesh.faces.len());
        for (i, face) in mesh.faces.iter().enumerate() {
            let v0 = mesh.vertices[face.v0 as usize].transform(matrix);
            let v1 = mesh.vertices[face.v1 as usize].transform(matrix);
            let v2 = mesh.vertices[face.v2 as usize].transform(matrix);
            let before_create = self.faces.len();
            self.create_face_f32(&[v0, v1, v2]);
            for _ in before_create..self.faces.len() {
                source_face_of_new_face.push(i);
            }
        }

        // The matrix only moves the vertices around, it does not change which corner is which,
        // so the UVs come across as they are.
        if !mesh.face_textures.is_empty() {
            for (i, &source_face) in source_face_of_new_face.iter().enumerate() {
                if let Some(texture_data) = mesh.face_textures.get(&(source_face as i32)) {
                    self.face_textures
                        .insert((f_start + i) as i32, texture_data.clone());
                }
            }
        }

        if mesh.face_colors.is_none() && self.face_colors.is_none() {
            return;
        }

        let total_faces = self.faces.len();
        let mut colors = self.face_colors.take().unwrap_or_default();
        if colors.len() < total_faces {
            colors.resize(total_faces, Self::UNCOLORED_FACE);
        }

        if let Some(source_colors) = &mesh.face_colors {
            for (i, &source_face) in source_face_of_new_face.iter().enumerate() {
                if source_face < source_colors.len() {
                    colors[f_start + i] = source_colors[source_face];
                }
            }
        }

        self.face_colors = Some(colors);
    }
}

/// Same vertices (exactly) and same face indices, in order. Normals and
/// change counts are not compared.
impl PartialEq for Mesh {
    fn eq(&self, other: &Self) -> bool {
        self.vertices.len() == other.vertices.len()
            && self.faces.len() == other.faces.len()
            && self.vertices == other.vertices
            && self
                .faces
                .iter()
                .zip(other.faces.iter())
                .all(|(a, b)| a.v0 == b.v0 && a.v1 == b.v1 && a.v2 == b.v2)
    }
}

impl fmt::Display for Mesh {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "Faces = {}, Vertices = {}",
            self.faces.len(),
            self.vertices.len()
        )
    }
}
