//! `Mesh`: a triangle mesh of f32 vertices and indexed faces. Ports the core of agg-sharp
//! `PolygonMesh/Mesh.cs` (the `Mesh` class: constructors, equality, winding, welding, hashing,
//! change tracking, transforms, bounds and face creation). The free extension methods of the
//! same file live in `mesh_extensions.rs`; measurements in `mesh_measure.rs`.
//!
//! `MergeVertices`, `GetVertexBvhTree`, `SplitFace` and `Split` are in `mesh_editing.rs`.
//!
//! Not yet ported (later Phase 2 slices, they need code that is not here yet): `FaceTextures`
//! and `FaceColors` with `RemapFaceData` (agg `ImageBuffer`/`Color`), the transformed-bounds
//! cache (convex hull), `PropertyBag` and the `Changed` event.

use std::collections::HashMap;
use std::fmt;
use std::sync::atomic::{AtomicI32, Ordering};
use std::sync::OnceLock;

use vector_math::{
    long_hash, AxisAlignedBoundingBox, Matrix4X4, Vector3, Vector3Float, LONG_HASH_SEED,
};

use crate::face::FaceList;
use crate::vector3_list;

/// The id counter behind [`Mesh::get_id`].
static NEXT_ID: AtomicI32 = AtomicI32::new(0);

/// A triangle mesh.
#[derive(Debug, Clone, Default)]
pub struct Mesh {
    pub vertices: Vec<Vector3Float>,
    pub faces: FaceList,
    changed_count: i32,
    cached_aabb: OnceLock<AxisAlignedBoundingBox>,
    long_hash_before_clean: OnceLock<u64>,
}

/// C# `int.GetLongHashCode(hash)`: FNV-1a over the int's 4 little-endian bytes.
pub(crate) fn int_long_hash(value: i32, hash: u64) -> u64 {
    long_hash::compute_hash(&value.to_le_bytes(), hash)
}

/// A dictionary key with C# `float.Equals` semantics: 0.0 and -0.0 alike, every NaN alike.
pub(crate) fn f32_key(value: f32) -> u32 {
    if value == 0.0 {
        0
    } else if value.is_nan() {
        f32::NAN.to_bits()
    } else {
        value.to_bits()
    }
}

/// A dictionary key with C# `double.Equals` semantics: 0.0 and -0.0 alike, every NaN alike.
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
        for c in v.chunks_exact(3) {
            mesh.vertices
                .push(Vector3Float::from_vector3(Vector3::new(c[0], c[1], c[2])));
        }
        for c in f.chunks_exact(3) {
            mesh.faces.add(c[0], c[1], c[2], &mesh.vertices);
        }
        mesh
    }

    /// A mesh from f64 vertices (narrowed) and a flat face index list (C# `Mesh(List<Vector3>,
    /// List<int>)`).
    pub fn from_vector3_list_and_indices(v: &[Vector3], f: &[i32]) -> Self {
        let mut mesh = Mesh::new();
        mesh.vertices = v.iter().map(|&p| Vector3Float::from_vector3(p)).collect();
        for c in f.chunks_exact(3) {
            mesh.faces.add(c[0], c[1], c[2], &mesh.vertices);
        }
        mesh
    }

    /// How many times [`Mesh::mark_as_changed`] has run (wraps, as C#'s unchecked increment).
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

        for face in self.faces.iter() {
            let iv0 = get_index(self.vertices[face.v0 as usize], &mut new_vertices);
            let iv1 = get_index(self.vertices[face.v1 as usize], &mut new_vertices);
            let iv2 = get_index(self.vertices[face.v2 as usize], &mut new_vertices);
            if iv0 != iv1 && iv1 != iv2 && iv2 != iv0 {
                new_faces.add(iv0, iv1, iv2, &new_vertices);
            }
        }

        if self.faces.len() != new_faces.len() || self.vertices.len() != new_vertices.len() {
            self.faces = new_faces;
            self.vertices = new_vertices;
            self.mark_as_changed();
        }
    }

    /// FNV-1a over the vertex and face counts, up to ~1000 sampled vertices and ~10000 sampled
    /// faces, starting from `hash` (C# `GetLongHashCode(ulong hash)`).
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

        hash
    }

    /// [`Mesh::get_long_hash_code_with`] from the default FNV seed.
    pub fn get_long_hash_code(&self) -> u64 {
        self.get_long_hash_code_with(LONG_HASH_SEED)
    }

    /// Invalidates the cached bounds and bumps [`Mesh::changed_count`].
    pub fn mark_as_changed(&mut self) {
        self.cached_aabb.take();
        // unchecked in C#: rolls over rather than throwing.
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

    /// Adds the distinct positions as new vertices and fans faces across them (C#
    /// `CreateFace(params Vector3[])`).
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

    /// Appends every face of `mesh`, transformed by `matrix`, as new welded-per-face geometry.
    pub fn copy_all_faces(&mut self, mesh: &Mesh, matrix: Matrix4X4) {
        for face in mesh.faces.iter() {
            let v0 = mesh.vertices[face.v0 as usize].transform(matrix);
            let v1 = mesh.vertices[face.v1 as usize].transform(matrix);
            let v2 = mesh.vertices[face.v2 as usize].transform(matrix);
            self.create_face_f32(&[v0, v1, v2]);
        }
    }
}

/// C# `Equals(Mesh)`: same vertices (exactly) and same face indices, in order. Normals and
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
