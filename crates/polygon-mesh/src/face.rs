//! `Face` and `FaceList`: a triangle's three vertex indices plus its normal, and the list of them a
//! [`crate::Mesh`] holds.
//!
//! A `Face` is a plain value and the list owns its faces, so every copy of a mesh is independent of
//! the original (see `mesh_copy_independence_tests`).

use std::ops::{Deref, DerefMut};

use vector_math::Vector3Float;

/// One triangle: indices into the mesh's vertex list and the face normal.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Face {
    pub normal: Vector3Float,
    pub v0: i32,
    pub v1: i32,
    pub v2: i32,
}

impl Face {
    /// A face whose normal is computed from `vertices`.
    pub fn new(v0: i32, v1: i32, v2: i32, vertices: &[Vector3Float]) -> Self {
        let mut face = Face {
            normal: Vector3Float::ZERO,
            v0,
            v1,
            v2,
        };
        face.calculate_normal(vertices);
        face
    }

    /// A face with the given normal.
    pub fn with_normal(v0: i32, v1: i32, v2: i32, normal: Vector3Float) -> Self {
        Face { normal, v0, v1, v2 }
    }

    /// Recomputes this face's normal from its three vertex positions.
    ///
    /// A face with no area gets the zero vector rather than a normalized one. Zero area faces
    /// are topologically valid - they occur at the seams where a solid touches itself, carrying
    /// real edges that hold the mesh manifold - so they cannot simply be dropped. But their
    /// cross product is zero length, and normalizing that divides by zero: nothing downstream
    /// in PolygonMesh guards against a NaN normal, so one such face poisons every
    /// normal-tolerance vertex merge and coplanar-face walk it takes part in (NaN compares
    /// false against every tolerance).
    pub fn calculate_normal(&mut self, vertices: &[Vector3Float]) {
        let position0 = vertices[self.v0 as usize];
        let position1 = vertices[self.v1 as usize];
        let position2 = vertices[self.v2 as usize];
        let v11_minus_v0 = position1 - position0;
        let v2_minus_v0 = position2 - position0;
        let cross = v11_minus_v0.cross(v2_minus_v0);
        let length_squared = cross.length_squared();

        // The NaN test catches a non-finite input position as well as an overflowed cross
        // product - both would otherwise pass a plain "> 0" check.
        self.normal =
            if length_squared > 0.0 && !length_squared.is_nan() && !length_squared.is_infinite() {
                cross.get_normal()
            } else {
                Vector3Float::ZERO
            };
    }

    /// The same triangle wound the other way, normal negated.
    pub fn flipped(&self) -> Face {
        Face::with_normal(self.v0, self.v2, self.v1, -self.normal)
    }
}

/// The faces of a mesh; derefs to `Vec<Face>`.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct FaceList(Vec<Face>);

impl FaceList {
    /// An empty list.
    pub fn new() -> Self {
        Self::default()
    }

    /// Faces from a flat `v0, v1, v2, ...` index list, normals computed from `vertices`.
    pub fn from_int_array(f: &[i32], vertices: &[Vector3Float]) -> Self {
        let mut list = FaceList::new();
        list.add_from_int_array(f, vertices);
        list
    }

    /// Adds a face from vertex indexes and calculates its normal from the vertex positions.
    pub fn add(&mut self, v0: i32, v1: i32, v2: i32, vertices: &[Vector3Float]) {
        self.0.push(Face::new(v0, v1, v2, vertices));
    }

    /// Adds a face with the given normal.
    pub fn add_with_normal(&mut self, v0: i32, v1: i32, v2: i32, normal: Vector3Float) {
        self.0.push(Face::with_normal(v0, v1, v2, normal));
    }

    /// Replaces the list with faces read three indices at a time from `f`. A trailing partial
    /// triple is dropped rather than padded with a repeated index: three-at-a-time input is the
    /// contract.
    pub fn add_from_int_array(&mut self, f: &[i32], vertices: &[Vector3Float]) {
        self.0.clear();
        for triple in f.as_chunks::<3>().0 {
            self.0
                .push(Face::new(triple[0], triple[1], triple[2], vertices));
        }
    }

    /// The faces as a flat `v0, v1, v2, ...` index list.
    pub fn to_int_array(&self) -> Vec<i32> {
        let mut fa = Vec::with_capacity(self.0.len() * 3);
        for face in &self.0 {
            fa.push(face.v0);
            fa.push(face.v1);
            fa.push(face.v2);
        }
        fa
    }
}

impl From<Vec<Face>> for FaceList {
    fn from(faces: Vec<Face>) -> Self {
        FaceList(faces)
    }
}

impl Deref for FaceList {
    type Target = Vec<Face>;

    fn deref(&self) -> &Vec<Face> {
        &self.0
    }
}

impl DerefMut for FaceList {
    fn deref_mut(&mut self) -> &mut Vec<Face> {
        &mut self.0
    }
}
