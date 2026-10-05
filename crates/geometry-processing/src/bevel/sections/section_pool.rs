//! Every point the section construction will ever publish, in one append-only list whose indices
//! never move: the source mesh's own vertices first, at their own indices, and then one entry for
//! each point a section mints. Ports `Bevel/Sections/SectionPool.cs`.
//!
//! The one thing this is NOT is the surface vertex pool. That pool answers "is there already a
//! vertex near here?", and every crack the surface construction has left had the same shape: two
//! phases each minting their own version of one place, a ruler or two apart, so the weld does not
//! join them. The section construction never has to ask that question: a section is computed ONCE
//! at a chain vertex and handed to BOTH strips that meet there by reference, so the places two
//! neighbours share are the same index and not two points that happen to agree.
//!
//! Hence [`SectionPool::add`] always appends. There is deliberately no find-near of any kind on
//! this type: an index is shared by being passed along, never by being looked up.

use std::fmt;

use vector_math::{Vector3, Vector3Float};

/// The refusal [`SectionPool::add`] gives a NaN point (C# throws `ArgumentOutOfRangeException`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NanSectionPoint;

impl fmt::Display for NanSectionPoint {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str("a section may not mint a point at NaN")
    }
}

impl std::error::Error for NanSectionPoint {}

/// C# `SectionPool`.
#[derive(Debug, Clone, PartialEq)]
pub struct SectionPool {
    at: Vec<Vector3>,
    seeded_count: i32,
}

impl SectionPool {
    /// Seeds the pool with the source mesh's vertices, each keeping its own mesh index, so a pool
    /// index below [`SectionPool::seeded_count`] IS a mesh vertex index.
    pub fn new(mesh_vertices: &[Vector3Float]) -> Self {
        let at: Vec<Vector3> = mesh_vertices
            .iter()
            .map(|&vertex| Vector3::from_vector3_float(vertex))
            .collect();
        let seeded_count = at.len() as i32;
        SectionPool { at, seeded_count }
    }

    /// How many entries came from the mesh. Indices below this are mesh vertex indices; indices at
    /// or above it were minted by the construction.
    pub fn seeded_count(&self) -> i32 {
        self.seeded_count
    }

    /// How many points the pool holds.
    pub fn count(&self) -> i32 {
        self.at.len() as i32
    }

    /// Where the point at `index` stands.
    pub fn position_of(&self, index: i32) -> Vector3 {
        self.at[index as usize]
    }

    /// Appends a point and returns its index. ALWAYS a new index, however near an existing point it
    /// stands - see the module's remarks for why there is no other behaviour here.
    pub fn add(&mut self, position: Vector3) -> Result<i32, NanSectionPoint> {
        if position.x.is_nan() || position.y.is_nan() || position.z.is_nan() {
            return Err(NanSectionPoint);
        }

        self.at.push(position);

        Ok(self.at.len() as i32 - 1)
    }
}
