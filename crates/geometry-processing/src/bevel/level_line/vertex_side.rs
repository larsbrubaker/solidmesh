//! Which side of the level line a mesh vertex stands on ([`VertexSide`]) and the per-vertex reading
//! cache the construction shares ([`VertexFieldReading`]). Ports the types of
//! `Bevel/LevelLine/VertexSide.cs`; the reading's queries land with the refinement's method
//! package.

use std::cell::RefCell;
use std::collections::HashMap;
use std::rc::Rc;

use vector_math::Vector3;

use super::{LevelLineMesh, ScalarField};

/// C# `VertexSide`: which side of the level line a mesh vertex stands on, decided once and read
/// by every triangle round it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum VertexSide {
    /// Field value below the snap band: in the material the cut deletes.
    #[default]
    Inside = 0,

    /// Within the snap band of the line: the line passes through this existing vertex.
    On,

    /// Field value above the snap band: in the material the cut keeps.
    Outside,
}

/// C# `VertexFieldReading`: the per-vertex reading of the field, and the cache the rest of the
/// construction shares so that a vertex is never evaluated twice and never answers two different
/// numbers. The C# fills the cache through every holder's reference, so the caches are
/// interior-mutable here.
pub struct VertexFieldReading {
    /// The working mesh whose vertices are read.
    pub mesh: Rc<RefCell<LevelLineMesh>>,
    /// The field being read - the refinement asks a chain distance field for more than a value at
    /// a point.
    pub field: Rc<dyn ScalarField>,
    /// The field value per vertex read so far. Read ONE AT A TIME, not filled up to the vertex
    /// asked for: the band a cut touches is a few hundred triangles of a part that may have tens of
    /// thousands, and the dense fill this replaces charged every build a field query at every
    /// vertex below the highest one anybody asked about - 20,000 of the cylinder's 63,000 queries,
    /// none of which any answer depended on.
    pub values: RefCell<HashMap<i32, f64>>,
    /// The side per vertex read so far.
    pub sides: RefCell<HashMap<i32, VertexSide>>,
    /// Edge roots already found on this reading's field, by the exact ordered end points (the six
    /// coordinates' bits, -0.0 read as 0.0, as C# double equality has them), with the work units
    /// finding each one charged.
    pub remembered_roots: RefCell<HashMap<[u64; 6], (Vector3, i64)>>,
    /// The half-width of the band, as a LENGTH, inside which a vertex counts as on the line. Never
    /// negative (C#'s constructor refuses it).
    pub snap: f64,
}

impl std::fmt::Debug for VertexFieldReading {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        // The field has no Debug; give the snap and how much has been read.
        f.debug_struct("VertexFieldReading")
            .field("snap", &self.snap)
            .field("values_read", &self.values.borrow().len())
            .finish_non_exhaustive()
    }
}
