//! How far a run may carry on PAST the end of its crease, and what stops it there
//! (`Bevel/BevelFaceRunOut.cs`): what stopped a run's tangent line ([`BevelRunOutBound`]), and
//! the walk along that line over the face the bead rides ([`BevelFaceRunOut`]). Creating it and
//! measuring land with the core's packages as `impl` blocks.

use std::collections::HashMap;
use std::rc::Rc;
use std::sync::Arc;

use indexmap::IndexSet;
use polygon_mesh::{Mesh, MeshEdgeGraph};

use super::BevelSurfaceWalk;
use crate::topology::EdgeConvexity;

/// C# `BevelRunOutBound`: what stopped a run's tangent line, which is what decides how the bead
/// ends there.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum BevelRunOutBound {
    /// The mesh ran out - an open rim, or a non-manifold junction. Nothing to run out onto.
    #[default]
    OpenBoundary = 0,
    /// The face ENDS: past this edge the surface falls away, so the bead's fill tapers to nothing
    /// on the edge itself. The cube's front corner edge on "5mm Bevel Around Center.mcx", where
    /// both of the left beads' tangent lines converge onto the 45 degree knife tip.
    ConvexEdge,
    /// A WALL rises: the bead runs out flat against it, which is what the control does where the
    /// four beads eat the notch's unselected centre crease.
    ConcaveEdge,
}

/// C# `BevelFaceRunOut` (a sealed class): the part's surface read once, at the same "is this still
/// one surface" tolerance `BevelFaceFit` walks with, so a run-out and a width refusal never
/// disagree about where a face ends. Shared by every bead of a rebuild (`Rc`).
///
/// A crease ends where either of its two faces ends. Where the face that ends is the OTHER one,
/// the bead does not stop with the crease: it rides the surviving face until its own tangent line
/// leaves it. So this answers a LINE question - how far along does the tangent line stay on this
/// face, and what is at the end of it - started on the tangent line and not on the crease, which
/// are parallel but offset by the setback within the face.
#[derive(Debug)]
pub struct BevelFaceRunOut {
    /// The part.
    pub mesh: Arc<Mesh>,
    /// Walking the surface, stopped at `surface_edges`.
    pub walk: BevelSurfaceWalk,
    /// Which mesh edges end a face, at the run-out's tolerance - shared with `walk`.
    pub surface_edges: Rc<IndexSet<i32>>,
    /// Each of those edges' convexity, by mesh edge index.
    pub convexity_by_mesh_edge: HashMap<i32, EdgeConvexity>,
    /// `mesh`'s edge graph.
    pub graph: Rc<MeshEdgeGraph>,
    /// Which mesh edge joins a pair of vertices, keyed `(low, high)`. The graph indexes edges but
    /// not the pair, and the flood has only the pair to go on.
    pub edges_by_vertex_pair: HashMap<(i32, i32), i32>,
    /// The diagonal of the part's own bounding box, which is the scale a POSITION tolerance about
    /// this mesh belongs to (C#'s `Diagonal` property).
    pub diagonal: f64,
}
