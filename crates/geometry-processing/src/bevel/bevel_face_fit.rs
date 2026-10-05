//! Whether a group's bevel has FACE to land on ([`BevelFaceFit`], `Bevel/BevelFaceFit.cs` and
//! `Bevel/BevelFaceFit.Exemptions.cs`): the plan's other size limit - not how far the tool runs
//! along the chain but how far it reaches across the two faces beside it, measured by walking the
//! surface ([`BevelSurfaceWalk`]) square to each selected edge. Creating the check, the walk per
//! sample and the exemptions land with the core's packages as `impl` blocks.

use std::cell::RefCell;
use std::collections::{HashMap, HashSet};
use std::rc::Rc;
use std::sync::Arc;

use indexmap::IndexSet;
use polygon_mesh::{Mesh, MeshEdgeGraph};

use super::{BevelEndCap, BevelSurfaceWalk};
use crate::topology::BevelEdgeGroupResolution;

/// One group's canonical free-end caps, by `(vertex, mesh edge)` of the run end they cap.
pub type BevelFaceFitEndCaps = HashMap<(i32, i32), BevelEndCap>;

/// C# `BevelFaceFit` (a sealed partial class): one rebuild's face-width check. Every selected edge
/// sets the cut back from itself along each of its two faces; the walk measures how much face
/// there IS to set back into, and a group whose setback does not fit is refused with the largest
/// size that would have.
#[derive(Debug)]
pub struct BevelFaceFit {
    /// The welded working mesh the groups were resolved against.
    pub mesh: Arc<Mesh>,
    /// `mesh`'s edge graph.
    pub graph: Rc<MeshEdgeGraph>,
    /// Every group of the rebuild, in the operation's own order. Unresolved ones are ignored.
    pub resolutions: Rc<Vec<BevelEdgeGroupResolution>>,
    /// True for a fillet's radius, false for a chamfer's setback.
    pub round_profile: bool,
    /// The operation's one bevel size, which every group of it is measured at.
    pub size: f64,
    /// Which mesh edges end a face, by mesh edge index, at the walk's own tolerance - the set
    /// `walk` stops at, shared with it.
    pub surface_edges: Rc<IndexSet<i32>>,
    /// The concave ones of those, which are the only stops a run's end cap can be argued to cover.
    pub concave_surface_edges: HashSet<i32>,
    /// Every selected mesh edge at each of its two vertices, so a run's END can be told from a
    /// corner two selected runs turn. A set rather than a count because one edge picked into two
    /// groups is one edge.
    pub selected_edges_at_vertex: HashMap<i32, HashSet<i32>>,
    /// Canonical free-end caps, read only for groups whose face width needs a cap exemption: by
    /// the group's index in `resolutions` (C# keys by the resolution object), then by
    /// `(vertex, mesh edge)`. Filled lazily the first time a group asks, hence the `RefCell`.
    pub free_end_caps: RefCell<HashMap<usize, BevelFaceFitEndCaps>>,
    /// How far off a plane a vertex may lie and still be called ON it, scaled to the part because
    /// mesh vertices are float.
    pub plane_tolerance: f64,
    /// Which mesh edges are themselves being cut, and by how much.
    pub setbacks: HashMap<i32, f64>,
    /// Walking the surface to find where a face ends - the measurement, split from the policy
    /// here.
    pub walk: BevelSurfaceWalk,
    /// The largest of `setbacks`.
    pub largest_setback: f64,
}
