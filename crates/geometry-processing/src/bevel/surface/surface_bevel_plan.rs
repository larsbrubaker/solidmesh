//! The geometry one chain's bevel is built from, worked out before anything is cut or stitched
//! ([`SurfaceBevelPlan`], `Bevel/Surface/SurfaceBevelPlan.cs`). Planning a chain, its band ends,
//! recording a junction and the per-region setbacks land with the core's surface packages as
//! `impl` blocks.

use std::cell::{Cell, RefCell};
use std::collections::HashMap;
use std::rc::Rc;
use std::sync::Arc;

use indexmap::IndexMap;

use super::{
    SurfaceBandEnd, SurfaceEdgePlan, SurfaceEdgeRoles, SurfaceMixedCorner, SurfacePlanRecipe,
};
use crate::topology::FaceRegionCache;

/// C# `SurfaceBevelPlan` (a sealed class): per chain edge, the two face regions, the wedge they
/// open, the setback into each of them, the two tangent lines the faces are walked back to, and a
/// round's center line ([`SurfaceEdgePlan`]). It decides WHERE the bevel lies and nothing else: it
/// cuts no triangles, builds no strip, and does not decide whether there is room for what it
/// plans - it reports the setback each region is asked for, and the room is the caller's.
///
/// Shared by every reading of the chain (its section run, the room, the junctions), hence held
/// through `Rc`; recording a junction after the chain is planned changes it in place, so those
/// fields are `RefCell`s.
#[derive(Debug)]
pub struct SurfaceBevelPlan {
    /// The regions the plan was read against, so a reading that has to name a face NOT on the
    /// chain can ask which surface it belongs to; `None` (C# null) where it had none.
    pub regions: Option<Arc<FaceRegionCache>>,
    /// The widest setback the plan asks of each region, by the region's key (its lowest face
    /// index). Only looked up.
    pub setback_by_region: HashMap<i32, f64>,
    /// The corners where the chain's convexity turns over, keyed by the index of the edge the
    /// chain ARRIVES on (a closed chain's last corner at the last edge's own key), plus a junction
    /// at either END under its sentinel key. Iterated in insertion order (C#'s `Dictionary`).
    pub mixed_corners: RefCell<IndexMap<i32, Rc<SurfaceMixedCorner>>>,
    /// Every reading of the corner standing at one END of this chain, by the sentinel naming that
    /// end: one on a cut's chain, both on the bead's. Only looked up.
    pub junction_arms: RefCell<HashMap<i32, Vec<Rc<SurfaceMixedCorner>>>>,
    /// What part each edge plays: which have strips of their own, which a blend swallows, and
    /// which edge each mixed corner's CUT is.
    pub roles: RefCell<SurfaceEdgeRoles>,
    /// What this plan was drawn from, so the same chain can be planned again at another size.
    pub recipe: SurfacePlanRecipe,
    /// One plan per chain edge, in the chain's walk order.
    pub edges: Vec<SurfaceEdgePlan>,
    /// Whether the chain closes on itself, so that no edge of it has a free end.
    pub is_closed: bool,
    /// The fillet radius or chamfer setback the plan was drawn at.
    pub size: f64,
    /// Remembers `NearEnoughOnThePart`, the weld ruler read off the part's own vertices; `None`
    /// until first asked (C#'s `double?`).
    pub near_enough_on_the_part: Cell<Option<f64>>,
    /// Whether the profile is a round's arc rather than a chamfer's straight cut.
    pub round_profile: bool,
    /// Whether the chain STOPS at its first corner rather than ending there, so the groove is
    /// closed by a wall of its own rather than notched into a face beyond the corner.
    pub start_runs_out: bool,
    /// The same for the corner the chain's last edge ends at.
    pub end_runs_out: bool,
    /// The plane a CONCAVE chain's first corner cuts its bead off on, where a face across that
    /// corner names one.
    pub start_trim: Option<SurfaceBandEnd>,
    /// The same at the corner the chain's last edge ends at.
    pub end_trim: Option<SurfaceBandEnd>,
}
