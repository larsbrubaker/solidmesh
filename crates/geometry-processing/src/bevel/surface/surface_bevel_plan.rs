//! The geometry one chain's bevel is built from, worked out before anything is cut or stitched
//! ([`SurfaceBevelPlan`], `Bevel/Surface/SurfaceBevelPlan.cs`): its constructor, accessors, band
//! ends, mixed corners and per-region setbacks. Planning a chain and recording a junction land with
//! later surface packages as further `impl` blocks.

use std::cell::{Cell, Ref, RefCell};
use std::collections::HashMap;
use std::rc::Rc;
use std::sync::Arc;

use indexmap::IndexMap;

use super::{
    SurfaceBandEnd, SurfaceEdgePlan, SurfaceEdgeRoles, SurfaceMixedCorner, SurfacePlanRecipe,
};
use crate::topology::{FaceGroup, FaceRegionCache};

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

impl SurfaceBevelPlan {
    /// The narrowest wedge a bevel is planned in: ONE DEGREE, and a real angle rather than a floor
    /// put there to keep a divide from overflowing. A wedge thinner than this is not a corner
    /// anybody meant: the setback is `size / tan(wedge / 2)`, which at a degree is already 115
    /// times the size, so the tangent lines land off the part. Every later guard in the surface
    /// construction cites this angle rather than writing a tolerance of its own.
    pub const MINIMUM_WEDGE_ANGLE_DEGREES: f64 = 1.0;

    /// The flattest wedge, the same floor read from the other end: a wedge this near 180 degrees
    /// is a flat edge dressed up, with no corner to set back from.
    pub const MAXIMUM_WEDGE_ANGLE_DEGREES: f64 = 180.0 - Self::MINIMUM_WEDGE_ANGLE_DEGREES;

    /// The plane the band a cut takes off stops at, at the chain's start: square to the edge
    /// through that corner, with the chain's own side named as the side the band is on.
    /// Meaningless where that end does not run out.
    pub fn start_band_end(&self) -> SurfaceBandEnd {
        SurfaceBandEnd::new(self.edges[0].from(), self.edges[0].edge.direction())
    }

    /// The same at the end of the chain, where the band is on the near side.
    pub fn end_band_end(&self) -> SurfaceBandEnd {
        let last = &self.edges[self.edges.len() - 1];
        SurfaceBandEnd::new(last.to(), -last.edge.direction())
    }

    /// The corner between the edge at `edge_index` and the one the chain leaves on, where that
    /// corner turns the convexity over; `None` where it does not.
    pub fn try_mixed_corner_after(&self, edge_index: i32) -> Option<Rc<SurfaceMixedCorner>> {
        self.mixed_corners.borrow().get(&edge_index).cloned()
    }

    /// The widest setback this plan asks of the region `face_index` belongs to - how far back
    /// that region has to give way for the bevel to land. Zero for a region the chain does not
    /// touch, and (as for a face with no region) where the plan was read against no regions.
    pub fn required_setback(&self, face_index: i32) -> f64 {
        let Some(region) = self.regions.as_ref().and_then(|r| r.region(face_index)) else {
            return 0.0;
        };

        self.setback_by_region
            .get(&Self::key_of(&region))
            .copied()
            .unwrap_or(0.0)
    }

    /// Records `setback` against `region`'s key where it is wider than what is there already.
    pub fn record_setback(
        setback_by_region: &mut HashMap<i32, f64>,
        region: &FaceGroup,
        setback: f64,
    ) {
        let key = Self::key_of(region);

        if setback_by_region
            .get(&key)
            .is_none_or(|&already| setback > already)
        {
            setback_by_region.insert(key, setback);
        }
    }

    /// What names a region here, and everywhere else in the surface construction. Not its seed
    /// face: the cache floods once per face asked for, so the same region reached from two of its
    /// faces comes back as two groups with two different seeds. Its lowest face index is the same
    /// number whichever face it was flooded from.
    pub fn key_of(region: &FaceGroup) -> i32 {
        let mut lowest = i32::MAX;

        for &face in region.faces() {
            if face < lowest {
                lowest = face;
            }
        }

        lowest
    }
}

/// The plan's constructor and its plain accessors (C# `<init>`, the private constructor,
/// `Regions`, `MixedCornerCount`, `MixedCornerKeys`, `Roles`). The other C# properties are the
/// public fields themselves.
impl SurfaceBevelPlan {
    /// C#'s private constructor, with the field initializers (`<init>`: no junction arms yet, and
    /// the part's ruler not read yet). Called by the planner once it has read the chain.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        edges: Vec<SurfaceEdgePlan>,
        is_closed: bool,
        size: f64,
        round_profile: bool,
        regions: Option<Arc<FaceRegionCache>>,
        setback_by_region: HashMap<i32, f64>,
        start_runs_out: bool,
        end_runs_out: bool,
        start_trim: Option<SurfaceBandEnd>,
        end_trim: Option<SurfaceBandEnd>,
        mixed_corners: IndexMap<i32, Rc<SurfaceMixedCorner>>,
        roles: SurfaceEdgeRoles,
        recipe: SurfacePlanRecipe,
    ) -> Self {
        Self {
            regions,
            setback_by_region,
            mixed_corners: RefCell::new(mixed_corners),
            junction_arms: RefCell::new(HashMap::new()),
            roles: RefCell::new(roles),
            recipe,
            edges,
            is_closed,
            size,
            near_enough_on_the_part: Cell::new(None),
            round_profile,
            start_runs_out,
            end_runs_out,
            start_trim,
            end_trim,
        }
    }

    /// C# `Regions`: the regions this plan was read against, so that a reading which has to name a
    /// face NOT on the chain can ask which surface it belongs to - the face beyond a chain's end
    /// (`SurfaceRoomEndCut`).
    pub fn regions(&self) -> Option<&Arc<FaceRegionCache>> {
        self.regions.as_ref()
    }

    /// C# `MixedCornerCount`: how many corners of this chain turn the convexity over.
    pub fn mixed_corner_count(&self) -> i32 {
        self.mixed_corners.borrow().len() as i32
    }

    /// C# `MixedCornerKeys`: what names each of them - the index of the edge the chain ARRIVES on
    /// for a corner inside the chain, and a sentinel for a JUNCTION standing at one of its ends - in
    /// the order they were recorded. A copy, since recording a junction later changes the map.
    pub fn mixed_corner_keys(&self) -> Vec<i32> {
        self.mixed_corners.borrow().keys().copied().collect()
    }

    /// C# `Roles`: what part each of this chain's edges plays - which of them have strips of their
    /// own, which a blend swallows, and which edge each mixed corner's CUT is.
    pub fn roles(&self) -> Ref<'_, SurfaceEdgeRoles> {
        self.roles.borrow()
    }
}

impl SurfaceBevelPlan {
    /// The mesh the plan was read against, or `None` where it had no regions.
    pub fn mesh(&self) -> Option<&std::sync::Arc<polygon_mesh::Mesh>> {
        self.regions.as_ref().map(|regions| regions.mesh())
    }

    /// The weld ruler read off the part's own vertices (zero where the plan has no mesh), read
    /// once and remembered.
    pub fn near_enough_on_the_part(&self) -> f64 {
        if let Some(known) = self.near_enough_on_the_part.get() {
            return known;
        }

        let near_enough = self.mesh().map_or(0.0, |mesh| {
            super::SurfaceVertexPool::near_enough_for(&mesh.vertices)
        });
        self.near_enough_on_the_part.set(Some(near_enough));
        near_enough
    }

    /// What names the corner standing at one END of this chain, where a JUNCTION stands there: a
    /// key no edge index can be (-1 at the start, -2 at the end).
    pub fn junction_key(at_the_start: bool) -> i32 {
        if at_the_start {
            -1
        } else {
            -2
        }
    }

    /// The junction's corner at the chain's start (or end), where one stands there.
    pub fn try_junction_at(&self, at_the_start: bool) -> Option<Rc<SurfaceMixedCorner>> {
        self.mixed_corners
            .borrow()
            .get(&Self::junction_key(at_the_start))
            .cloned()
    }
}
