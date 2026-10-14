//! The geometry one chain's bevel is built from, worked out before anything is cut or stitched
//! ([`SurfaceBevelPlan`]): its constructor, accessors, band
//! ends, mixed corners, per-region setbacks, and where each edge's band stops
//! ([`SurfaceBevelPlan::band_ends_of`]), recording a junction
//! ([`SurfaceBevelPlan::try_record_junction`]) and planning a chain
//! ([`SurfaceBevelPlan::try_create`]).

use std::cell::{Cell, Ref, RefCell};
use std::collections::HashMap;
use std::rc::Rc;
use std::sync::Arc;

use indexmap::IndexMap;

use vector_math::Vector3;

use super::surface_run_across::SurfaceRunAcrossRegionMapsCache;
use super::{
    surface_end_across_the_corner, surface_face_carries_on, surface_handover_leaves_a_strip,
    surface_junction_band_ends, surface_mitre_on_a_chorded_run, surface_mixed_corner_readings,
    surface_plan_band_ends, surface_plan_edge_reading,
};
use super::{
    SurfaceBandEnd, SurfaceChainEndReading, SurfaceEdgePlan, SurfaceEdgeRoles, SurfaceMixedCorner,
    SurfacePlanRecipe, SurfaceRegionFaceGrid,
};
use crate::bevel::bevel_chain_linker::BevelSweptChain;
use crate::bevel::{bevel_edge_tool_builder, BevelAdvisory, BevelAdvisoryCode};
use crate::topology::{EdgeConvexity, FaceGroup, FaceRegionCache};

/// Per chain edge, the two face regions, the wedge they
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
    /// chain can ask which surface it belongs to; `None` where it had none.
    pub regions: Option<Arc<FaceRegionCache>>,
    /// The widest setback the plan asks of each region, by the region's key (its lowest face
    /// index). Only looked up.
    pub setback_by_region: HashMap<i32, f64>,
    /// The corners where the chain's convexity turns over, keyed by the index of the edge the
    /// chain ARRIVES on (a closed chain's last corner at the last edge's own key), plus a junction
    /// at either END under its sentinel key. Iterated in insertion order, so every run reads the
    /// corners the same way.
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
    /// until first asked.
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

/// The plan's constructor and its plain accessors (the regions, the mixed corners and their keys,
/// the roles). Everything else is read off the public fields themselves.
impl SurfaceBevelPlan {
    /// A plan with no junction arms yet and the part's ruler not read yet. Called by the planner
    /// once it has read the chain.
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

    /// The regions this plan was read against, so that a reading which has to name a
    /// face NOT on the chain can ask which surface it belongs to - the face beyond a chain's end
    /// (`SurfaceRoomEndCut`).
    pub fn regions(&self) -> Option<&Arc<FaceRegionCache>> {
        self.regions.as_ref()
    }

    /// How many corners of this chain turn the convexity over.
    pub fn mixed_corner_count(&self) -> i32 {
        self.mixed_corners.borrow().len() as i32
    }

    /// What names each of them - the index of the edge the chain ARRIVES on
    /// for a corner inside the chain, and a sentinel for a JUNCTION standing at one of its ends - in
    /// the order they were recorded. A copy, since recording a junction later changes the map.
    pub fn mixed_corner_keys(&self) -> Vec<i32> {
        self.mixed_corners.borrow().keys().copied().collect()
    }

    /// What part each of this chain's edges plays - which of them have strips of their
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

/// Where each edge's band stops.
impl SurfaceBevelPlan {
    /// Records the corner a JUNCTION reading found at one end of this
    /// chain (`at_the_start` or its end) - `readings` is the corner as each arm read it (one on a
    /// cut's chain, both on the bead's), `edge_index` the edge of this chain that meets it, and
    /// `swallowed` the chords between, which owe no strip. The handover check is made again, since
    /// the band ends it reads are the ones that have just changed. `Err` is why it would not:
    /// that end already carries a corner of its own, or the handover leaves no strip.
    pub fn try_record_junction(
        &self,
        at_the_start: bool,
        readings: &[Rc<SurfaceMixedCorner>],
        edge_index: i32,
        swallowed: &[i32],
    ) -> Result<(), BevelAdvisory> {
        let corner = Self::junction_key(at_the_start);

        if !self.roles.borrow_mut().read_junction(
            corner,
            edge_index,
            at_the_start,
            self.edges[edge_index as usize].convexity() == EdgeConvexity::Convex,
            swallowed,
        ) {
            // That end of that edge already carries a corner of its own, which two corners
            // cannot share.
            return Err(BevelAdvisory::new(
                BevelAdvisoryCode::SurfaceCornerNotSupported,
                vec![readings[0].corner.into()],
            ));
        }

        self.mixed_corners
            .borrow_mut()
            .insert(corner, readings[0].clone());
        self.junction_arms
            .borrow_mut()
            .insert(corner, readings.to_vec());

        surface_handover_leaves_a_strip::try_check(self)
    }
}

impl SurfaceBevelPlan {
    /// The planes the band on one face of edge `edge_index` (`left` or right) stops
    /// at, one per end that has one.
    ///
    /// At an interior corner that is the mitre plane with the neighbour (through the chorded run's
    /// joint where the two are pieces of one chord) where the mitre bounds this side; where the
    /// convexity turns over it is the mixed corner's own plane for this edge - left off the face the
    /// two edges SHARE, which the other edge's band ends, unless the corner hands over on a mitre
    /// ring ([`SurfaceMixedCorner::both_bands_stop_on_the_shared_face`]). At a free end it is the
    /// junction's reading, the end's trim, or the square plane where the end runs out.
    ///
    /// Which corner stands at an end is the ROLES' answer and not the neighbour's index: where a
    /// blend swallows an edge, the corner's cut is further along the chain than the corner is.
    /// The mixed-corner path reads only `readings[0]`: a second junction arm owed at an interior
    /// two-crease corner is not read, and no fixture reaches one.
    pub fn band_ends_of(&self, edge_index: usize, left: bool) -> Vec<SurfaceBandEnd> {
        let mut ends = Vec::with_capacity(2);
        let last = self.edges.len() - 1;
        let here = &self.edges[edge_index];
        let region_key = if left {
            here.left_region_key()
        } else {
            here.right_region_key()
        };
        let mixed_at = |where_the_chain_leaves: bool| {
            let at = self
                .roles
                .borrow()
                .try_corner_at(edge_index as i32, where_the_chain_leaves)?;
            self.mixed_corners.borrow().get(&at).cloned()
        };

        if edge_index > 0 || self.is_closed {
            let arrived_on = if edge_index > 0 { edge_index - 1 } else { last };

            if let Some(mixed) = mixed_at(false) {
                // The convexity turns over here, so the two bands do not meet on a mitre plane: each
                // stops on the plane the OTHER edge's direction names through the point the two
                // strips touch at; the shared face runs on and the other edge's band ends it.
                if mixed.shared_region_key != region_key
                    || mixed.both_bands_stop_on_the_shared_face()
                {
                    ends.push(mixed.band_end_for(here));
                }
            } else if surface_plan_band_ends::the_mitre_bounds_this_side(
                here,
                &self.edges[arrived_on],
                region_key,
            ) {
                // Including a SWALLOWED edge, whose band is bounded by the ordinary mitre planes with
                // its two neighbours: it owes no strip, but the band it takes off is its own stretch.
                ends.push(surface_plan_band_ends::band_end_where_the_chain_arrives(
                    &self.edges[arrived_on],
                    here,
                    self.the_chorded_runs_joint_after(arrived_on),
                ));
            }
        } else if let Some(junction) = self.junction_arms.borrow().get(&Self::junction_key(true)) {
            surface_junction_band_ends::add(&mut ends, junction, here, region_key);
        } else if let Some(trim) = self.start_trim {
            ends.push(trim);
        } else if self.start_runs_out {
            ends.push(self.start_band_end());
        }

        if edge_index < last || self.is_closed {
            if let Some(mixed) = mixed_at(true) {
                if mixed.shared_region_key != region_key
                    || mixed.both_bands_stop_on_the_shared_face()
                {
                    ends.push(mixed.band_end_for(here));
                }
            } else {
                let leaving_for = &self.edges[if edge_index < last { edge_index + 1 } else { 0 }];

                if surface_plan_band_ends::the_mitre_bounds_this_side(here, leaving_for, region_key)
                {
                    ends.push(surface_plan_band_ends::band_end_where_the_chain_leaves(
                        here,
                        leaving_for,
                        self.the_chorded_runs_joint_after(edge_index),
                    ));
                }
            }
        } else if let Some(junction) = self.junction_arms.borrow().get(&Self::junction_key(false)) {
            surface_junction_band_ends::add(&mut ends, junction, here, region_key);
        } else if let Some(trim) = self.end_trim {
            ends.push(trim);
        } else if self.end_runs_out {
            ends.push(self.end_band_end());
        }

        ends
    }

    /// Where the mitre plane between edge `leaving` and the one after
    /// it stands, when the two are two pieces of ONE chord a boolean split and the corner between
    /// them is not where the plane goes - `None` at every ordinary corner
    /// (`SurfaceMitreOnAChordedRun`). Read here because it is the RUN's reading: it needs the two
    /// pieces' outer neighbours, which only the chain's own edge list holds.
    pub fn the_chorded_runs_joint_after(&self, leaving: usize) -> Option<Vector3> {
        surface_mitre_on_a_chorded_run::try_the_point(
            &self.edges,
            self.is_closed,
            leaving,
            self.near_enough_on_the_part(),
        )
    }
}

impl SurfaceBevelPlan {
    /// Plans one resolved
    /// chain with no curve-tolerance reading, so no end of it runs out on a crease carrying on.
    pub fn try_create(
        chain: &BevelSweptChain,
        regions: &Arc<FaceRegionCache>,
        size: f64,
        round_profile: bool,
    ) -> Result<Rc<SurfaceBevelPlan>, BevelAdvisory> {
        Self::try_create_with_curve_regions(chain, regions, None, size, round_profile)
    }

    /// Plans
    /// one resolved chain, told as well how the mesh reads as SURFACES (`curve_regions`), which is
    /// read for the chain's free ENDS only - whether the crease it stops at goes on without it.
    /// Refuses, by name, a chain with no edges, a size that removes nothing, an edge
    /// the plan cannot read, a mixed corner it cannot place, a concave end that runs out with
    /// nothing to cut its bead off by, or a handover that leaves no strip.
    ///
    /// The region face grids a round's curved side asks for are kept by this call, one per region
    /// for the whole chain: a grid's answers depend only on the mesh and the region, so one laid
    /// grid serves every edge, with no process-wide cache shared between rebuild workers.
    pub fn try_create_with_curve_regions(
        chain: &BevelSweptChain,
        regions: &Arc<FaceRegionCache>,
        curve_regions: Option<&Arc<FaceRegionCache>>,
        size: f64,
        round_profile: bool,
    ) -> Result<Rc<SurfaceBevelPlan>, BevelAdvisory> {
        if chain.edges.is_empty() {
            return Err(BevelAdvisory::new(
                BevelAdvisoryCode::NoEdgesResolved,
                Vec::new(),
            ));
        }

        bevel_edge_tool_builder::try_validate_size(size)?;

        let mut planned: Vec<SurfaceEdgePlan> = Vec::with_capacity(chain.edges.len());
        let mut setback_by_region = HashMap::new();

        // One reading of each region's facets for the whole chain.
        let mut region_maps = SurfaceRunAcrossRegionMapsCache::new();
        let mesh = regions.mesh().clone();
        let mut grids: HashMap<i32, Rc<SurfaceRegionFaceGrid>> = HashMap::new();
        let mut grid_of = |region: &FaceGroup| {
            Some(
                grids
                    .entry(Self::key_of(region))
                    .or_insert_with(|| Rc::new(SurfaceRegionFaceGrid::new(&mesh, region)))
                    .clone(),
            )
        };

        for chain_edge in &chain.edges {
            let edge_plan = surface_plan_edge_reading::try_plan_edge(
                chain_edge,
                regions,
                size,
                round_profile,
                &mut region_maps,
                &mut grid_of,
            )?;

            Self::record_setback(
                &mut setback_by_region,
                &edge_plan.left_region,
                edge_plan.left_setback,
            );
            Self::record_setback(
                &mut setback_by_region,
                &edge_plan.right_region,
                edge_plan.right_setback,
            );
            planned.push(edge_plan);
        }

        let (mixed_corners, roles) = surface_mixed_corner_readings::try_read_all(
            &planned,
            chain.is_closed,
            size,
            regions.mesh(),
        )?;

        let first = &chain.edges[0];
        let last = &chain.edges[chain.edges.len() - 1];
        let first_plan = &planned[0];
        let last_plan = &planned[planned.len() - 1];

        // One reading for both ends (SurfaceChainEndReading).
        let ends = if chain.is_closed {
            None
        } else {
            SurfaceChainEndReading::for_regions(curve_regions)
        };

        // The two halves of a run-out, kept apart because a CONCAVE end wants opposite things of
        // them: the CREASE carrying on past the corner, or only a FACE carrying on.
        let start_crease_carries_on = !chain.is_closed
            && ends
                .as_ref()
                .is_some_and(|ends| ends.runs_out_at(first, first.from_vertex()));

        let end_crease_carries_on = !chain.is_closed
            && ends
                .as_ref()
                .is_some_and(|ends| ends.runs_out_at(last, last.to_vertex()));

        let start_runs_out = start_crease_carries_on
            || (!chain.is_closed
                && surface_face_carries_on::past(
                    Some(regions),
                    first_plan,
                    first.from_vertex(),
                    first_plan.from(),
                    -first.direction(),
                ));

        let end_runs_out = end_crease_carries_on
            || (!chain.is_closed
                && surface_face_carries_on::past(
                    Some(regions),
                    last_plan,
                    last.to_vertex(),
                    last_plan.to(),
                    last.direction(),
                ));

        // A concave end where the CREASE stops is cut off by a face standing ACROSS it, where one
        // does (SurfaceEndAcrossTheCorner); the crease carrying on is the stronger reading.
        let start_trim = if start_runs_out && !start_crease_carries_on {
            surface_end_across_the_corner::try_read(
                Some(regions),
                first_plan,
                first.from_vertex(),
                first_plan.from(),
                -first.direction(),
            )
        } else {
            None
        };

        let end_trim = if end_runs_out && !end_crease_carries_on {
            surface_end_across_the_corner::try_read(
                Some(regions),
                last_plan,
                last.to_vertex(),
                last_plan.to(),
                last.direction(),
            )
        } else {
            None
        };

        // And where the crease stopped and no face across named a plane, a CONCAVE run-out is
        // refused here rather than built, named at the corner.
        if start_runs_out
            && !start_crease_carries_on
            && start_trim.is_none()
            && first_plan.convexity() == EdgeConvexity::Concave
        {
            return Err(BevelAdvisory::new(
                BevelAdvisoryCode::SurfaceStripEndNotSupported,
                vec![first_plan.from().into()],
            ));
        }

        if end_runs_out
            && !end_crease_carries_on
            && end_trim.is_none()
            && last_plan.convexity() == EdgeConvexity::Concave
        {
            return Err(BevelAdvisory::new(
                BevelAdvisoryCode::SurfaceStripEndNotSupported,
                vec![last_plan.to().into()],
            ));
        }

        let plan = SurfaceBevelPlan::new(
            planned,
            chain.is_closed,
            size,
            round_profile,
            Some(regions.clone()),
            setback_by_region,
            start_runs_out && start_trim.is_none(),
            end_runs_out && end_trim.is_none(),
            start_trim,
            end_trim,
            mixed_corners,
            roles,
            SurfacePlanRecipe::new(
                chain.clone(),
                Some(regions.clone()),
                curve_regions.cloned(),
                round_profile,
            ),
        );

        // And a blend that swallowed an edge has to have left its own CUT edge a band to hand
        // over to (SurfaceHandoverLeavesAStrip).
        surface_handover_leaves_a_strip::try_check(&plan)?;

        Ok(Rc::new(plan))
    }
}
