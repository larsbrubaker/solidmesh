//! The corner where THREE picked edges meet and their convexity is mixed
//! ([`SurfaceMixedJunction`]), which shape of junction it is ([`SurfaceJunctionKind`]), one cut's
//! side of it ([`SurfaceMixedJunctionArm`]) and a free chain end the gathering found
//! ([`SurfaceMixedJunctionChainEnd`]). Ports `Bevel/Surface/SurfaceMixedJunction.cs`: the
//! records, the reading of one junction and of a whole pass's, and the hand-over to the plans.

use std::collections::{HashMap, HashSet};
use std::rc::Rc;

use vector_math::Vector3;

use super::{
    surface_mixed_corner_cut_edge, surface_mixed_corner_hands_over_at_the_mitre,
    surface_two_crease_junction, SurfaceBevelPlan, SurfaceEdgePlan, SurfaceLine,
    SurfaceMixedCorner, SurfaceMixedCornerDualFrame,
};
use crate::bevel::{BevelAdvisory, BevelAdvisoryArg, BevelAdvisoryCode};
use crate::topology::EdgeConvexity;

/// C# `SurfaceJunctionKind`: which shape of junction a [`SurfaceMixedJunction`] is - which of the
/// three picked edges at the vertex are the creases.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum SurfaceJunctionKind {
    /// Two cuts and ONE bead - the corner [`SurfaceMixedJunction`] was written about.
    #[default]
    OneCrease = 0,

    /// One cut and TWO beads - a ridge running into the two creases its own two faces make with the
    /// face they both stand against (`SurfaceTwoCreaseJunction`).
    TwoCreases = 1,
}

/// C# `SurfaceMixedJunction.ChainEnd` (a readonly struct): a free end of one chain, as the
/// gathering found it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct SurfaceMixedJunctionChainEnd {
    /// Which plan of the pass the chain is.
    pub plan: i32,
    /// Which edge of that chain the end is.
    pub edge: i32,
    /// Whether the end is the chain's START rather than its finish.
    pub at_the_start: bool,
}

/// C# `SurfaceMixedJunction.Arm` (a readonly struct): one cut's side of the junction, with
/// everything the plan and the patch need to quote.
#[derive(Debug, Clone)]
pub struct SurfaceMixedJunctionArm {
    /// Which plan of the pass this cut's chain is.
    pub plan: i32,
    /// Which edge of that chain the cut is - its end edge, unless a blend swallowed chords.
    pub cut_index: i32,
    /// Whether the corner stands at the START of that chain rather than its finish.
    pub corner_at_the_start: bool,
    /// The chords between the cut and the corner, which owe no strip of their own.
    pub swallowed: Vec<i32>,
    /// The mixed corner this cut makes with the bead.
    pub mixed: Rc<SurfaceMixedCorner>,
    /// The end of the BEAD's chain this arm reads - a two-crease junction has one per arm.
    ///
    /// Its `edge` is the edge that really CARRIES the blend's end ring, which on a crease chorded
    /// beside the corner is not the chain's own end edge (`SurfaceTwoCreaseJunction`); the chords
    /// between the two are [`SurfaceMixedJunctionArm::bead_swallowed`]. Its `at_the_start` is
    /// still which END of the chain the corner stands at, because that is what says which way the
    /// walk away from it runs.
    pub bead_end: SurfaceMixedJunctionChainEnd,
    /// The bead's chords between the corner and [`SurfaceMixedJunctionArm::bead_end`] - the ones
    /// the blend reaches over, which owe no strip of their own, exactly as
    /// [`SurfaceMixedJunctionArm::swallowed`] is for the cut. Empty at a one-crease junction, which
    /// does not read the bead this way (C# defaults a null argument to the empty array).
    pub bead_swallowed: Vec<i32>,
    /// The canal the blend really is at a TWO-crease junction - the bead's void ball rolling along
    /// the wall both creases stand against, pressed against the cut's tube - or `None` at a
    /// one-crease junction, which is the cap-riding torus [`SurfaceMixedJunctionArm::mixed`]
    /// already carries.
    pub dual: Option<Rc<SurfaceMixedCornerDualFrame>>,
}

/// C# `SurfaceMixedJunction` (a sealed class): the corner where THREE picked edges meet and their
/// convexity is mixed, read as the two mixed corners it really is rather than as a corner of a
/// third kind. With ONE crease among the three that is two cuts and a bead; with TWO it is one cut
/// and two beads, and the arms are read the same way but the canal between them is another shape
/// altogether (`SurfaceTwoCreaseJunction`, which this hands over to).
///
/// **Why it is not the octant.** The three-convex corner is closed by the ball that touches all
/// three faces at once (`SurfaceCornerPatch`). With a crease among the three there is no such
/// ball: the ball touching the two cuts' faces sits a radius INSIDE the material at their corner,
/// and a ball rolling on the bead stands `2r` from the bead's axis out in the void. On the L prism
/// at `r = 1` the two readings stand `2 root 2` apart and a patch fanned from the first cuts the
/// bead off a whole radius short (`SurfaceMixedThreeEdgeCornerTests`).
///
/// **What it is instead.** Each cut is tangent to the bead at ONE point, on the face that cut and
/// the bead SHARE - a mixed corner of exactly the shape [`SurfaceMixedCorner`] already reads, one
/// per cut. The two readings are of one bead, so they share an axis, an outer rail and a turn, and
/// the ball the first hands over at is the very ball the second starts from. So the blend between
/// the two cuts is ONE sweep of the rolling ball about the bead's axis, and it needs no runout:
/// the turn ends ON the second cut and the surface closes.
///
/// **What it refuses, and what it hands on.** Four or more picked edges, a CHAMFER at the corner,
/// or a cut that does not share the cap face with the other cut are refused by name; TWO creases is
/// a corner of its own (`SurfaceTwoCreaseJunction`); three creases has no cut at all. Each arm is
/// read like any other mixed corner, chorded side and all, so one arm of a junction can lean while
/// the other does not, which is the lobed prism's own seam.
#[derive(Debug, Clone)]
pub struct SurfaceMixedJunction {
    /// The vertex all three edges meet at.
    pub corner: Vector3,
    /// The mesh's own index for it, which is what gathered the three ends.
    pub vertex: i32,
    /// Which of the two shapes of junction this is - one crease among the three, or two.
    pub kind: SurfaceJunctionKind,
    /// The arm the blend's turn STARTS on - turn zero stands on this cut's centre line.
    pub first: SurfaceMixedJunctionArm,
    /// The arm it ends on, whose centre line the ball stands on at the far end of the turn.
    pub second: SurfaceMixedJunctionArm,
    /// What names the face BOTH cuts come down from - the cap the outer rail runs across - or -1
    /// where there is no such face. A TWO-crease junction has none: its one cut's two faces are
    /// each SHARED with a crease, so each arm's cap is the other arm's shared face.
    pub cap_region_key: i32,
}

impl SurfaceMixedJunction {
    /// The bead's rolling-ball axis, the one line both arms turn about.
    pub fn axis(&self) -> SurfaceLine {
        self.first.mixed.axis()
    }

    /// How far the ball rolls from the first cut's centre line round to the second's.
    pub fn turn_radians(&self) -> f64 {
        self.first.mixed.turn_radians()
    }

    /// The face the cut comes down from, which is its side that is NOT the one it shares with the
    /// bead.
    pub fn cap_key_of(mixed: &SurfaceMixedCorner) -> i32 {
        if mixed.cut.left_region_key() == mixed.shared_region_key {
            mixed.cut.right_region_key()
        } else {
            mixed.cut.left_region_key()
        }
    }
}

impl SurfaceMixedJunctionChainEnd {
    /// C#'s constructor.
    pub fn new(plan: i32, edge: i32, at_the_start: bool) -> Self {
        SurfaceMixedJunctionChainEnd {
            plan,
            edge,
            at_the_start,
        }
    }
}

impl SurfaceMixedJunctionArm {
    /// C#'s constructor; `bead_swallowed` is `None` for C#'s default (none).
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        plan: i32,
        cut_index: i32,
        corner_at_the_start: bool,
        swallowed: Vec<i32>,
        mixed: Rc<SurfaceMixedCorner>,
        bead_end: SurfaceMixedJunctionChainEnd,
        dual: Option<Rc<SurfaceMixedCornerDualFrame>>,
        bead_swallowed: Option<Vec<i32>>,
    ) -> Self {
        SurfaceMixedJunctionArm {
            plan,
            cut_index,
            corner_at_the_start,
            swallowed,
            mixed,
            bead_end,
            bead_swallowed: bead_swallowed.unwrap_or_default(),
            dual,
        }
    }
}

impl SurfaceMixedJunction {
    /// How near the two arms' readings of the same turn have to agree - an angle, so fixed.
    pub const SAME_TURN: f64 = 1e-6;

    /// C#'s private constructor.
    pub fn new(
        corner: Vector3,
        vertex: i32,
        kind: SurfaceJunctionKind,
        first: SurfaceMixedJunctionArm,
        second: SurfaceMixedJunctionArm,
        cap_region_key: i32,
    ) -> Self {
        SurfaceMixedJunction {
            corner,
            vertex,
            kind,
            first,
            second,
            cap_region_key,
        }
    }

    /// The bead's plan.
    pub fn bead_plan(&self) -> i32 {
        self.first.bead_end.plan
    }

    /// The bead's edge at the junction.
    pub fn bead_index(&self) -> i32 {
        self.first.bead_end.edge
    }

    /// Whether the junction stands at the bead edge's start.
    pub fn bead_corner_at_the_start(&self) -> bool {
        self.first.bead_end.at_the_start
    }

    /// The three chain ends arriving at the junction: on a two-crease junction the cut's and the
    /// two beads'; otherwise the two cuts' and the bead's.
    pub fn the_arriving_ends(&self) -> [SurfaceMixedJunctionChainEnd; 3] {
        let first_cut = SurfaceMixedJunctionChainEnd::new(
            self.first.plan,
            self.first.cut_index,
            self.first.corner_at_the_start,
        );

        if self.kind == SurfaceJunctionKind::TwoCreases {
            return [first_cut, self.first.bead_end, self.second.bead_end];
        }

        [
            first_cut,
            SurfaceMixedJunctionChainEnd::new(
                self.second.plan,
                self.second.cut_index,
                self.second.corner_at_the_start,
            ),
            SurfaceMixedJunctionChainEnd::new(
                self.bead_plan(),
                self.bead_index(),
                self.bead_corner_at_the_start(),
            ),
        ]
    }

    /// The bead's plan edge.
    pub fn bead(&self) -> &SurfaceEdgePlan {
        &self.first.mixed.bead
    }

    /// Where the bead's axis meets the cap face - the centre of the arc the ball traces on it.
    pub fn outer_rail_centre(&self) -> Vector3 {
        self.first.mixed.outer_rail_centre
    }

    /// The junction of ONE cut and TWO creases at `vertex`.
    pub fn of_two_creases(
        corner: Vector3,
        vertex: i32,
        first: SurfaceMixedJunctionArm,
        second: SurfaceMixedJunctionArm,
    ) -> Self {
        Self::new(
            corner,
            vertex,
            SurfaceJunctionKind::TwoCreases,
            first,
            second,
            -1,
        )
    }

    /// The open chain ends standing at `vertex` across `plans`, where exactly three edges touch it
    /// and all three are chain ends (a junction's shape); `None` otherwise.
    pub fn try_gather_the_ends(
        plans: &[Rc<SurfaceBevelPlan>],
        vertex: i32,
    ) -> Option<Vec<SurfaceMixedJunctionChainEnd>> {
        let (ends, touching) = Self::gather_the_ends(plans, vertex);
        (touching == 3 && ends.len() == 3).then_some(ends)
    }

    /// C# `TryGatherTheEnds`' walk: the free chain ends standing at `vertex`, and how many picked
    /// edge ends stand there however they are chained - which is what tells a CROWDED corner from
    /// one of three that simply does not end there, the two being refused under different names.
    fn gather_the_ends(
        plans: &[Rc<SurfaceBevelPlan>],
        vertex: i32,
    ) -> (Vec<SurfaceMixedJunctionChainEnd>, i32) {
        let mut ends = Vec::with_capacity(3);
        let mut touching = 0;

        for (p, plan) in plans.iter().enumerate() {
            let planned = &plan.edges;

            for (edge, plan_edge) in planned.iter().enumerate() {
                if plan_edge.edge.from_vertex() == vertex {
                    touching += 1;

                    if edge == 0 && !plan.is_closed {
                        ends.push(SurfaceMixedJunctionChainEnd::new(
                            p as i32,
                            edge as i32,
                            true,
                        ));
                    }
                }

                if plan_edge.edge.to_vertex() == vertex {
                    touching += 1;

                    if edge == planned.len() - 1 && !plan.is_closed {
                        ends.push(SurfaceMixedJunctionChainEnd::new(
                            p as i32,
                            edge as i32,
                            false,
                        ));
                    }
                }
            }
        }

        (ends, touching)
    }

    /// The plan edge a chain end names.
    pub fn edge_of(
        plans: &[Rc<SurfaceBevelPlan>],
        end: SurfaceMixedJunctionChainEnd,
    ) -> &SurfaceEdgePlan {
        &plans[end.plan as usize].edges[end.edge as usize]
    }
}

impl SurfaceMixedJunction {
    /// C# `TryReadTheArm`: one cut's side of the junction - which edge of its chain the cut really
    /// is, the chords the blend swallows before it, and the mixed corner it makes with the bead.
    /// The error is C#'s nullable failure reason: `None` where the cut could not be found, which
    /// the caller answers with its own default sentence.
    pub fn try_read_the_arm(
        plans: &[Rc<SurfaceBevelPlan>],
        end: SurfaceMixedJunctionChainEnd,
        bead_end: SurfaceMixedJunctionChainEnd,
        corner: Vector3,
        size: f64,
        near_enough: f64,
    ) -> Result<SurfaceMixedJunctionArm, Option<BevelAdvisory>> {
        let bead = Self::edge_of(plans, bead_end);
        let planned = &plans[end.plan as usize].edges;
        let is_closed = plans[end.plan as usize].is_closed;

        // The chain ARRIVES at the corner where the corner is its finish, which is which way the
        // walk away from the corner runs and which side of the touching chord its own neighbour is
        // on.
        let cut_is_arriving = !end.at_the_start;

        let (mut cut_index, mut swallowed) = surface_mixed_corner_cut_edge::try_find_for_the_bead(
            planned,
            is_closed,
            end.at_the_start,
            bead,
            size,
            near_enough,
        )
        .ok_or(None)?;

        let mut mixed = SurfaceMixedCorner::try_read(
            &planned[cut_index as usize],
            bead,
            corner,
            size,
            near_enough,
        )
        .map_err(Some)?;

        // And WHERE this arm hands over: on the chord that touches, or on the mitre ring just
        // before it where that chord has no band left to carry a ring - the same reading a mixed
        // corner INSIDE a chain makes, because the question is the chord's and not the corner's.
        let (carries_the_ring, ring_stops_at) =
            surface_mixed_corner_hands_over_at_the_mitre::try_read(
                planned,
                is_closed,
                cut_index as usize,
                cut_is_arriving,
                &mixed,
            );

        if ring_stops_at.is_some() {
            let touching = &planned[cut_index as usize];
            swallowed.push(cut_index);
            cut_index = carries_the_ring as i32;
            mixed = SurfaceMixedCorner::try_read_carried(
                &planned[carries_the_ring],
                touching,
                ring_stops_at,
                touching,
                bead,
                corner,
                size,
                near_enough,
            )
            .map_err(Some)?;
        }

        Ok(SurfaceMixedJunctionArm::new(
            end.plan,
            cut_index,
            end.at_the_start,
            swallowed,
            Rc::new(mixed),
            bead_end,
            None,
            None,
        ))
    }

    /// C# `TryRead`: reads the junction three chain ends make at `vertex` (standing at `corner`),
    /// or says why it cannot be read - always named AT the corner. Pure: it asks the plans and
    /// nothing else. `near_enough` is the part's own ruler (`SurfaceVertexPool::near_enough`).
    pub fn try_read(
        plans: &[Rc<SurfaceBevelPlan>],
        vertex: i32,
        corner: Vector3,
        near_enough: f64,
    ) -> Result<SurfaceMixedJunction, BevelAdvisory> {
        // The default answer, which every refusal below that has nothing better to say falls back
        // to: the edges at this corner do not read as two cuts and a bead about one axis. FOUR or
        // more of them is named by the count, because a user who picked a crowded vertex is told
        // the number rather than that something could not be read.
        let not_read = || refusal(BevelAdvisoryCode::SurfaceMixedJunctionNotRead, corner);

        let (ends, touching) = Self::gather_the_ends(plans, vertex);
        if !(touching == 3 && ends.len() == 3) {
            if touching > 3 {
                return Err(refusal(
                    BevelAdvisoryCode::SurfaceJunctionOfFourOrMoreEdges,
                    corner,
                ));
            }

            return Err(not_read());
        }

        let creases: Vec<usize> = (0..ends.len())
            .filter(|&end| Self::edge_of(plans, ends[end]).convexity() == EdgeConvexity::Concave)
            .collect();

        if creases.is_empty() {
            // No crease at all: that is the ball's own octant, which is built elsewhere and would
            // be answered wrongly rather than refused if it fell through to here.
            return Err(not_read());
        }

        if creases.len() > 1 {
            // Two creases at one vertex is a corner read by its own reading, which answers the two
            // arms it really has. THREE creases never arrives here (TryReadAll passes over a
            // vertex whose tally has no convex arm), so the count is dead defence, kept so that
            // the hand-over states the shape it answers.
            if creases.len() != 2 {
                return Err(refusal(
                    BevelAdvisoryCode::SurfaceJunctionOfTwoCreases,
                    corner,
                ));
            }

            return surface_two_crease_junction::try_read(
                plans,
                vertex,
                corner,
                near_enough,
                &ends,
                &creases,
            );
        }

        let concave = creases[0];
        let bead = ends[concave];

        // A CHAMFER before any arm is read: the whole junction is one sweep of the rolling ball
        // about the bead's axis, and a straight profile has no ball and no centre line to swing
        // about. Named here, where the profile is what is wrong.
        for &end in &ends {
            if !Self::edge_of(plans, end).has_center_line {
                return Err(refusal(
                    BevelAdvisoryCode::SurfaceMixedJunctionOfChamfers,
                    corner,
                ));
            }
        }

        let mut arms = Vec::with_capacity(2);
        for (end, &chain_end) in ends.iter().enumerate() {
            if end == concave {
                continue;
            }

            let arm = Self::try_read_the_arm(
                plans,
                chain_end,
                bead,
                corner,
                plans[bead.plan as usize].size,
                near_enough,
            )
            .map_err(|reason| reason.unwrap_or_else(not_read))?;
            arms.push(arm);
        }

        let second = arms.pop().expect("two cuts were read");
        let first = arms.pop().expect("two cuts were read");

        // One cap: both cuts come down from the same face, and each shares a DIFFERENT one with
        // the bead. Anything else is three edges that happen to meet rather than this corner.
        if Self::cap_key_of(&first.mixed) != Self::cap_key_of(&second.mixed)
            || first.mixed.shared_region_key == second.mixed.shared_region_key
        {
            return Err(not_read());
        }

        // And the two arms have to be reading ONE ball: the same turn, and the ball the first
        // hands over at is the ball the second starts from.
        if (first.mixed.turn_radians() - second.mixed.turn_radians()).abs() > Self::SAME_TURN
            || (first.mixed.ball_centre_at(first.mixed.turn_radians())
                - second.mixed.ball_centre_at(0.0))
            .length()
                > near_enough
        {
            return Err(not_read());
        }

        let cap_region_key = Self::cap_key_of(&first.mixed);
        Ok(SurfaceMixedJunction::new(
            corner,
            vertex,
            SurfaceJunctionKind::OneCrease,
            first,
            second,
            cap_region_key,
        ))
    }

    /// C# `TryReadAll`: every junction of a pass, read and handed to the plans that meet at it -
    /// or the first vertex where three or more picked edges of mixed convexity meet and CANNOT be
    /// read as one, named there. Vertices are gathered by the welded mesh's own index and walked
    /// in the plans' own order, so a part with two such corners always names the same one first.
    ///
    /// **The plans are written to as this goes, and a later refusal leaves the earlier ones
    /// written.** That is the contract: the builder aborts the whole pass on a refusal here, and a
    /// plan is not published anywhere a refused pass can be seen from.
    pub fn try_read_all(
        plans: &[Rc<SurfaceBevelPlan>],
        near_enough: f64,
    ) -> Result<Vec<SurfaceMixedJunction>, BevelAdvisory> {
        let mut junctions = Vec::new();

        // (edges, convex, concave) meeting at each vertex.
        let mut meeting: HashMap<i32, (i32, i32, i32)> = HashMap::new();
        let mut tally = |vertex: i32, convexity: EdgeConvexity| {
            let so_far = meeting.entry(vertex).or_default();
            so_far.0 += 1;
            so_far.1 += i32::from(convexity == EdgeConvexity::Convex);
            so_far.2 += i32::from(convexity == EdgeConvexity::Concave);
        };

        for plan in plans {
            for edge in &plan.edges {
                tally(edge.edge.from_vertex(), edge.convexity());
                tally(edge.edge.to_vertex(), edge.convexity());
            }
        }

        let mut read = HashSet::new();
        for plan in plans {
            for edge in &plan.edges {
                for (vertex, corner) in [
                    (edge.edge.from_vertex(), edge.from()),
                    (edge.edge.to_vertex(), edge.to()),
                ] {
                    let (edges, convex, concave) = meeting[&vertex];
                    if edges <= 2 || convex == 0 || concave == 0 || !read.insert(vertex) {
                        continue;
                    }

                    let junction = Self::try_read(plans, vertex, corner, near_enough)?;
                    junction.try_apply_to(plans)?;
                    junctions.push(junction);
                }
            }
        }

        Ok(junctions)
    }

    /// C# `TryApplyTo`: hands the junction to the three plans that meet at it, so that the
    /// walkback and the strip ends read one corner rather than three loose chain ends. The bead is
    /// given BOTH arms' readings, and each cut its own: which arm answers on the bead is the face's
    /// own question (`SurfaceBevelPlan::band_ends_of`).
    pub fn try_apply_to(&self, plans: &[Rc<SurfaceBevelPlan>]) -> Result<(), BevelAdvisory> {
        if self.kind == SurfaceJunctionKind::TwoCreases {
            // One cut and two beads, so the roles are the other way about: the CUT is the chain
            // both readings bound - both its faces are shared, one with each crease - and each
            // crease is bounded by its own arm alone. The two arms are the same cut, so it is
            // recorded once.
            plans[self.first.plan as usize].try_record_junction(
                self.first.corner_at_the_start,
                &[self.first.mixed.clone(), self.second.mixed.clone()],
                self.first.cut_index,
                &self.first.swallowed,
            )?;

            for arm in [&self.first, &self.second] {
                plans[arm.bead_end.plan as usize].try_record_junction(
                    arm.bead_end.at_the_start,
                    std::slice::from_ref(&arm.mixed),
                    arm.bead_end.edge,
                    &arm.bead_swallowed,
                )?;
            }

            return Ok(());
        }

        for arm in [&self.first, &self.second] {
            plans[arm.plan as usize].try_record_junction(
                arm.corner_at_the_start,
                std::slice::from_ref(&arm.mixed),
                arm.cut_index,
                &arm.swallowed,
            )?;
        }

        plans[self.bead_plan() as usize].try_record_junction(
            self.bead_corner_at_the_start(),
            &[self.first.mixed.clone(), self.second.mixed.clone()],
            self.bead_index(),
            &[],
        )
    }
}

/// A refusal of the junction's reading, which every code here words at the corner.
fn refusal(code: BevelAdvisoryCode, at: Vector3) -> BevelAdvisory {
    BevelAdvisory::new(code, vec![BevelAdvisoryArg::Vector3(at)])
}
