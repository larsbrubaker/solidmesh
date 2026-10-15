//! What a run does at each of its two ends: [`BevelChainEnds`], the decision `BevelEndCap` records
//! one half of: its constructor, the shared overshoot answer, the squared-off ends and the
//! resolution of both ends ([`BevelChainEnds::resolve`]). The plane an end is capped on is
//! [`super::bevel_chain_end_plane`]; what an end can afford (the bead's square cap and the
//! overshoot bound) is [`super::bevel_chain_end_budget`].
//!
//! A cutter that finds a face bounding its end vertex stops on that face's plane; one that finds
//! none overshoots, which costs nothing because what it reaches is material a neighbouring tool
//! takes anyway, or air. A BEAD that finds none squares off instead, because every bit of tool
//! hanging past the last vertex is material ADDED where the user selected nothing - except at a
//! 5.3 corner, where the overshoot is the mechanism: the tools overlap and their union resolves
//! the corner. Only ONE plane caps an end; where several faces bound a corner the most square to
//! the run is used.

use super::BevelEndCap;
use crate::topology::ChainEdge;

/// The end treatment for one run of a group's selection.
///
/// `Default` is [`BevelChainEnds::overshoot`]: both ends run past the last vertex, unbounded - what
/// a closed loop and a run with nothing to cap against get.
#[derive(Debug, Clone, PartialEq)]
pub struct BevelChainEnds {
    /// The faces the run leaves the part through at a START that had to square off, as planes
    /// the bead may not reach past (`BevelBeadExit`). `None` at every end that capped, overshot,
    /// or is a cutter's.
    pub start_exit_planes: Option<Vec<BevelEndCap>>,
    /// As [`BevelChainEnds::start_exit_planes`], at the run's end.
    pub end_exit_planes: Option<Vec<BevelEndCap>>,
    /// What happens where the run begins.
    pub start: BevelEndCap,
    /// What happens where the run ends.
    pub end: BevelEndCap,
    /// The vertex where this run's START deliberately overshoots into another selected run of
    /// its own convexity across an unselected CONCAVE crease between them
    /// (`BevelPairCornerCrease`) - 5.3's PAIR CORNER - or `None` where it does not. Decided here
    /// and nowhere else; `BevelPairCornerTrim` builds the solid that follows from it.
    pub start_pair_corner: Option<i32>,
    /// As [`BevelChainEnds::start_pair_corner`], at the run's end.
    pub end_pair_corner: Option<i32>,
    /// The vertex this run's START runs PAST because another selected run of its own convexity
    /// departs from it - 5.3's corner read at its widest, recognised crease or not - or `None`.
    /// Only an UNCAPPED end fills it in. What reads it is the bead's void wedge
    /// (`BevelGroupTools`), which is squared off there while the vertex's own void cone carries
    /// the corner.
    pub start_overshoot_corner: Option<i32>,
    /// As [`BevelChainEnds::start_overshoot_corner`], at the run's end.
    pub end_overshoot_corner: Option<i32>,
    /// How far past its own last vertex this run's START may reach when it is uncapped: half the
    /// segment adjoining the end, so the spill can never be more than half of what the user
    /// picked there - infinity (the overshoot unbounded) at a 5.3 corner, which is exempt and
    /// must be, and wherever no bound applies.
    pub start_maximum_overshoot: f64,
    /// As [`BevelChainEnds::start_maximum_overshoot`], at the run's end.
    pub end_maximum_overshoot: f64,
}

impl Default for BevelChainEnds {
    fn default() -> Self {
        // Nothing read yet: no pair or overshoot corner, no exit planes, and both overshoots
        // unbounded.
        Self {
            start_exit_planes: None,
            end_exit_planes: None,
            start: BevelEndCap::default(),
            end: BevelEndCap::default(),
            start_pair_corner: None,
            end_pair_corner: None,
            start_overshoot_corner: None,
            end_overshoot_corner: None,
            start_maximum_overshoot: f64::INFINITY,
            end_maximum_overshoot: f64::INFINITY,
        }
    }
}

impl BevelChainEnds {
    /// The steepest a cap plane may stand off square to the run, in degrees.
    pub const MAXIMUM_CAP_PLANE_ANGLE_DEGREES: f64 = 80.0;

    /// The two ends' caps, with everything else at its default: no pair or overshoot corner, no
    /// exit planes, and an unbounded overshoot at both ends. Callers that name an optional argument
    /// set it with struct-update syntax.
    pub fn new(start: BevelEndCap, end: BevelEndCap) -> Self {
        BevelChainEnds {
            start,
            end,
            ..BevelChainEnds::default()
        }
    }

    /// Both ends overshooting: no cap plane at either.
    pub fn overshoot() -> Self {
        Self::new(BevelEndCap::default(), BevelEndCap::default())
    }

    /// These ends with the run's start and/or end squared off - capped on the plane square to the
    /// run through its first (or last) vertex - and everything else kept.
    pub fn squared_at(&self, run: &[ChainEdge], at_start: bool, at_end: bool) -> BevelChainEnds {
        if !at_start && !at_end {
            return self.clone();
        }

        let first = &run[0];
        let last = &run[run.len() - 1];

        let out_at_start = -first.direction();
        let out_at_end = last.direction();

        BevelChainEnds {
            start: if at_start {
                BevelEndCap::new(out_at_start, out_at_start.dot(first.from()))
            } else {
                self.start.clone()
            },
            end: if at_end {
                BevelEndCap::new(out_at_end, out_at_end.dot(last.to()))
            } else {
                self.end.clone()
            },
            ..self.clone()
        }
    }
}

/// [`BevelChainEnds::resolve`]'s optional arguments, at their defaults by `Default` (round profile
/// on, everything else absent).
pub struct ResolveOptions<'a, 's> {
    /// Where a corner patch has put a ball in an OUTSIDE corner, by mesh vertex.
    pub patched_corners: Option<&'a indexmap::IndexMap<i32, vector_math::Vector3>>,
    /// The plane a blend hands a cut over on, by (mesh vertex, mesh edge).
    pub blend_handovers: Option<&'a indexmap::IndexMap<(i32, i32), BevelEndCap>>,
    /// The face normal a blend caps a bead on, by (mesh vertex, mesh edge).
    pub blend_capped_corners: Option<&'a indexmap::IndexMap<(i32, i32), vector_math::Vector3>>,
    /// Where a corner patch has put a ball out in the VOID of an inside corner, by mesh vertex.
    pub filled_corners: Option<&'a indexmap::IndexMap<i32, vector_math::Vector3>>,
    /// Whether the tool's profile is an arc.
    pub round_profile: bool,
    /// The surface label of each face, read only by the end that needs it.
    pub surface_of_face: Option<&'a mut dyn FnMut() -> Option<&'s [i32]>>,
    /// The patched vertices whose runs may drive straight through.
    pub runs_may_drive_through: Option<&'a std::collections::HashSet<i32>>,
    /// The points a corner patch has already put on each run's plane, by (vertex, mesh edge).
    pub patch_seam_rings: Option<&'a indexmap::IndexMap<(i32, i32), Vec<vector_math::Vector3>>>,
    /// Every concave run of the whole rebuild.
    pub concave_runs: Option<&'a [super::BevelSweptChain]>,
}

impl Default for ResolveOptions<'_, '_> {
    fn default() -> Self {
        ResolveOptions {
            patched_corners: None,
            blend_handovers: None,
            blend_capped_corners: None,
            filled_corners: None,
            round_profile: true,
            surface_of_face: None,
            runs_may_drive_through: None,
            patch_seam_rings: None,
            concave_runs: None,
        }
    }
}

impl BevelChainEnds {
    /// Both ends of an open run capped
    /// ([`resolve_end`](super::bevel_chain_end_plane::resolve_end)), each one's overshoot bounded,
    /// with the advisories for interrupted creases left short and ends held back. A closed run, an
    /// empty one or no feature set overshoots freely.
    #[allow(clippy::too_many_arguments)]
    pub fn resolve<'s>(
        run: &[ChainEdge],
        is_closed: bool,
        size: f64,
        features: Option<&crate::topology::FeatureEdgeSet>,
        selected_edges: Option<&[crate::topology::FeatureEdge]>,
        mut warnings: Option<&mut Vec<super::BevelAdvisory>>,
        mut options: ResolveOptions<'_, 's>,
    ) -> BevelChainEnds {
        use super::bevel_chain_end_plane::{resolve_end, EndHandovers};
        use super::{bevel_chain_end_budget as budget, bevel_interrupted_crease};
        use crate::topology::EdgeConvexity;

        let Some(features) = features else {
            return BevelChainEnds::overshoot();
        };
        if is_closed || run.is_empty() {
            return BevelChainEnds::overshoot();
        }

        let run_convexity = run[0].edge.convexity;
        let selected: std::collections::HashSet<i32> = selected_edges
            .unwrap_or(&[])
            .iter()
            .filter(|e| e.convexity == run_convexity)
            .map(|e| e.mesh_edge_index)
            .collect();

        let first = &run[0];
        let last = &run[run.len() - 1];
        let is_convex_run = run_convexity == EdgeConvexity::Convex;
        let corners = if is_convex_run {
            options.patched_corners
        } else {
            options.filled_corners
        };
        let lean_guard =
            vector_math::math_helper::degrees_to_radians(Self::MAXIMUM_CAP_PLANE_ANGLE_DEGREES)
                .cos();
        let start_minimum = if is_convex_run {
            lean_guard
        } else {
            vector_math::ieee_math::max_f64(
                lean_guard,
                budget::minimum_bead_cap_agreement(size, first),
            )
        };
        let end_minimum = if is_convex_run {
            lean_guard
        } else {
            vector_math::ieee_math::max_f64(
                lean_guard,
                budget::minimum_bead_cap_agreement(size, last),
            )
        };
        let start_reach = budget::reach(size, options.round_profile, first);
        let end_reach = budget::reach(size, options.round_profile, last);
        let handovers = EndHandovers {
            patched_corners: corners,
            blend_handovers: options.blend_handovers,
            blend_capped_corners: options.blend_capped_corners,
            runs_may_drive_through: options.runs_may_drive_through,
            patch_seam_rings: options.patch_seam_rings,
        };

        let start = resolve_end(
            first.from_vertex(),
            first.from(),
            -first.direction(),
            first.left_face(),
            first.right_face(),
            first.left_normal(),
            first.right_normal(),
            first.edge.mesh_edge_index,
            !is_convex_run,
            start_minimum,
            start_reach,
            features,
            &selected,
            warnings.as_deref_mut(),
            handovers,
            match options.surface_of_face {
                Some(ref mut ask) => Some(&mut **ask as &mut dyn FnMut() -> Option<&'s [i32]>),
                None => None,
            },
            options.concave_runs,
        );
        let end = resolve_end(
            last.to_vertex(),
            last.to(),
            last.direction(),
            last.left_face(),
            last.right_face(),
            last.left_normal(),
            last.right_normal(),
            last.edge.mesh_edge_index,
            !is_convex_run,
            end_minimum,
            end_reach,
            features,
            &selected,
            warnings.as_deref_mut(),
            handovers,
            match options.surface_of_face {
                Some(ref mut ask) => Some(&mut **ask as &mut dyn FnMut() -> Option<&'s [i32]>),
                None => None,
            },
            options.concave_runs,
        );

        let start_bound = budget::maximum_overshoot(
            &start.cap,
            start.overlaps_selected_run || start.interruption.runs_on(),
            size,
            options.round_profile,
            first,
        );
        let end_bound = budget::maximum_overshoot(
            &end.cap,
            end.overlaps_selected_run || end.interruption.runs_on(),
            size,
            options.round_profile,
            last,
        );

        let unspanned = bevel_interrupted_crease::short_of_the_far_side(
            start.interruption,
            end.interruption,
            size,
            options.round_profile,
            first,
            last,
        );
        if let Some(warnings) = warnings.as_mut() {
            warnings.extend(unspanned);
            if start_bound.1 || end_bound.1 {
                warnings.push(budget::held_back(
                    start_bound.1,
                    end_bound.1,
                    first,
                    last,
                    size,
                    run.len() as i32,
                ));
            }
        }

        let start_overshoot_corner =
            (!start.cap.is_capped() && start.overlaps_selected_run).then(|| first.from_vertex());
        let end_overshoot_corner =
            (!end.cap.is_capped() && end.overlaps_selected_run).then(|| last.to_vertex());

        BevelChainEnds {
            start_pair_corner: start.pair_corner_vertex,
            end_pair_corner: end.pair_corner_vertex,
            start_maximum_overshoot: start_bound.0,
            end_maximum_overshoot: end_bound.0,
            start_overshoot_corner,
            end_overshoot_corner,
            start_exit_planes: start.exit_planes,
            end_exit_planes: end.exit_planes,
            ..BevelChainEnds::new(start.cap, end.cap)
        }
    }
}
