//! THE SECOND PASS OF A MIXED BEVEL, with its [`BevelSecondPassRun`]: round the convex edges first,
//! then read every selected concave crease AGAIN on the part as the cut left it, and roll one bead
//! along the whole of each - over the rounds and all - instead of asking a boolean to settle a bead
//! against the round it runs into. Each merged run is planned by [`BevelSecondPassRunPlan`] and
//! swept by [`bevel_group_tools::try_build`].
//!
//! **Why the crease has to be read twice.** A crease that ran into a sharp ridge stops there; once
//! that ridge is ROUNDED the same crease carries on across the round's facets, because the round is
//! tangent to the faces the crease lay on. Where two selected creases died on the same ridge from
//! opposite sides, the round makes them ONE crease over it. That is the surface a CAD package rolls
//! a fillet along, and it is not the surface two straight beads and a union make: those meet nearly
//! tangentially at the ridge and the kernel shreds the join (measured at 0.325 mm of raw ridge, and a
//! 171 degree fold, on "Inner Right Sides and Center.mcx").
//!
//! **What is re-read, and what is not.** Only groups that are CONCAVE throughout and that the first
//! pass found eligible: a cutter has already been used by the time this runs, and a group that did
//! not fit has no tool in either pass. And of those, only the ones whose crease actually CHANGED - it
//! merged with another group's, or it stopped being straight - keep their second-pass bead. An
//! operation whose cut goes nowhere near its beads gets the first pass's answer unchanged, byte for
//! byte, which is what keeps every fixture without a mixed corner exactly where it was.
//!
//! **What the beads are built against.** The cut mesh, and everything the sweep reads is read there:
//! the corner voids, the surface labels, the partner runs and the face run-outs. That is the whole
//! point of the pass - the faces a crease dies against near a rounded ridge are the round's own
//! tangent lines, which exist on no other mesh - and it is why these beads go on the part WHOLE rather
//! than as `bead - the cut region`. They were built on a part the cut had already been taken out of,
//! so there is nothing of the cut left in them to put back.

// The negated comparisons are written that way on purpose: each is false for NaN, which is the
// point.
#![allow(clippy::neg_cmp_op_on_partial_ord)]

use std::rc::Rc;
use std::sync::Arc;

use indexmap::IndexSet;
use polygon_mesh::{CancelToken, Mesh};
use vector_math::{math_helper, Vector3};

use super::bevel_face_fit::DEFAULT_SURFACE_TOLERANCE_DEGREES;
use super::bevel_group_tools::{self, BevelGroupToolsOptions};
use super::level_line::WorkError;
use super::{bevel_crease_merge, bevel_partnered_bead_partners};
use super::{
    BevelAdvisory, BevelAdvisoryCode, BevelCornerVoids, BevelFaceRunOut, BevelRoundFaces,
    BevelSecondPassRunPlan, BevelSurfaceLabels, BevelTool,
};
use crate::topology::bevel_edge_group_resolver;
use crate::topology::{
    BevelEdgeGroup, BevelEdgeGroupResolution, EdgeConvexity, FeatureEdge, FeatureEdgeChain,
    FeatureEdgeSet, MatchTolerances,
};

/// One concave crease read off the CUT part and swept as one bead, together with the rows of the
/// user's group list it was built for - which is more than one wherever the round joined two of
/// their creases into one.
#[derive(Debug, Clone)]
pub struct BevelSecondPassRun {
    /// The resolutions' indices of every group whose crease this run carries, in order. More than
    /// one means the round joined their creases: the first is the row the run was resolved from, and
    /// the rest are told on their own rows that they were built as one run with it.
    pub groups: Vec<usize>,
    /// What the sweep built - a bead, and its void wedge where it has one. Never a cutter: this pass
    /// only ever re-beads what the first pass already cut.
    pub tools: Vec<BevelTool>,
    /// How many mesh edges the crease came to on the cut part, which is what tells a row how much
    /// longer its crease became once the round was taken out of the ridge it died on.
    pub edge_count: i32,
}

/// What [`rebead`] hands back besides its runs.
#[derive(Debug, Clone, Default)]
pub struct BevelSecondPassRebead {
    /// One per crease this pass re-beaded.
    pub runs: Vec<BevelSecondPassRun>,
    /// Which groups this pass took over, by resolution index. A group not in it keeps the bead the
    /// first pass built, and the caller must leave that bead alone. In the order they were taken.
    pub rebuilt: IndexSet<usize>,
    /// The groups whose crease was re-read but whose bead would not sweep, with the reason. They are
    /// NOT in [`Self::rebuilt`]: a pass that could not build a replacement leaves the first pass's
    /// bead exactly where it was, because a fillet built along a crease the cut has since changed is
    /// still a better answer than no fillet at all. The caller notes the reason on the row.
    pub kept_first_pass: Vec<(usize, BevelAdvisory)>,
}

/// How far a crease may swing from one edge to the next and still be the same crease. The tangent
/// chain's own number: a crease crossing a round swings faster than the round's facets do where the
/// curvature is highest, and what says where a crease ENDS is the face tolerance below.
const DIRECTION_TOLERANCE_DEGREES: f64 = FeatureEdgeChain::DEFAULT_TANGENT_ANGLE_DEGREES;

/// The margin on the facet step, in degrees, for the arithmetic and for the half step the crease
/// takes where it leaves the flat face onto the round's first facet.
const FACET_STEP_MARGIN_DEGREES: f64 = 2.0;

/// How near the pick's own midpoint a re-read crease has to pass, as a fraction of the cut part's
/// diagonal. A thousandth: near enough that no crease elsewhere on the part can pass the test, and
/// loose enough for the accumulated float error of a boolean and a weld.
pub const ANCHOR_FRACTION_OF_THE_DIAGONAL: f64 = 1e-3;

/// How far a crease may turn from one edge to the next, in degrees - the widest facet step any
/// round in this operation can have, and a margin. A cut spans its edge's own dihedral in
/// `segments` facets, and the widest dihedral a convex edge can present is 180 degrees - so this is
/// the bound over every round the operation might have made, taken once rather than tracked per
/// cutter. It stays far below the turn a real corner makes (67 degrees at the tightest crease on
/// the nose-in-notch part, against 13.25 at 16 segments).
pub fn face_tolerance_degrees(segments: i32) -> f64 {
    (180.0 / f64::from(segments.max(1))) + FACET_STEP_MARGIN_DEGREES
}

/// Re-reads the concave creases on the cut part and sweeps one bead along each.
///
/// `cut_mesh` is the part with every cutter already taken out of it, welded and repaired (or
/// `None`); `groups` the stored groups, in the resolutions' order and in the build's own space;
/// `resolutions` what the FIRST pass made of those groups, on the uncut part; `eligible` which of
/// them the first pass was willing to build a tool for. A cancel comes back as
/// [`WorkError::Cancelled`].
#[allow(clippy::too_many_arguments)]
pub fn rebead(
    cut_mesh: Option<&Arc<Mesh>>,
    groups: &[BevelEdgeGroup],
    resolutions: &[BevelEdgeGroupResolution],
    eligible: &[bool],
    round_profile: bool,
    size: f64,
    segments: i32,
    cancellation_token: &CancelToken,
) -> Result<BevelSecondPassRebead, WorkError> {
    let mut answer = BevelSecondPassRebead::default();

    let Some(cut_mesh) = cut_mesh.filter(|_| round_profile) else {
        // A chamfer's cut and its bead meet in the planar cutback the union of two half spaces
        // gives, which is the right answer already - there is no round for a crease to run onto.
        return Ok(answer);
    };

    let graph = Rc::new(cut_mesh.get_mesh_edge_graph());
    // The feature sets read so far, looked up by the flat angle's value.
    let mut by_flat_angle: Vec<(f64, Arc<FeatureEdgeSet>)> = Vec::new();
    let face_tolerance = face_tolerance_degrees(segments);

    // How near the pick a re-read crease has to pass to count as the SAME crease. Scaled off the part
    // rather than fixed, because the whole point of working in the operation's space is that a
    // millimetre there is a millimetre of the part the user is looking at, whatever the mesh's own
    // units were - and this is a "these are the same point" tolerance, so it is small.
    let anchor_tolerance =
        ANCHOR_FRACTION_OF_THE_DIAGONAL * cut_mesh.get_axis_aligned_bounding_box().size().length();

    // The creases as the cut part carries them, by the group that named each. `None` for a group
    // this pass has nothing to say about.
    let mut creases: Vec<Option<BevelEdgeGroupResolution>> = vec![None; resolutions.len()];

    for i in 0..resolutions.len() {
        if cancellation_token.is_cancelled() {
            return Err(WorkError::Cancelled);
        }

        if !eligible[i] || !is_all_concave(&resolutions[i]) {
            continue;
        }

        let flat_angle = groups[i].flat_angle_degrees;
        let features = match by_flat_angle
            .iter()
            .find(|(angle, _)| same_key(*angle, flat_angle))
        {
            Some((_, features)) => features.clone(),
            None => {
                let features = Arc::new(FeatureEdgeSet::create_with_graph(
                    cut_mesh, &graph, flat_angle,
                ));
                by_flat_angle.push((flat_angle, features.clone()));
                features
            }
        };

        // WITH THE AMBIGUITY GATE OPEN. The gate exists because picking the wrong one of two equally
        // good candidates is worse than failing - on a comb with the anchored tooth deleted, both
        // survivors score the same and neither is the answer. Here the two candidates are the pieces
        // the CUT made of the one crease the anchor names: the round trims its end and the tangency
        // splits what is left, so the runners-up are the crease's own neighbours, and the walk that
        // follows takes the whole crease from whichever of them wins. Measured on "Inner Right Sides
        // and Center.mcx", whose upper crease comes back AnchorMatchesMoreThanOneEdge with the gate
        // shut - and whose bead is then built twice, once by this pass along the merged crease and
        // once by the first pass along the piece this one could not name.
        let crease = bevel_edge_group_resolver::resolve_crease(
            &groups[i],
            features,
            DIRECTION_TOLERANCE_DEGREES,
            face_tolerance,
            MatchTolerances {
                ambiguity_ratio: 1.0,
                ..MatchTolerances::default()
            },
        );

        // AND CHECKED AGAINST THE PICK IT CAME FROM. The gate above is open, so the resolver is
        // allowed to settle a tie - and a tie settled the wrong way is a bead rolled along a crease
        // somewhere else on the part, in place of one the first pass had already built correctly.
        // So the winner is only taken if it is demonstrably the user's own crease: it either runs
        // through the pick's own midpoint, or it carries one of the very edges the first pass
        // resolved. A crease the round trimmed still satisfies both; a crease from another feature
        // satisfies neither, and that group keeps the bead it already has.
        let names_it = crease.is_resolved()
            && crease.chain.as_ref().is_some_and(|chain| {
                chain.count() > 0
                    && names_the_same_crease(chain, &groups[i], &resolutions[i], anchor_tolerance)
            });

        if names_it {
            creases[i] = Some(crease);
        }
    }

    // One run per distinct crease, keyed on the MESH edges each runs over and merged transitively -
    // see BevelCreaseMerge for why neither of those is the obvious thing to do.
    let mesh_edges: Vec<Option<Vec<i32>>> = creases
        .iter()
        .map(|crease| {
            crease.as_ref().map(|crease| {
                crease.chain.as_ref().map_or_else(Vec::new, |chain| {
                    chain
                        .edges()
                        .iter()
                        .map(|edge| edge.edge.mesh_edge_index)
                        .collect()
                })
            })
        })
        .collect();

    let members = bevel_crease_merge::merge(&mesh_edges);
    let mut plans = Vec::with_capacity(members.len());

    for run in &members {
        // The run is named by the UNION of its members' readings, not by the first of them. Two
        // groups the merge put together need only OVERLAP - BevelCreaseMerge joins A to B and B to
        // C with A and C sharing nothing - so the first member's reading can be a proper part of
        // what the run has to carry, and sweeping it alone would take C's first-pass bead off the
        // batch and put nothing where it was. The merge only ever puts a group with a reading in a
        // run, so a plan always comes back.
        plans.push(BevelSecondPassRunPlan::for_members(
            run,
            &creases,
            &mesh_edges,
        ));
    }

    // Which of them are worth taking over, decided before anything is swept because the readings
    // below are made across the runs that ARE taken over and would otherwise be measured against
    // creases this pass is about to hand back.
    let mut taken: Vec<&BevelSecondPassRunPlan> = Vec::new();

    for (run, plan) in plans.iter().enumerate() {
        let Some(plan) = plan else {
            continue;
        };

        if members[run].len() > 1
            || the_round_changed_it(&plan.crease, &resolutions[members[run][0]])
        {
            taken.push(plan);
        }
    }

    if taken.is_empty() {
        return Ok(answer);
    }

    let taken_resolutions: Vec<BevelEdgeGroupResolution> =
        taken.iter().map(|plan| plan.crease.clone()).collect();
    let all_eligible = vec![true; taken_resolutions.len()];

    // The same cross-run readings the first pass makes, made on the CUT part: what a bead may run
    // out onto, which runs bound each other, what the surface either side of a corner is. Read here
    // so every run is handed one reading rather than one each.
    let concave_runs = bevel_partnered_bead_partners::concave_runs_across_groups(
        &taken_resolutions,
        Some(&all_eligible),
    );

    // AND THE ROUNDS THE FIRST PASS CUT, which is the one reading this pass makes that the first
    // pass has no use for. A crease re-read here crosses a round, and each of its mesh edges is
    // shared with one planar FACET of it - a section inscribed in that facet's plane, drawn square to
    // the crease chord, is neither on the round nor square to the rolling ball's own spine, and left
    // 0.113 mm on the nose-in-notch fixture at a radius of 2. See BevelRoundFaces, and
    // BevelChainSweep's TryStandOnTheRound for what the sweep does with it. It is read from the FIRST
    // pass's resolutions, on the uncut part, because only they say where each cutter's arc was
    // centred. The profile is handed over as well, though this pass has already turned back for a
    // chamfer above: the reading and both of its readers are then gated on one flag rather than
    // three.
    let rounds = BevelRoundFaces::create(
        cut_mesh,
        resolutions,
        eligible,
        size,
        segments,
        round_profile,
    );
    let run_out = Rc::new(BevelFaceRunOut::create(
        cut_mesh.clone(),
        Some(graph.clone()),
        DEFAULT_SURFACE_TOLERANCE_DEGREES,
    ));
    let corner_voids = BevelCornerVoids::new(cut_mesh.clone(), Some(graph.clone()));
    let mut surface_labels = BevelSurfaceLabels::new(cut_mesh, Some(&graph));

    let selected_across_runs: Vec<FeatureEdge> = taken
        .iter()
        .flat_map(|plan| plan.edges.iter().copied())
        .collect();

    for plan in taken {
        if cancellation_token.is_cancelled() {
            return Err(WorkError::Cancelled);
        }

        // The members this run's swept reading does NOT cover keep the bead they have, whatever
        // becomes of the sweep. They are only here at all because the readings would not join into
        // one chain, which is the run being unable to answer for them rather than the sweep failing.
        for &group in &plan.not_covered {
            answer.kept_first_pass.push((
                group,
                BevelAdvisory::new(BevelAdvisoryCode::SecondPassCreaseDidNotJoin, vec![]),
            ));
        }

        let built = bevel_group_tools::try_build(
            &plan.crease,
            round_profile,
            size,
            segments,
            BevelGroupToolsOptions {
                selected_across_groups: Some(&selected_across_runs),
                corner_voids: Some(&corner_voids),
                surface_labels: Some(&mut surface_labels),
                concave_runs_across_groups: Some(&concave_runs),
                run_out: Some(run_out.clone()),
                linked_runs: plan.linked_runs.as_deref(),
                rounds: Some(&rounds),
                ..BevelGroupToolsOptions::default()
            },
        );

        let tools = match built {
            Ok(built) => built.tools,
            Err(refusal) => {
                // NOT marked rebuilt. Nothing was built to put in its place, so withdrawing the
                // first pass's bead here would answer a crease the cut changed with no fillet at all
                // - strictly worse than the fillet that is already there, which at least follows
                // most of it.
                for &group in &plan.covered {
                    answer.kept_first_pass.push((group, refusal.reason.clone()));
                }

                continue;
            }
        };

        // Marked rebuilt only where the bead that was just built really does run along that member's
        // own crease - which is every member of a run whose readings joined, and only the covered
        // ones where they did not.
        answer.runs.push(BevelSecondPassRun {
            groups: plan.covered.clone(),
            tools,
            edge_count: plan.edge_count,
        });

        answer.rebuilt.extend(plan.covered.iter().copied());
    }

    Ok(answer)
}

/// Key equality by value: equal values, with NaN equal to NaN.
fn same_key(a: f64, b: f64) -> bool {
    a == b || (a.is_nan() && b.is_nan())
}

/// Whether a crease re-read on the CUT part is the crease the group's pick names, rather than some
/// other crease the resolver settled a tie in favour of. `tolerance` is how near counts as through,
/// in the build's own units - see [`ANCHOR_FRACTION_OF_THE_DIAGONAL`].
///
/// Two ways of being the same, and a crease need only be one of them. It may run through the pick's
/// own midpoint - which is the pick, so nothing more is needed. Or it may carry one of the edges the
/// FIRST pass resolved, which is what a crease whose midpoint the round has since cut away looks
/// like: the pick is gone, but the run it was picked out of is still there and still selected.
pub fn names_the_same_crease(
    chain: &FeatureEdgeChain,
    group: &BevelEdgeGroup,
    before: &BevelEdgeGroupResolution,
    tolerance: f64,
) -> bool {
    for edge in chain.edges() {
        if distance_to_segment(group.anchor_midpoint, edge.from(), edge.to()) <= tolerance {
            return true;
        }

        for was in &before.edges {
            // Compared as the two ENDS of a segment rather than by mesh edge index: the first pass
            // read the uncut part and this pass reads the cut one, so the same edge of the same
            // surface has a different index in each.
            if ((edge.from() - was.start).length() <= tolerance
                && (edge.to() - was.end).length() <= tolerance)
                || ((edge.from() - was.end).length() <= tolerance
                    && (edge.to() - was.start).length() <= tolerance)
            {
                return true;
            }
        }
    }

    false
}

/// How far a point lies from a segment, which is what "the crease runs through the pick" is.
fn distance_to_segment(point: Vector3, from: Vector3, to: Vector3) -> f64 {
    let along = to - from;
    let length_squared = along.length_squared();

    if !(length_squared > 0.0) {
        return (point - from).length();
    }

    // f64::clamp passes a NaN through.
    let t = ((point - from).dot(along) / length_squared).clamp(0.0, 1.0);

    (point - (from + (along * t))).length()
}

/// Whether the cut changed this crease into something the first pass's bead does not describe - it
/// bends, which is a crease that has run onto a round. A crease that is still the straight run it
/// was keeps its first-pass bead, whatever else the cut did elsewhere on the part.
fn the_round_changed_it(
    crease: &BevelEdgeGroupResolution,
    before: &BevelEdgeGroupResolution,
) -> bool {
    let edges = crease.chain.as_ref().map_or(&[][..], |chain| chain.edges());

    if edges.len() != before.edges.len() {
        return true;
    }

    let Some(first) = edges.first().map(|edge| edge.direction()) else {
        return false;
    };

    edges[1..]
        .iter()
        .any(|edge| first.dot(edge.direction()).abs() < collinear_dot())
}

/// How straight the edges of a run have to be to each other to be the straight crease the first
/// pass swept - `FeatureEdgeChain::default_collinear_angle_degrees` as a dot product.
fn collinear_dot() -> f64 {
    math_helper::degrees_to_radians(FeatureEdgeChain::default_collinear_angle_degrees()).cos()
}

/// Whether the group resolved to edges and every one of them is concave.
fn is_all_concave(resolution: &BevelEdgeGroupResolution) -> bool {
    if !resolution.is_resolved() || resolution.edges.is_empty() {
        return false;
    }

    resolution
        .edges
        .iter()
        .all(|edge| edge.convexity == EdgeConvexity::Concave)
}
