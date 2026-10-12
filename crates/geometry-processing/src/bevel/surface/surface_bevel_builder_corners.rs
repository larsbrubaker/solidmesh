//! The corner loop of `SurfaceBevelBuilder.TryBuild` (`Bevel/Surface/SurfaceBevelBuilder.cs`),
//! split out of [`super::surface_bevel_builder`] to keep each file under the length limit: every
//! corner between two consecutive edges of a chain, closed as the construction it is - the
//! staircase a consumed chord stands on ([`super::surface_staircase_mitre`], built before the loop),
//! a MIXED corner ([`surface_builder_mixed_corners::try_close`]), the two staircases a run-out
//! crossing both strips leaves ([`super::SurfaceCrossingAtTheMitre`]), or the ordinary mitre ring
//! ([`surface_strip_mitre::try_mitre`]).

use std::cell::OnceCell;
use std::rc::Rc;

use super::surface_bevel_builder::{is_swallowed, plan_of};
use super::surface_mitre_joint_step::{SideBandCorners, SideSeams};
use super::surface_strip_mitre::SurfaceStripMitrePair;
use super::surface_strip_trimmed_at_the_crossing::SurfaceStripTrims;
use super::{
    surface_builder_mixed_corners, surface_staircase_mitre, surface_strip_mitre, SurfaceBevelPlan,
    SurfaceEdgePlan, SurfaceMitreConsumesTheRail, SurfaceRoomCrossing, SurfaceStrip,
    SurfaceStripEndsOnTheHullSupportingPlane, SurfaceStripStation,
    SurfaceStripTrimmedAtTheCrossing, SurfaceVertexPool,
};
use crate::bevel::{BevelAdvisory, BevelAdvisoryCode};
use crate::topology::FaceRegionCache;

/// Everything the corner loop reads and does not change.
pub struct CornerLoopInputs<'a> {
    /// The region cache the plans were made with.
    pub regions: &'a FaceRegionCache,
    /// The pass's plans.
    pub plans: &'a [Rc<SurfaceBevelPlan>],
    /// The first edge of each plan under the one numbering.
    pub first_of_plan: &'a [i32],
    /// Every chain's edges, end to end.
    pub edges: &'a [SurfaceEdgePlan],
    /// One strip per edge, `None` where the blend swallows it.
    pub strips: &'a [Option<Rc<SurfaceStrip>>],
    /// The seams, `[edge][side]`.
    pub seams: &'a [SideSeams],
    /// The band corners, `[edge][side][end]`.
    pub band_corners: &'a [SideBandCorners],
    /// The chords their own two mitres consumed.
    pub consumed: &'a [Option<SurfaceMitreConsumesTheRail>],
    /// The room's crossings of a band by a run-out.
    pub crossings: Option<&'a [SurfaceRoomCrossing]>,
    /// Lifts THE LEAN GATE (a test seam).
    pub lay_the_oblique_rows_anyway: bool,
    /// The part's hull planes, read once and only if asked (C#'s `theHull`).
    pub hull: &'a dyn Fn() -> Vec<SurfaceStripEndsOnTheHullSupportingPlane>,
}

/// The planes the part is bounded by, read ONCE and only if somebody asks (C#'s `hullPlanes ??=`
/// local function). Two readings want them - a strip whose band ran off part of its landing
/// (`SurfaceStripEndsOnTheHull`) and a mitre whose tip stands past the shared face's rim
/// (`SurfaceMitreOverTheFaceBeyond`) - and the second cannot say whether it wants them without a
/// cheap reading of its own, so the answer is handed down as something to ask rather than as
/// something already paid for.
pub fn the_hull_once<'a>(
    cell: &'a OnceCell<Vec<SurfaceStripEndsOnTheHullSupportingPlane>>,
    read: impl Fn() -> Vec<SurfaceStripEndsOnTheHullSupportingPlane> + 'a,
) -> impl Fn() -> Vec<SurfaceStripEndsOnTheHullSupportingPlane> + 'a {
    move || cell.get_or_init(&read).clone()
}

/// The consumed chords' staircases and then every corner of every chain, in C#'s two passes.
pub fn try_close_the_corners(
    pool: &mut SurfaceVertexPool,
    faces: &mut Vec<(i32, i32, i32)>,
    inputs: &CornerLoopInputs<'_>,
    trimmed: &mut SurfaceStripTrims,
    start_rings: &mut [Option<SurfaceStripStation>],
    end_rings: &mut [Option<SurfaceStripStation>],
) -> Result<(), BevelAdvisory> {
    let plans = inputs.plans;
    let first_of_plan = inputs.first_of_plan;
    let consumed = inputs.consumed;
    let count = inputs.edges.len() as i32;

    // A consumed chord's two corners are ONE construction and not two - they share the virtual
    // corner's vertices - so both of its rings are built here, and the corner loop below leaves the
    // two corners either side of it alone.
    for reading in consumed.iter().flatten() {
        surface_staircase_mitre::try_build(
            pool,
            inputs.strips,
            inputs.seams,
            reading,
            start_rings,
            end_rings,
        )?;
    }

    // TWICE over the same corners: every MIXED corner first, and only then the mitres. A mitre
    // whose band the run-out crossed is closed on the crossing rather than on a ring the two strips
    // share, and the crossing is made by the mixed corner - which can stand in a LATER plan than
    // the band does. One pass read the trims a plan at a time and found nothing there yet, so the
    // mitre built its ring and the crossing went unclosed, for no reason but the order the chains
    // were picked in.
    for pass in 0..2 {
        for (p, plan) in plans.iter().enumerate() {
            let in_chain = plan.edges.len();

            let mut i = 0;
            while i + 1 < in_chain || (plan.is_closed && i < in_chain) {
                let corner = i;
                i += 1;

                let here = (first_of_plan[p] + corner as i32) as usize;
                let next = (first_of_plan[p] + ((corner + 1) % in_chain) as i32) as usize;

                if consumed[here].is_some() || consumed[next].is_some() {
                    // Already standing on the staircase above. The reading that named it has
                    // excluded every other shape this corner could be - a mixed corner, a
                    // swallowed neighbour, a free end - so there is nothing else here to do.
                    continue;
                }

                // A corner the convexity turns over at is not a mitre at all: the two strips are
                // tangent at one point and what goes between them is a torus patch about the
                // bead's own axis (SurfaceMixedCorner): the cut's end ring carried round that axis,
                // with the two bites its footprint takes out of the faces nobody picked
                // (SurfaceMixedCornerPatch).
                if let Some(mixed) = plan.try_mixed_corner_after(corner as i32) {
                    if pass != 0 {
                        continue;
                    }

                    surface_builder_mixed_corners::try_close(
                        pool,
                        faces,
                        Some(inputs.regions),
                        plans,
                        first_of_plan,
                        inputs.edges,
                        inputs.strips,
                        inputs.seams,
                        count,
                        inputs.crossings,
                        p,
                        corner as i32,
                        here,
                        next,
                        &mixed,
                        inputs.lay_the_oblique_rows_anyway,
                        trimmed,
                        start_rings,
                        end_rings,
                    )?;

                    continue;
                }

                if pass == 0 {
                    continue;
                }

                if is_swallowed(plans, first_of_plan, here as i32)
                    || is_swallowed(plans, first_of_plan, next as i32)
                {
                    // One of the two has no strip, so there is nothing to mitre here: what stands
                    // between them is the blend, and it was closed at the corner that swallowed it.
                    continue;
                }

                // Where a run-out crossed BOTH of these two, the mitre is not one ring they meet
                // along: below the hand-over the run-out has taken the rails out of both, and each
                // strip stops at its own crossing with it (SurfaceCrossingAtTheMitre). Read off the
                // two TRIMS rather than off the strips, because a closed rim is mitred at every
                // corner and one band can be crossed at both of its ends - the hand-over belongs to
                // the END and not to the band.
                let leaving_here = SurfaceStripTrimmedAtTheCrossing::standing_at(
                    Some(trimmed),
                    here as i32,
                    false,
                );
                let arriving_next =
                    SurfaceStripTrimmedAtTheCrossing::standing_at(Some(trimmed), next as i32, true);

                let leaving_mitre = leaving_here
                    .as_ref()
                    .and_then(|trim| trim.at_the_mitre.borrow().clone());
                let arriving_mitre = arriving_next
                    .as_ref()
                    .and_then(|trim| trim.at_the_mitre.borrow().clone());

                if let (Some(at_the_mitre), Some(arriving_mitre)) =
                    (&leaving_mitre, &arriving_mitre)
                {
                    if Rc::ptr_eq(at_the_mitre, arriving_mitre) {
                        let (leaving_on, arriving_on) = at_the_mitre.try_the_staircases(
                            pool,
                            leaving_here.as_deref(),
                            arriving_next.as_deref(),
                        )?;

                        end_rings[here] = Some(leaving_on);
                        start_rings[next] = Some(arriving_on);

                        continue;
                    }
                }

                // The plan's own reading of where a CHORDED run's joint stands, asked of the plan
                // the corner belongs to, so the ring is cut on the very plane the band was.
                let mitres_plan = plan_of(first_of_plan, here as i32) as usize;

                let the_chorded_runs_joint = plans[mitres_plan]
                    .the_chorded_runs_joint_after(here - first_of_plan[mitres_plan] as usize);

                let (Some(strip), Some(next_strip)) = (
                    inputs.strips[here].as_deref(),
                    inputs.strips[next].as_deref(),
                ) else {
                    // A strip that is neither swallowed nor built is a contradiction C# would
                    // throw on; refused as the mitre that does not meet (divergence 5070).
                    return Err(BevelAdvisory::new(
                        BevelAdvisoryCode::SurfaceStripsDoNotMitre,
                        vec![inputs.edges[here].to().into()],
                    ));
                };

                let ring = surface_strip_mitre::try_mitre(
                    pool,
                    faces,
                    SurfaceStripMitrePair {
                        strip,
                        next: next_strip,
                        leaving: &inputs.edges[here],
                        arriving: &inputs.edges[next],
                        edge_index: here as i32,
                        next_index: next as i32,
                    },
                    inputs.seams,
                    Some(inputs.band_corners),
                    inputs.edges[here].to(),
                    the_chorded_runs_joint,
                    Some(inputs.hull),
                )?;

                // Two rings, each its own strip's - the same vertices wherever the two beads have
                // rails to pair, and the wider one carrying its own besides. The wider one's
                // station carries what the far face's own triangulation left along its unpaired
                // chords when that face gave up the sector (SurfaceMitreSector), exactly as an
                // ordinary end does. They are not the same parameters - each strip measures from
                // its own edge's start - and a stitch that used the wrong strip's would clamp to a
                // window that is not there.
                end_rings[here] = Some(SurfaceStripStation::new(
                    ring.ring,
                    ring.leaving_at,
                    ring.between_leaving,
                ));
                start_rings[next] = Some(SurfaceStripStation::new(
                    ring.next_ring,
                    ring.arriving_at,
                    ring.between_arriving,
                ));
            }
        }
    }

    Ok(())
}
