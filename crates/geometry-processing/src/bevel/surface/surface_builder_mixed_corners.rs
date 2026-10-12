//! The corner loop's MIXED corner, lifted whole out of `SurfaceBevelBuilder`: a corner the
//! convexity turns over at is not a mitre at all, so everything the build does with one - the lean
//! gate, the wall's run-out, the bands the run-out crosses, the patch and the two end rings it
//! leaves - stands here rather than in the middle of the loop that finds it. Ports
//! `Bevel/Surface/SurfaceBuilderMixedCorners.cs` (C# `internal static class`; free functions here):
//! [`try_survey_the_run_outs`] and [`try_close`], over the corner's patch
//! ([`super::surface_mixed_corner_patch`]) or, for a leaning corner, its dual
//! ([`super::surface_mixed_corner_dual_patch`]).
//!
//! C#'s private `TheCapIsSquareEnough` constant is not carried: nothing in the class reads it
//! (`SurfaceMixedCorner::the_cap_leans` holds the test the corner loop asks).

use std::rc::Rc;

use vector_math::Vector3;

use super::surface_mitre_joint_step::SideSeams;
use super::surface_mixed_corner_patch::SurfaceMixedCornerPatchClosed;
use super::surface_strip_trimmed_at_the_crossing::SurfaceStripTrims;
use super::{
    surface_bevel_crossings, surface_mixed_corner_dual_patch, surface_mixed_corner_patch,
    surface_mixed_corner_wall_ends, surface_mixed_corner_which_canal, SurfaceBevelPlan,
    SurfaceEdgePlan, SurfaceMixedCorner, SurfaceMixedCornerDualCanal, SurfaceMixedCornerWall,
    SurfaceRoomCrossing, SurfaceStrip, SurfaceStripStation, SurfaceVertexPool,
};
use crate::bevel::{BevelAdvisory, BevelAdvisoryCode};
use crate::topology::{EdgeConvexity, FaceRegionCache};

use super::SurfaceMitreConsumesTheRail;

/// C# `TrySurveyTheRunOuts`: READ BEFORE THE SEAMS - every leaning mixed corner asked whether its
/// run-out would die PAST the end of the crease it dies on, which is a refusal about the SIZE and
/// has to be said before any reading that the same overshoot makes fail downstream of it.
///
/// The corner loop runs after the seams are picked, and by then a run-out that overshoots its
/// crease has already taken the tangent run the seam on that side would have been sewn to - so the
/// seam loop answers first and the pass is named `SurfaceStripSeamDidNotMeet` at whatever mesh
/// vertex the run happens to die at. MEASURED on the nose-in-notch solid, the ridge plus the upper
/// right crease at radius 2: the clean mesh says `SurfaceMixedCornerRunOutPastTheCrease` at the
/// right ridge end and the boolean-CHORDED one said the seam name at the chord (12, 8, 12). Both
/// build at 1.70 and refuse at 1.71; the overshoot is the whole of it.
///
/// ONLY that one name is raised here, and only where every gate the corner loop itself applies is
/// already passed: a corner the loop would not build on the dual at all, or would refuse for
/// another reason, is left entirely alone. `consumed` says which chords their own two mitres have
/// eaten (staircases, no corner of this kind); `lay_the_oblique_rows_anyway` lifts THE LEAN GATE.
#[allow(clippy::too_many_arguments)]
pub fn try_survey_the_run_outs(
    regions: Option<&FaceRegionCache>,
    plans: &[Rc<SurfaceBevelPlan>],
    first_of_plan: &[i32],
    count: i32,
    crossings: Option<&[SurfaceRoomCrossing]>,
    consumed: &[Option<SurfaceMitreConsumesTheRail>],
    lay_the_oblique_rows_anyway: bool,
) -> Result<(), BevelAdvisory> {
    if lay_the_oblique_rows_anyway {
        return Ok(());
    }

    for (p, plan) in plans.iter().enumerate() {
        let in_chain = plan.edges.len();

        let mut i = 0;
        while i + 1 < in_chain || (plan.is_closed && i < in_chain) {
            let here = (first_of_plan[p] + i as i32) as usize;
            let next = (first_of_plan[p] + ((i + 1) % in_chain) as i32) as usize;
            let corner_index = i as i32;
            i += 1;

            if consumed[here].is_some() || consumed[next].is_some() {
                continue;
            }

            let Some(mixed) = plan.try_mixed_corner_after(corner_index) else {
                continue;
            };

            if !mixed.the_cap_leans() {
                continue;
            }

            // A crossed run-out is refused by the corner loop under its own name, before the canal
            // is ever read - so it is not this survey's to answer for.
            let Ok((crossed, crossed_on_the_wall)) =
                surface_bevel_crossings::try_the_band_crossed_by(
                    crossings,
                    plans,
                    first_of_plan,
                    p as i32,
                    corner_index,
                    count,
                    &mixed,
                )
            else {
                continue;
            };

            if crossed >= 0 || crossed_on_the_wall >= 0 {
                continue;
            }

            // And a corner with no crease to die on is the oblique-rows refusal, which is a
            // different rung and stays where it is said.
            let Some(how_far_the_crease_runs) = try_how_far_the_crease_runs(regions, &mixed) else {
                continue;
            };

            if let Err(why) = SurfaceMixedCornerDualCanal::try_read(&mixed, how_far_the_crease_runs)
            {
                if why.code() == BevelAdvisoryCode::SurfaceMixedCornerRunOutPastTheCrease {
                    return Err(why);
                }
            }
        }
    }

    Ok(())
}

/// C# `TryClose`: closes one mixed corner - the cut's end ring carried round the bead's axis, with
/// the two bites its footprint takes out of the faces nobody picked, every band its run-out crosses
/// cut back to the curve they meet in, and each strip left standing on the ring the patch cut it
/// at. `p` is the plan the corner is in, `i` which corner of it (the one after edge `i`), `here`
/// and `next` those edges under the pass's numbering. The crossing trims are written into
/// `trimmed`, and each strip's ring at the start and end of its edge into `start_rings` and
/// `end_rings`.
///
/// A cut or bead with no strip (C# hands the patch a null and throws there) is refused as
/// `SurfaceCornerNotSupported` at the corner (divergence 5035).
#[allow(clippy::too_many_arguments)]
pub fn try_close(
    pool: &mut SurfaceVertexPool,
    faces: &mut Vec<(i32, i32, i32)>,
    regions: Option<&FaceRegionCache>,
    plans: &[Rc<SurfaceBevelPlan>],
    first_of_plan: &[i32],
    edges: &[SurfaceEdgePlan],
    strips: &[Option<Rc<SurfaceStrip>>],
    seams: &[SideSeams],
    count: i32,
    crossings: Option<&[SurfaceRoomCrossing]>,
    p: usize,
    i: i32,
    here: usize,
    next: usize,
    mixed: &Rc<SurfaceMixedCorner>,
    lay_the_oblique_rows_anyway: bool,
    trimmed: &mut SurfaceStripTrims,
    start_rings: &mut [Option<SurfaceStripStation>],
    end_rings: &mut [Option<SurfaceStripStation>],
) -> Result<(), BevelAdvisory> {
    // WHICH CANAL, and it is the lean that decides. A mixed corner's blend can be read from either
    // of its two strips - as the MATERIAL ball riding the cap and pressed against the bead's tube,
    // or as its DUAL, the bead's VOID ball riding the wall and pressed against the cut's. On a
    // SQUARE corner the two are one torus and there is nothing to choose. On a LEANING one they are
    // different surfaces, and measured against the Onshape control of "Top Center Face" it is the
    // dual the corner draws: 1.3e-5 mm over 500 blend vertices at each end, where the cap-riding
    // reading stands 0.648 off
    // (BevelTopCenterFaceRegressionTests.TheControlsBlendLiesOnTheDualCanalNotTheCapRidingOne).
    // Until a square control arrives the square corner keeps the rows below, unchanged to the bit.
    let the_cap_is_square = !mixed.the_cap_leans();

    // Which edge the CUT is comes off the plan's roles rather than off the corner's own two: where
    // the blend swallows a chord or more, the cut stands further along the chain and the chords
    // between have no strip to join (SurfaceEdgeRoles).
    let cut_is_arriving = edges[here].convexity() == EdgeConvexity::Convex;

    let Some(cut_in_chain) = plans[p].roles.borrow().try_cut_of(i) else {
        return Err(BevelAdvisory::new(
            BevelAdvisoryCode::SurfaceCornerNotSupported,
            vec![edges[here].to().into()],
        ));
    };

    let cut = (first_of_plan[p] + cut_in_chain) as usize;
    let bead = if cut_is_arriving { next } else { here };

    if !the_cap_is_square && !lay_the_oblique_rows_anyway {
        // WHICH CANAL, by CREASE REACH (SurfaceMixedCornerWhichCanal). The dual runs past the end
        // of the cut's own strip and stops where the ball's contact on the wall reaches the crease
        // the cap and that wall meet in, so it is the corner's canal only where the part carries
        // that crease THE WAY THE DUAL TRAVELS. The same line the other way is what the CAP-RIDING
        // run-out dies on, and on an L prism that is where the ten millimetres of edge are; so that
        // corner falls through to the cap-riding path below.
        if let Some(how_far_the_crease_runs) = try_how_far_the_crease_runs(regions, mixed) {
            return try_close_on_the_dual(
                pool,
                faces,
                plans,
                first_of_plan,
                strips,
                seams,
                count,
                crossings,
                p,
                i,
                mixed,
                (cut, bead),
                cut_is_arriving,
                how_far_the_crease_runs,
                start_rings,
                end_rings,
            );
        }

        // THE LEAN GATE, now down to the one handover the cap-riding path still draws on the
        // TORUS: a corner that hands over on a MITRE RING, whose grid is the SCHEDULE's
        // (SurfaceMixedCornerRingSchedule), turning the ring rigidly about the axis. That is right
        // only while the cap stands square, and no fixture has a corner that both leans and hands
        // over at a mitre - so it is refused BY NAME rather than published as a blend the right
        // reading and the wrong rows would make.
        if mixed.hands_over_at_a_mitre {
            return Err(BevelAdvisory::new(
                BevelAdvisoryCode::SurfaceObliqueMixedCornerRowsNotBuilt,
                vec![mixed.corner.into(), mixed.spine.cap_lean_degrees().into()],
            ));
        }
    }

    // Whether the wall nobody picked runs out under the blend, and what stands across it there:
    // read here, where the regions are, so the corner itself stays a reading of plans alone
    // (SurfaceMixedCornerWallEnds). Where it runs out into something that is not one plane to trim
    // against, that is refused by name rather than built as though the wall went on.
    let wall_ends_at = surface_mixed_corner_wall_ends::try_read(regions, mixed)?;

    // And whether another chain's band is crossed by this corner's run-out: named by the room
    // reading, which is the one stage that sees the two footprints together.
    let (crossed, crossed_on_the_wall) = surface_bevel_crossings::try_the_band_crossed_by(
        crossings,
        plans,
        first_of_plan,
        p as i32,
        i,
        count,
        mixed,
    )?;

    // Both bands of one mitred corner crossed at once: the hand-over on their own ring, read and
    // pooled before anything is drawn, because it is the last corner of both curves and the
    // run-out's own last column (SurfaceCrossingAtTheMitre).
    let mut at_the_mitre = None;

    if crossed >= 0 && crossed_on_the_wall >= 0 {
        let the_wall = SurfaceMixedCornerWall::try_read(mixed)?;
        at_the_mitre = Some(surface_bevel_crossings::try_the_hand_over_at_the_mitre(
            pool,
            strips,
            edges,
            crossed,
            crossed_on_the_wall,
            mixed,
            &the_wall,
        )?);
    }

    let (cut_strip, bead_strip) = the_two_strips(strips, cut, bead, mixed)?;

    let crossed_band = if crossed >= 0 {
        strips[crossed as usize].as_ref()
    } else {
        None
    };
    let crossed_wall_band = if crossed_on_the_wall >= 0 {
        strips[crossed_on_the_wall as usize].as_deref()
    } else {
        None
    };

    let SurfaceMixedCornerPatchClosed {
        cut_station,
        bead_station,
        crossing,
        crossing_on_the_wall,
        the_die_point,
    } = surface_mixed_corner_patch::try_close(
        pool,
        faces,
        mixed,
        &cut_strip,
        &bead_strip,
        seams,
        cut,
        bead,
        cut_is_arriving,
        !cut_is_arriving,
        wall_ends_at,
        crossed_band,
        if crossed >= 0 {
            edges[crossed as usize].from()
        } else {
            Vector3::ZERO
        },
        crossed_wall_band,
        at_the_mitre.as_deref(),
    )?;

    // Each crossed band cut back by its own curve. With one band that is the one call it always
    // was; with two, each closes on the hand-over they share.
    surface_bevel_crossings::try_cut_the_band_back(
        pool,
        strips,
        edges,
        trimmed,
        crossed,
        false,
        the_die_point,
        crossing.as_deref(),
        at_the_mitre.clone(),
        mixed,
    )?;
    surface_bevel_crossings::try_cut_the_band_back(
        pool,
        strips,
        edges,
        trimmed,
        crossed_on_the_wall,
        true,
        the_die_point,
        crossing_on_the_wall.as_deref(),
        at_the_mitre,
        mixed,
    )?;

    // The patch is its own two end rings, so each strip is stitched up to the ring the corner cut
    // it at rather than to a mitre's - each at the end of its own edge that faces the corner, which
    // for a cut the blend reached over is not the end the corner's own two edges meet at.
    leave_the_strips_standing(
        cut_is_arriving,
        (cut, bead),
        cut_station,
        bead_station,
        start_rings,
        end_rings,
    );

    Ok(())
}

/// C# `TryCloseOnTheDual`: closes a LEANING corner on its DUAL canal - the one the control draws -
/// carried past the end of the cut's own strip until it dies on the crease the cap and the wall
/// meet in. A separate path and not a flag through [`try_close`], because almost nothing of that
/// block applies: the wall does not run out under this blend, there is no mitre handover, the cap's
/// bite is the run-out's own curve, and the run-out is not the flat-wall closed form at all.
#[allow(clippy::too_many_arguments)]
fn try_close_on_the_dual(
    pool: &mut SurfaceVertexPool,
    faces: &mut Vec<(i32, i32, i32)>,
    plans: &[Rc<SurfaceBevelPlan>],
    first_of_plan: &[i32],
    strips: &[Option<Rc<SurfaceStrip>>],
    seams: &[SideSeams],
    count: i32,
    crossings: Option<&[SurfaceRoomCrossing]>,
    p: usize,
    i: i32,
    mixed: &Rc<SurfaceMixedCorner>,
    (cut, bead): (usize, usize),
    cut_is_arriving: bool,
    how_far_the_crease_runs: f64,
    start_rings: &mut [Option<SurfaceStripStation>],
    end_rings: &mut [Option<SurfaceStripStation>],
) -> Result<(), BevelAdvisory> {
    // A leaning corner whose run-out crosses another chain's band is a shape nobody has drawn: the
    // cap-riding path cuts those bands back against a run-out this corner does not have, and
    // guessing what the dual's own rows would do to them is not a reading. Named rather than built.
    let (crossed, crossed_on_the_wall) = surface_bevel_crossings::try_the_band_crossed_by(
        crossings,
        plans,
        first_of_plan,
        p as i32,
        i,
        count,
        mixed,
    )?;

    if crossed >= 0 || crossed_on_the_wall >= 0 {
        return Err(BevelAdvisory::new(
            BevelAdvisoryCode::SurfaceCornerNotSupported,
            vec![mixed.corner.into()],
        ));
    }

    let canal = Rc::new(SurfaceMixedCornerDualCanal::try_read(
        mixed,
        how_far_the_crease_runs,
    )?);

    let (cut_strip, bead_strip) = the_two_strips(strips, cut, bead, mixed)?;

    let (cut_station, bead_station) = surface_mixed_corner_dual_patch::try_close(
        pool,
        faces,
        &canal,
        &cut_strip,
        &bead_strip,
        seams,
        cut,
        bead,
        cut_is_arriving,
        !cut_is_arriving,
    )?;

    leave_the_strips_standing(
        cut_is_arriving,
        (cut, bead),
        cut_station,
        bead_station,
        start_rings,
        end_rings,
    );

    Ok(())
}

/// The cut's and the bead's strips. C# hands the patch whatever the array holds and a null throws
/// there; a corner whose strip the blend swallowed has no ring to close, so it is refused by name
/// at the corner instead (divergence 5035).
fn the_two_strips(
    strips: &[Option<Rc<SurfaceStrip>>],
    cut: usize,
    bead: usize,
    mixed: &SurfaceMixedCorner,
) -> Result<(Rc<SurfaceStrip>, Rc<SurfaceStrip>), BevelAdvisory> {
    match (&strips[cut], &strips[bead]) {
        (Some(cut), Some(bead)) => Ok((Rc::clone(cut), Rc::clone(bead))),
        _ => Err(BevelAdvisory::new(
            BevelAdvisoryCode::SurfaceCornerNotSupported,
            vec![mixed.corner.into()],
        )),
    }
}

/// Each strip left standing on the ring the corner cut it at, at the end of its own edge that
/// faces the corner (the tail both of C#'s close paths share).
fn leave_the_strips_standing(
    cut_is_arriving: bool,
    (cut, bead): (usize, usize),
    cut_station: SurfaceStripStation,
    bead_station: SurfaceStripStation,
    start_rings: &mut [Option<SurfaceStripStation>],
    end_rings: &mut [Option<SurfaceStripStation>],
) {
    if cut_is_arriving {
        end_rings[cut] = Some(cut_station);
        start_rings[bead] = Some(bead_station);
    } else {
        start_rings[cut] = Some(cut_station);
        end_rings[bead] = Some(bead_station);
    }
}

/// C# `TryHowFarTheCreaseRuns`: how far the crease the cap and the wall meet in actually runs THE
/// WAY THE DUAL TRAVELS, or `None` where the part carries no edge that way out of this corner -
/// which is what says whether the dual is this corner's canal at all. The region cache is asked
/// for nothing but its presence: a pass handed no cache at all was handed no regions to read a rim
/// off either, and it keeps the answer it had.
fn try_how_far_the_crease_runs(
    regions: Option<&FaceRegionCache>,
    corner: &SurfaceMixedCorner,
) -> Option<f64> {
    regions?;
    surface_mixed_corner_which_canal::try_how_far_the_duals_crease_runs(corner)
}
