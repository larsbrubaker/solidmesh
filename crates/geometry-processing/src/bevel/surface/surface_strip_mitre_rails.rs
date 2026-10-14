//! The rail-by-rail machinery of the MITRE (everything but `surface_strip_mitre::try_mitre`):
//! which rail of a strip stands on the face two strips share, where the mitre plane
//! stands, one rail or one pair of rails cut on that plane, the rails the narrower strip has no
//! partner for, and the between-station lists the two end rings carry. `surface_strip_mitre` puts
//! them together into the ring; [`super::surface_mitre_chorded_apart`] and the staircase mitre cut
//! their own rings with [`try_one_rail_of_the_mitre`] and [`try_one_pair_of_the_mitre`].

// The negated comparisons are deliberate: each is false for NaN, which is the point.
#![allow(clippy::neg_cmp_op_on_partial_ord)]

use std::sync::Arc;

use vector_math::Vector3;

use super::surface_mitre_joint_step::SideSeams;
use super::{surface_mitre_seam, surface_strip_ends};
use super::{SurfaceEdgePlan, SurfaceMitreSliver, SurfaceStrip, SurfaceVertexPool};
use crate::bevel::{BevelAdvisory, BevelAdvisoryCode};

/// A point and a normal, named so the far landing plane can be passed about as
/// one thing.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SurfacePlane {
    /// A point on the plane.
    pub point: Vector3,
    /// The plane's normal.
    pub normal: Vector3,
}

/// The advisory most refusals here are named with.
fn do_not_mitre(corner: Vector3) -> BevelAdvisory {
    BevelAdvisory::new(
        BevelAdvisoryCode::SurfaceStripsDoNotMitre,
        vec![corner.into()],
    )
}

/// Two readings of what stands along one ring's chords, laid together. The sector's
/// leavings and the union's foreign crossings never share a chord - the sector runs along the
/// UNPAIRED rails and the union along the paired ones - so this is a choice per chord rather than an
/// ordering of two lists.
pub fn merge(mut sector: Vec<Vec<i32>>, union: Option<&[Vec<i32>]>) -> Vec<Vec<i32>> {
    let Some(union) = union else {
        return sector;
    };

    for (chord, theirs) in sector.iter_mut().zip(union) {
        if !theirs.is_empty() {
            // Where both carry something the sector's come first, then the union's.
            chord.extend_from_slice(theirs);
        }
    }

    sector
}

/// Puts one vertex on a chord BESIDE what it already carries, at the end the caller
/// names - leading where the point stands nearer the chord's lower rail and following where it
/// stands nearer the higher, which is the order a station's own list is read in.
pub fn beside(between: &mut [Vec<i32>], chord: i32, vertex: i32, leading: bool) {
    if chord < 0 || chord >= between.len() as i32 {
        return;
    }

    let on = &mut between[chord as usize];

    if leading {
        on.insert(0, vertex);
    } else {
        on.push(vertex);
    }
}

/// ONE strip's rail of a pair the sliver has broken, cut on the mitre plane
/// alone. A claimed foot takes the vertex its walkback left at the rim; its partner at that angle is
/// an interior rail of the wider run - the run with rails to spare - and is minted like any other
/// crossing.
#[allow(clippy::too_many_arguments)]
pub fn try_one_rail_apart(
    pool: &mut SurfaceVertexPool,
    strip: &SurfaceStrip,
    edge_index: i32,
    rail: i32,
    corner: Vector3,
    through: Vector3,
    normal: Vector3,
    sliver: &SurfaceMitreSliver,
    ring: &mut [i32],
    at: &mut [f64],
) -> Result<(), BevelAdvisory> {
    if sliver.claims_the_foot(edge_index, rail) {
        sliver.take_the_foot(edge_index, ring, at);

        return Ok(());
    }

    let Some(param) = strip.try_cross(rail, through, normal) else {
        return Err(do_not_mitre(corner));
    };
    at[rail as usize] = param;

    ring[rail as usize] = pool.add(strip.point_on(rail, param));

    Ok(())
}

/// Puts one vertex on the chord leaving `rail` in the direction the rails were
/// counted out in - which is the chord BEFORE it where they were counted down.
pub fn on_the_chord(between: &mut [Vec<i32>], rail: i32, step: i32, vertex: i32) {
    let chord = if step > 0 { rail } else { rail - 1 };

    if chord >= 0 && chord < between.len() as i32 {
        between[chord as usize] = vec![vertex];
    }
}

/// The rails of the wider strip that the narrower one has none to
/// pair with. Nothing of the narrower bead stands over them - it swept a smaller turn and its own
/// surface stopped - so what they end on is the part's face BEYOND the mitre, which is the narrower
/// edge's far landing plane still standing where it always did.
#[allow(clippy::too_many_arguments)]
pub fn try_the_rails_with_no_partner(
    pool: &mut SurfaceVertexPool,
    strip: &SurfaceStrip,
    seams: &[SideSeams],
    edge_index: i32,
    corner: Vector3,
    mine_on_the_face: i32,
    mine_step: i32,
    paired: i32,
    beyond: SurfacePlane,
    ring: &mut [i32],
    leaving_at: &mut [f64],
) -> Result<(), BevelAdvisory> {
    if paired >= strip.rail_count() {
        return Ok(());
    }

    for step in paired..strip.rail_count() {
        let mine = mine_on_the_face + (mine_step * step);

        let Some(param) = strip.try_cross(mine, beyond.point, beyond.normal) else {
            return Err(do_not_mitre(corner));
        };
        leaving_at[mine as usize] = param;

        let at = strip.point_on(mine, param);

        if mine == 0 || mine == strip.rail_count() - 1 {
            // This strip's own outermost rail, where the walkback already left a vertex on the
            // tangent line: taken by index rather than minted, as every other outer rail is.
            let did_not_meet = || {
                BevelAdvisory::new(
                    BevelAdvisoryCode::SurfaceStripSeamDidNotMeet,
                    vec![at.into()],
                )
            };
            let side = if mine == 0 { 0 } else { 1 };

            // A missing or empty seam (which the builder never hands in) is refused as the seam not
            // meeting, which is what an unmatched one is: library code does not panic on its input.
            let Some(run) = seams
                .get(edge_index as usize)
                .and_then(|seam| seam[side].as_deref())
                .filter(|run| !run.is_empty())
            else {
                return Err(did_not_meet());
            };
            let meets = surface_strip_ends::meets_the_seam(pool);
            let mut standing = run[run.len() - 1].0;

            if (pool.position_of(standing) - at).length() > meets {
                standing = run[0].0;
            }

            if (pool.position_of(standing) - at).length() > meets {
                // The two ends are where this rail runs out in the ordinary case, and they are asked
                // first so that nothing about it moves. But a rail may end part way ALONG its own
                // run: on the mouth of a raked bore the face beyond is the next facet of the same
                // wall, so the unpaired rail is cut on the CREASE the two facets share - a mesh
                // edge, where the walkback's own cut already left a vertex, and one standing a good
                // thirtieth of a millimetre inside the band's far end rather than at it.
                if let Some(station) = run
                    .iter()
                    .find(|station| (pool.position_of(station.0) - at).length() <= meets)
                {
                    standing = station.0;
                }
            }

            if (pool.position_of(standing) - at).length() > meets {
                return Err(did_not_meet());
            }

            ring[mine as usize] = standing;
            continue;
        }

        ring[mine as usize] = pool.add(at);
    }

    Ok(())
}

/// Which of a strip's two outermost rails stands on the face it shares with
/// its neighbour - rail 0 where that region is the plan's left, the last rail where it is its right
/// - or `None` where the two share no region.
pub fn try_the_shared_rail(
    strip: &SurfaceStrip,
    mine: &SurfaceEdgePlan,
    theirs: &SurfaceEdgePlan,
) -> Option<i32> {
    if Arc::ptr_eq(&mine.left_region, &theirs.left_region)
        || Arc::ptr_eq(&mine.left_region, &theirs.right_region)
    {
        return Some(0);
    }

    if Arc::ptr_eq(&mine.right_region, &theirs.left_region)
        || Arc::ptr_eq(&mine.right_region, &theirs.right_region)
    {
        return Some(strip.rail_count() - 1);
    }

    None
}

/// Where the mitre plane stands, as a refusal rather than as a fallback: two
/// runs that turn on no one face have no pair of tangent lines to cross, and two whose lines never
/// meet double back rather than turn a corner. Either way the two sides of the chain are not one
/// face each here, which is what `SurfaceStripsDoNotMitre` already says.
pub fn try_the_seam_point(
    pool: &SurfaceVertexPool,
    leaving: &SurfaceEdgePlan,
    arriving: &SurfaceEdgePlan,
    corner: Vector3,
    the_chorded_runs_joint: Option<Vector3>,
) -> Result<Vector3, BevelAdvisory> {
    let meets = surface_strip_ends::meets_the_seam(pool);

    if let Some(through) = surface_mitre_seam::try_the_point(leaving, arriving, meets) {
        return Ok(through);
    }

    if surface_mitre_seam::the_lines_are_one(leaving, arriving, meets) {
        // The run's own joint where the plan found one, and the corner everywhere else: the
        // walkback's band and the strips' ring are cut on ONE plane or the two leave a crack between
        // them (SurfaceMitreOnAChordedRun).
        //
        // Two COLLINEAR pieces of one crease - one the source's own boolean chorded - have one
        // tangent line between them rather than two that cross, so there is nothing to read a
        // crossing off and nothing that wants reading: one crease between one pair of faces sets
        // both pieces back by the same amount, and the corner is where the plane goes. With the
        // normal being the sum of two equal walk directions, that is the square cut through the
        // corner a straight run wants, and every rail of the two strips pairs with its own.
        return Ok(the_chorded_runs_joint.unwrap_or(corner));
    }

    Err(do_not_mitre(corner))
}

/// ONE rail of a mitre - the vertex the two strips share where that rail
/// crosses the plane, and the parameter each of them meets it at, as `(vertex, leaving_at,
/// arriving_at)`. Rail 0 and the last rail take the vertex the walkback already left on the seam
/// rather than minting a second one there; the rails between are new points.
///
/// Written apart from `try_mitre` so that a staircase - where only SOME rails end on this corner and
/// the rest end on a virtual one - cuts its ordinary rails by the same statement rather than by a
/// copy of it. `corner` is where the two strips meet, which is where a refusal here is REPORTED;
/// `through` is the point the mitre plane passes through, which is the corner only where the two
/// beads set back by the same amount on the face they share (see [`try_the_seam_point`]).
#[allow(clippy::too_many_arguments)]
pub fn try_one_rail_of_the_mitre(
    pool: &mut SurfaceVertexPool,
    strip: &SurfaceStrip,
    next: &SurfaceStrip,
    seams: &[SideSeams],
    edge_index: i32,
    next_index: i32,
    rail: i32,
    corner: Vector3,
    through: Vector3,
    normal: Vector3,
) -> Result<(i32, f64, f64), BevelAdvisory> {
    try_one_pair_of_the_mitre(
        pool, strip, next, seams, edge_index, next_index, rail, rail, corner, through, normal,
    )
}

/// ONE PAIR of rails of a mitre - one of each strip, which are the same
/// rail only where the two beads swept the same turn - as `(vertex, leaving_at, arriving_at)`. They
/// are paired by their ANGLE from the shared face, counted in steps out from each strip's own foot on
/// it, and the two are checked to arrive at one point: reflection in the seam plane carries one
/// cylinder onto the other preserving that angle, so a pair that does NOT meet is the two strips
/// chording at different angular steps, which is what the check really tests and why no separate
/// assertion of the step is needed.
#[allow(clippy::too_many_arguments)]
pub fn try_one_pair_of_the_mitre(
    pool: &mut SurfaceVertexPool,
    strip: &SurfaceStrip,
    next: &SurfaceStrip,
    seams: &[SideSeams],
    edge_index: i32,
    next_index: i32,
    rail: i32,
    their_rail: i32,
    corner: Vector3,
    through: Vector3,
    normal: Vector3,
) -> Result<(i32, f64, f64), BevelAdvisory> {
    let (Some(here), Some(there)) = (
        strip.try_cross(rail, through, normal),
        next.try_cross(their_rail, through, normal),
    ) else {
        return Err(do_not_mitre(corner));
    };

    let at = strip.point_on(rail, here);

    // Where that crossing lands along the edge is NOT bounded by the edge: a mitre carries each rail
    // by its own offset from the edge over the cosine of half the turn, back from the corner on the
    // inside of the turn and PAST it on the outside, so a rim whose facets are finer than its bevel
    // - a part chord 1.12 long under a 3 mm fillet at a reflex corner, which is what a cube merged
    // with a cylinder leaves - carries a rail a good chord or two past its own end and is perfectly
    // ordinary (SurfaceMitreOverAShortChordTests). What the strip really needs is a window between
    // its two ends, and that is the stitch's own reading, per rail and exact.
    let (leaving_at, arriving_at) = (here, there);

    if (at - next.point_on(their_rail, there)).length() > surface_strip_ends::meets_the_seam(pool) {
        return Err(do_not_mitre(corner));
    }

    let mine_is_outer = rail == 0 || rail == strip.rail_count() - 1;
    let theirs_is_outer = their_rail == 0 || their_rail == next.rail_count() - 1;

    if mine_is_outer && theirs_is_outer {
        // Both strips' outermost rails, which is the pair standing ON the shared face: the walkback
        // left ONE vertex there for the two of them, and a mitre that minted a second would leave
        // the seam split down its whole length.
        let did_not_meet = || {
            BevelAdvisory::new(
                BevelAdvisoryCode::SurfaceStripSeamDidNotMeet,
                vec![at.into()],
            )
        };
        let side = if rail == 0 { 0 } else { 1 };
        let their_side = if their_rail == 0 { 0 } else { 1 };

        // A missing or empty seam (which the builder never hands in) is refused as the seam not
        // meeting, which is what an unmatched one is: library code does not panic on its input.
        let run_of = |edge: i32, side: usize| {
            seams
                .get(edge as usize)
                .and_then(|seam| seam[side].as_deref())
                .filter(|run| !run.is_empty())
        };
        let (Some(mine_run), Some(their_run)) =
            (run_of(edge_index, side), run_of(next_index, their_side))
        else {
            return Err(did_not_meet());
        };
        let leaving = mine_run[mine_run.len() - 1].0;
        let arriving = their_run[0].0;

        if leaving != arriving
            || (pool.position_of(leaving) - at).length() > surface_strip_ends::meets_the_seam(pool)
        {
            return Err(did_not_meet());
        }

        return Ok((leaving, leaving_at, arriving_at));
    }

    // ONE of them outermost and the other not is the TURN POINT of a staircase, and it is MINTED
    // like any interior rail rather than taken off the walkback. It stands ON the narrower bead's
    // far tangent line, but not at the END of it: that band is no longer stopped by the mitre at all
    // - the face it lies on is not the one the two edges share, so it runs on to its region's own
    // rim (`surface_plan_band_ends::the_mitre_bounds_this_side`) - and the walkback's vertices
    // there are at the rim, past this point. What lies between the two is the sector the face gives
    // up.
    Ok((pool.add(at), leaving_at, arriving_at))
}
