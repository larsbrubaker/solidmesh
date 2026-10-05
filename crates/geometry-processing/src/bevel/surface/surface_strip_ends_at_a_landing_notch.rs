//! The end of a band that LANDED on its face, with a standing feature's extension notching it: the
//! ring the ordinary open end would have stood on, cut back to a cross section clear of the notch,
//! the tail past that section sewn band by band, and the section the end gives up split around the
//! extension. Ports `Bevel/Surface/SurfaceStripEndsAtALandingNotch.cs` (C# `internal static class`;
//! a module of free functions here). What is here is which way the cap is walked ([`try_the_walk`])
//! and the notched ring ([`try_the_ring`]); the close itself (`TryCloseTheLandingEnd`) lands with a
//! later package.
//!
//! Nothing here is a second composition: the three pieces are the very ones the RUN-OFF end already
//! composes (`SurfaceStripEndsOverTheFaceBeyond`), and what differs is where the ring comes from. A
//! landing ring is the open end's: every rail crossing the one end plane, rail 0 and the last
//! standing on their seams' own last vertices.
//!
//! **The end has to be square to the edge.** The notch's two Qs are read where each extended plane
//! meets the chord the end ring draws at the band's own END PARAMETER, so a ring standing anywhere
//! else is a ring the Qs are not on, and that is refused by name.

use std::rc::Rc;

use vector_math::Vector3;

use super::surface_mitre_joint_step::SideSeams;
use super::{
    surface_strip_ends, SurfaceStandingFeatureWithheldFace, SurfaceStrip,
    SurfaceStripNotchedByAFeature, SurfaceVertexPool,
};
use crate::bevel::{BevelAdvisory, BevelAdvisoryCode};

/// C# `TryTheWalk`: the order the cap walks the ring's rails in, from the CORNER's side, so the
/// cap's head is the corner fan and its tail the sliver. `Err` is C#'s `failure`.
///
/// A landing end has both ring ends standing on the rim and nothing about the rails' numbering says
/// which of them the corner's piece is on: the chain's own walk decides it, and the same part read as
/// a kernel mesh and loaded from its `.mcx` reads the two the opposite way round. So it is MEASURED,
/// on the one thing that tells the two pieces apart: the SLIVER is the piece with no corner on it,
/// and that shows as its crease vertex standing ON the straight side between its own ring end and
/// the corner.
pub fn try_the_walk(
    pool: &SurfaceVertexPool,
    notch: &SurfaceStripNotchedByAFeature,
    ring: &[i32],
    corner: Vector3,
) -> Result<Vec<i32>, BevelAdvisory> {
    let count = ring.len() as i32;
    let walked = |reversed: bool| -> Vec<i32> {
        (0..count)
            .map(|at| if reversed { count - 1 - at } else { at })
            .collect()
    };

    if notch.is_one_faced() {
        // Nothing to measure. A one-faced notch runs OFF the profile at its own crease rail - the
        // seam rail the crease dies on - so the corner's piece is whatever is left on the other side
        // of it, and the walk starts at that end.
        return Ok(walked(notch.the_crease_rail == 0));
    }

    let does_not_extend = || {
        BevelAdvisory::new(
            BevelAdvisoryCode::SurfaceStandingFeatureDoesNotExtend,
            vec![corner.into()],
        )
    };

    let crease_end = |face: &Option<Rc<SurfaceStandingFeatureWithheldFace>>| {
        face.as_ref().and_then(|face| {
            notch
                .feature
                .try_the_ends_of_the_crease(pool, face, notch.reach)
        })
    };

    let (Some((_, low_crease)), Some((_, high_crease))) = (
        crease_end(&notch.at_the_low_rail.face),
        crease_end(&notch.at_the_high_rail.face),
    ) else {
        return Err(does_not_extend());
    };

    let low_is_the_sliver =
        the_crease_stands_on_the_straight_side(pool, ring[0], low_crease, corner);
    let high_is_the_sliver =
        the_crease_stands_on_the_straight_side(pool, ring[ring.len() - 1], high_crease, corner);

    if low_is_the_sliver == high_is_the_sliver {
        // Both pieces read as the sliver, or neither does: the feature's footprint is not standing
        // against ONE of the section's straight sides with the corner outside it, which is the only
        // shape this split draws. Named rather than guessed at.
        return Err(does_not_extend());
    }

    Ok(walked(low_is_the_sliver))
}

/// C# `TheCreaseStandsOnTheStraightSide`: whether `crease` stands on the straight side running
/// from one end of the ring to the corner - its PERPENDICULAR distance from that line, against the
/// same ruler every other meeting of two readings is held to, and inside the side's own two ends.
/// Perpendicular rather than "the two legs come to the whole", whose difference is QUADRATIC in the
/// offset and so read a point a hundred times too far off as on the line.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
pub fn the_crease_stands_on_the_straight_side(
    pool: &SurfaceVertexPool,
    end: i32,
    crease: i32,
    corner: Vector3,
) -> bool {
    let at = pool.position_of(end);
    let on = pool.position_of(crease);
    let side = corner - at;
    let near = surface_strip_ends::meets_the_seam(pool);

    if !(side.length() > near) {
        // The ring end stands on the corner itself, so there is no straight side to be on.
        return false;
    }

    let along = (on - at).dot(side) / side.length();

    // Inside the side's own ends as well as on its line: a crease vertex standing BEYOND the corner,
    // or behind the ring end, is on the line the side lies in and on no part of the side.
    along > -near
        && along < side.length() + near
        && (on - at).cross(side / side.length()).length() < near
}

/// C# `TryTheRing`: the ring at a notched landing end - the open end's own ring, with every rail
/// the feature took pulled back to the cross section (`section`) instead of reaching the end plane -
/// and where each rail meets it (C#'s `ring` and `meetsAt`). `seams` is C#'s
/// `List<(int, double)>[,]` table indexed by `edge_index`; a missing or empty run there, where C#
/// would throw, is refused as an end not supported.
#[allow(clippy::too_many_arguments, clippy::neg_cmp_op_on_partial_ord)]
pub fn try_the_ring(
    pool: &mut SurfaceVertexPool,
    strip: &SurfaceStrip,
    seams: &[SideSeams],
    edge_index: i32,
    corner: Vector3,
    at_the_start: bool,
    runs_out_facing: Vector3,
    notch: &SurfaceStripNotchedByAFeature,
    section: f64,
) -> Result<(Vec<i32>, Vec<f64>), BevelAdvisory> {
    let not_supported = || {
        BevelAdvisory::new(
            BevelAdvisoryCode::SurfaceStripEndNotSupported,
            vec![corner.into()],
        )
    };

    let seam_end = |side: usize| -> Option<i32> {
        let run = seams.get(edge_index as usize)?[side].as_ref()?;
        let end = if at_the_start {
            run.first()
        } else {
            run.last()
        };
        end.map(|&(vertex, _)| vertex)
    };

    let (Some(on_the_left), Some(on_the_right)) = (seam_end(0), seam_end(1)) else {
        return Err(not_supported());
    };

    // The WALL the groove ends against, square to the edge, rather than the plane through the
    // corner and the two seams' last vertices that an un-notched end reads. Those two readings are
    // the same plane while both seams reach the corner - and a ONE-FACED notch is exactly the case
    // where one of them does not. The notch's own Qs are read at the band's end parameter, which is
    // this wall.
    let normal = if runs_out_facing.length_squared() > 0.0 {
        runs_out_facing.get_normal()
    } else {
        (pool.position_of(on_the_left) - corner).cross(pool.position_of(on_the_right) - corner)
    };

    if !(normal.length_squared() > 0.0) {
        return Err(not_supported());
    }

    let normal = normal.get_normal();

    let rail_count = strip.rail_count();
    let mut ring = vec![0i32; rail_count as usize];
    let mut meets_at = vec![0.0; rail_count as usize];
    let does_not_extend_at = |at: Vector3| {
        BevelAdvisory::new(
            BevelAdvisoryCode::SurfaceStandingFeatureDoesNotExtend,
            vec![at.into()],
        )
    };

    for rail in 0..rail_count {
        let index = rail as usize;
        let Some(param) = strip.try_cross(rail, corner, normal) else {
            return Err(not_supported());
        };

        if rail == notch.the_crease_rail {
            // The rail the CREASE runs out on stops at its crossing and nowhere else. It is the
            // strip's own seam rail, and that crossing is the vertex the walkback's seam ENDS on - so
            // standing the ring anywhere short of it would leave the last stretch of the seam with no
            // station at it, which the stitch refuses outright.
            meets_at[index] = notch.param_on(rail).ok_or_else(not_supported)?;
            ring[index] = notch.exit.vertex;
            continue;
        }

        if notch.notches(rail) {
            let notched_at = notch.param_on(rail).ok_or_else(not_supported)?;

            if rail == 0 || rail == rail_count - 1 {
                // A notched rail at the edge of the profile has the trace leaving through the strip's
                // own SEAM rather than through the band's end, and the seam's vertex then stands on
                // nothing. The one rail this is ordinary for is the CREASE rail, taken above.
                return Err(does_not_extend_at(strip.point_on(rail, notched_at)));
            }

            meets_at[index] = section;
            ring[index] = pool.add(strip.point_on(rail, section));
            continue;
        }

        if (param - notch.end_param).abs() > surface_strip_ends::meets_the_seam(pool) {
            // The end plane is not square to the edge, so this rail reaches it somewhere other than
            // the band's own end parameter - and the notch's Qs were read on the chords AT that
            // parameter. Refused rather than sewn to a chord the trace does not cross.
            return Err(does_not_extend_at(strip.point_on(rail, param)));
        }

        meets_at[index] = param;

        let at = strip.point_on(rail, param);

        if rail == 0 || rail == rail_count - 1 {
            let vertex = if rail == 0 { on_the_left } else { on_the_right };

            if (pool.position_of(vertex) - at).length() > surface_strip_ends::meets_the_seam(pool) {
                return Err(BevelAdvisory::new(
                    BevelAdvisoryCode::SurfaceStripSeamDidNotMeet,
                    vec![at.into()],
                ));
            }

            ring[index] = vertex;
            continue;
        }

        ring[index] = pool.add(at);
    }

    Ok((ring, meets_at))
}
