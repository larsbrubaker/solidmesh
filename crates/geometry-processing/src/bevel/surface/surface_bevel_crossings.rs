//! Where the build takes a crossing the room reading found - a mixed corner's RUN-OUT lying over
//! another chain's BAND - and cuts that band back to the curve the two meet in.
//!
//! The room measures the pair ([`super::SurfaceRoomCrossing`]) and the corner draws the curve, but
//! only the builder holds both the strips and the corners in one scope, so this is where the two
//! are put together: which strip a corner crosses, whether that strip has been claimed already, the
//! trim it is cut back by ([`super::SurfaceStripTrimmedAtTheCrossing`]), the hand-over where it
//! crosses both bands of a mitre ([`super::SurfaceCrossingAtTheMitre`]), and the seams that have to
//! stop at the same place the trim does.
//!
//! Kept apart from `SurfaceBevelBuilder` because it is the one part of the build that is about two
//! chains at once: everything else there walks one plan's edges and corners in order.

use std::rc::Rc;

use solidmesh_math::Vector3;

use super::surface_mitre_joint_step::SideSeams;
use super::{
    SurfaceBevelPlan, SurfaceCrossingAtTheMitre, SurfaceEdgePlan, SurfaceMixedCorner,
    SurfaceMixedCornerWall, SurfaceRoomCrossing, SurfaceStrip, SurfaceStripTrimmedAtTheCrossing,
    SurfaceStripTrims, SurfaceVertexPool,
};
use crate::bevel::{BevelAdvisory, BevelAdvisoryCode};

/// Which strip this corner's run-out crosses on the CAP it runs across
/// and which on the WALL it turns against, `(crossed, crossed_on_the_wall)`, each -1 where it
/// crosses none. BOTH is the mitred pair, which is two curves meeting at one point rather than one
/// curve, and is read by [`try_the_hand_over_at_the_mitre`]. Never refuses; it answers a `Result`
/// so its callers read it as they read every other step.
#[allow(clippy::too_many_arguments)]
pub fn try_the_band_crossed_by(
    crossings: Option<&[SurfaceRoomCrossing]>,
    plans: &[Rc<SurfaceBevelPlan>],
    first_of_plan: &[i32],
    plan: i32,
    corner_key: i32,
    count: i32,
    _corner: &SurfaceMixedCorner,
) -> Result<(i32, i32), BevelAdvisory> {
    let crossed = the_band_crossed_by(
        crossings,
        plans,
        first_of_plan,
        plan,
        corner_key,
        count,
        false,
    );
    let crossed_on_the_wall = the_band_crossed_by(
        crossings,
        plans,
        first_of_plan,
        plan,
        corner_key,
        count,
        true,
    );

    Ok((crossed, crossed_on_the_wall))
}

/// Where a run-out crosses BOTH bands at once, the corner those two
/// bands MITRE at - and the hand-over on it, which is the one point all three surfaces share and
/// the turn the run-out's window shuts at.
///
/// The two have to be consecutive edges of one chain, meeting end to end: a band on the cap and a
/// band on the wall that are not each other's neighbour stand at two different corners of the
/// part, and there is no one ring for the crossing to hand over on. Which of the two arrives and
/// which leaves is read off their own ends rather than off which face each lies in.
#[allow(clippy::too_many_arguments)]
pub fn try_the_hand_over_at_the_mitre(
    pool: &mut SurfaceVertexPool,
    strips: &[Option<Rc<SurfaceStrip>>],
    edges: &[SurfaceEdgePlan],
    crossed: i32,
    crossed_on_the_wall: i32,
    corner: &SurfaceMixedCorner,
    wall: &SurfaceMixedCornerWall,
) -> Result<Rc<SurfaceCrossingAtTheMitre>, BevelAdvisory> {
    let mut meet_at = Vector3::ZERO;
    let mut meet = false;

    for (first, second) in [
        (crossed, crossed_on_the_wall),
        (crossed_on_the_wall, crossed),
    ] {
        if (edges[first as usize].to() - edges[second as usize].from()).length()
            <= pool.near_enough()
        {
            meet_at = edges[first as usize].to();
            meet = true;
        }
    }

    if !meet {
        // Two bands the run-out lies over that are not one corner's pair: nothing they share is a
        // ring, so there is no hand-over to read and no single point to close both curves on.
        return Err(BevelAdvisory::new(
            BevelAdvisoryCode::SurfaceCrossingMissesTheMitre,
            vec![corner.corner.into()],
        ));
    }

    let at_the_mitre = SurfaceCrossingAtTheMitre::try_read(
        corner,
        wall,
        strips[crossed as usize].as_deref(),
        strips[crossed_on_the_wall as usize].as_deref(),
        meet_at,
        pool.near_enough(),
    )?;

    // Pooled ONCE, here, before either curve is drawn: both staircases close on this vertex and the
    // run-out's own last column is it, so a second Add anywhere downstream would be a second point
    // a weld apart from the first with a sliver between them.
    at_the_mitre.vertex.set(pool.add(at_the_mitre.at));
    at_the_mitre.the_cap_band.set(crossed);
    at_the_mitre.the_wall_band.set(crossed_on_the_wall);

    Ok(Rc::new(at_the_mitre))
}

/// The crossed band cut back to the curve, or the reason it could not be:
/// a band crossed by TWO corners' run-outs would have to be cut at two curves at once, which is
/// named rather than left to the later crossing to overwrite the earlier. `crossing` is the curve
/// the corner drew on that strip, `None` where it crossed nothing.
#[allow(clippy::too_many_arguments)]
pub fn try_cut_the_band_back(
    pool: &SurfaceVertexPool,
    strips: &[Option<Rc<SurfaceStrip>>],
    edges: &[SurfaceEdgePlan],
    trimmed: &mut SurfaceStripTrims,
    crossed: i32,
    on_the_wall: bool,
    the_die_point: i32,
    crossing: Option<&[i32]>,
    at_the_mitre: Option<Rc<SurfaceCrossingAtTheMitre>>,
    corner: &SurfaceMixedCorner,
) -> Result<(), BevelAdvisory> {
    let Some(crossing) = crossing else {
        return Ok(());
    };

    if crossed < 0 {
        return Ok(());
    }

    let trim = SurfaceStripTrimmedAtTheCrossing::try_read(
        Some(pool),
        strips[crossed as usize].as_deref(),
        crossed,
        &edges[crossed as usize],
        Some(crossing),
        at_the_mitre,
    )?;

    // No trim is read only where the strip is gone or the curve has fewer than two corners, which
    // no crossing the build hands over has: a crossing comes back from the patch only where the
    // run-out met this band's strip, and it is a column of the run-out's profile. A guard, so a
    // future caller cannot reach a panic: that state cuts nothing and refuses nothing.
    let Some(trim) = trim else {
        return Ok(());
    };

    if SurfaceStripTrimmedAtTheCrossing::the_band_is_already_crossed(
        Some(trimmed),
        crossed,
        trim.at_the_start,
    ) {
        // Two corners' run-outs on ONE END of one band, which is a staircase cut at two curves at
        // once: each crossing was read against the whole of that end, so the second stands on
        // material the first already took. The two ENDS of one band are another matter - a band
        // running between two corners is crossed at each of them, and each end closes on its own
        // curve as it would on its own face.
        return Err(BevelAdvisory::new(
            BevelAdvisoryCode::SurfaceCrossingsShareOneBand,
            vec![corner.corner.into()],
        ));
    }

    trim.the_foot_is_on_the_rim.set(on_the_wall);
    trim.the_die_vertex.set(the_die_point);

    let end = if trim.at_the_start { 0 } else { 1 };
    trimmed[crossed as usize][end] = Some(Rc::new(trim));

    Ok(())
}

/// Which strip a corner's run-out crosses, or -1: the room reading names
/// the band by its plan, its edge and the corner the skirt came out of, so nothing here is
/// recognised by where it stands.
#[allow(clippy::too_many_arguments)]
fn the_band_crossed_by(
    crossings: Option<&[SurfaceRoomCrossing]>,
    plans: &[Rc<SurfaceBevelPlan>],
    first_of_plan: &[i32],
    plan: i32,
    corner_key: i32,
    count: i32,
    on_the_wall: bool,
) -> i32 {
    let Some(crossings) = crossings else {
        return -1;
    };

    for crossing in crossings {
        if crossing.skirt_plan_index() != plan
            || crossing.skirt_corner_key() != corner_key
            // On the face asked for: a band crossed on the cap ends the run-out's rows and a band
            // crossed on the wall ends its columns, and the two are never the same band.
            || crossing.stands_on_the_wall() != on_the_wall
            || crossing.band_plan_index() < 0
            || crossing.band_plan_index() as usize >= plans.len()
            || crossing.band_edge_index() < 0
            || crossing.band_edge_index() as usize
                >= plans[crossing.band_plan_index() as usize].edges.len()
        {
            continue;
        }

        let band = first_of_plan[crossing.band_plan_index() as usize] + crossing.band_edge_index();

        if band >= 0 && band < count {
            return band;
        }
    }

    -1
}

/// Every crossed strip's seam cut back to the crossing itself, so
/// the last vertex the stitch has to reach on that tangent line is the crossing's own.
pub fn trim_the_seams_at_the_crossings(
    strips: &[Option<Rc<SurfaceStrip>>],
    trimmed: &SurfaceStripTrims,
    seams: &mut [SideSeams],
) {
    // A row-major walk of the [strip, end] table: strip by strip, its start before its end.
    for (strip, ends) in trimmed.iter().enumerate() {
        for crossing in ends.iter().flatten() {
            for side in [0usize, 1] {
                // A trimmed strip always has its strip; a missing one is skipped rather than panic.
                let Some(the_strip) = strips[strip].as_deref() else {
                    continue;
                };
                let rail = if side == 0 {
                    0
                } else {
                    the_strip.rail_count() - 1
                };

                let Some(seam) = seams[strip][side].as_mut() else {
                    continue;
                };

                if !crossing.crosses(rail) {
                    continue;
                }

                // Both present, since the crossing crosses this rail.
                let (Some(cut), Some(vertex)) = (crossing.param_on(rail), crossing.vertex_on(rail))
                else {
                    continue;
                };
                let mut kept: Vec<(i32, f64)> = Vec::with_capacity(seam.len());

                for &one in seam.iter() {
                    // Only what stands on the TRIMMED side of the cut comes out, and nothing is
                    // allowed for by a width: a later split the cap face still uses can stand a
                    // hair on the KEPT side of the crossing, and dropping it for being near would
                    // leave that face with a vertex part way along an edge the strip no longer sews
                    // to. The crossing itself is told by its VERTEX rather than by how near its
                    // parameter stands, so it comes out here and goes back in below however the
                    // seam already carried it.
                    if one.0 == vertex {
                        continue;
                    }

                    if if crossing.at_the_start {
                        one.1 > cut
                    } else {
                        one.1 < cut
                    } {
                        kept.push(one);
                    }
                }

                if crossing.at_the_start {
                    kept.insert(0, (vertex, cut));
                } else {
                    kept.push((vertex, cut));
                }

                *seam = kept;
            }
        }
    }
}
