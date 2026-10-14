//! The two faces of a feature that BIT a strip, each carried on past the landing face it used to
//! die on and trimmed against the round - the other half of the hole
//! [`super::SurfaceStripBiteRows`] leaves in the strip. Free functions.
//!
//! The same ruling as [`super::surface_standing_feature_faces`], which does this where the feature
//! stands under an END of the band, and a great deal less work than it: a feature standing part way
//! ALONG the band has its crease cut by the band's own tangent line, so the walkback has already
//! split that crease at the very point the hole's corner stands - both faces arrive with that
//! corner in their outlines, next to the ridge end. So the extension replaces ONE side of the
//! outline, the stretch from the seam corner to the ridge end, by the trace it now runs out on: the
//! seam corner, the hole's rim rail by rail, and the tip. Nothing is minted and nothing is searched
//! for.
//!
//! **The old ridge end goes.** The ridge runs PAST it out to the tip, so the extended ridge is one
//! straight edge shared by the two faces the other way round, and carrying the old end as a corner
//! as well puts that edge on four faces rather than two. It is kept only where something else
//! already stands on it, which would be a T junction if it went.

use std::rc::Rc;

use vector_math::Vector3;

use super::surface_fan::emit_facing;
use super::surface_ring_ears::try_triangulate_the_ring;
use super::surface_standing_feature_faces::{is_carried, no_apex_covers_it};
use super::{
    SurfaceStandingFeatureWithheld, SurfaceStandingFeatureWithheldFace, SurfaceStripBiteRows,
    SurfaceStripBittenByAFeature, SurfaceVertexPool,
};
use crate::bevel::{BevelAdvisory, BevelAdvisoryCode};

type Face = SurfaceStandingFeatureWithheldFace;

/// Emits every face of one bitten feature ONCE, extended out to EVERY hole bitten out
/// of it.
///
/// Every bite of the feature together, and not one call per bite, because what is emitted is the
/// FACE: a feature two bands undercut from opposite ends has one upper face and one lower one, each
/// running out to a lens tip at each end. The replacements are applied SEQUENTIALLY to one ring:
/// each takes the side of the outline running from its own hole's seam corner to its own ridge end,
/// and the two sides are at opposite ends of the feature, so neither can see the other's work.
pub fn try_emit(
    pool: &mut SurfaceVertexPool,
    faces: &mut Vec<(i32, i32, i32)>,
    feature: &SurfaceStandingFeatureWithheld,
    bites: &[(&SurfaceStripBittenByAFeature, &SurfaceStripBiteRows)],
) -> Result<(), BevelAdvisory> {
    for face in &feature.faces {
        // Its own outline to start with, which is what a face NO bite touched goes in as. Held back
        // all the same - the walkback holds a feature back as one thing, since its faces share a
        // ridge - and dropped instead it leaves the feature open along that ridge. (The walkback
        // has taken every withheld face's outline before any bite is emitted.)
        let mut ring = face.ring.borrow().clone().unwrap_or_default();

        for &(bite, rows) in bites {
            let draws_the_near_side = Rc::ptr_eq(face, &bite.near_face);

            if !draws_the_near_side && !Rc::ptr_eq(face, &bite.far_face) {
                // Neither side of THIS hole is this face's, so this bite took nothing off it.
                continue;
            }

            ring = try_the_ring(pool, faces, bite, rows, face, &ring, draws_the_near_side)?;
        }

        emit(pool, faces, &ring, face.plane_normal)?;
    }

    Ok(())
}

/// One face's ring - its own outline with the side running from the hole's corner
/// on the seam to the feature's ridge end replaced by the trace the extension now runs out on.
fn try_the_ring(
    pool: &SurfaceVertexPool,
    faces: &[(i32, i32, i32)],
    bite: &SurfaceStripBittenByAFeature,
    rows: &SurfaceStripBiteRows,
    face: &Face,
    outline: &[i32],
    draws_the_near_side: bool,
) -> Result<Vec<i32>, BevelAdvisory> {
    let corner = if draws_the_near_side {
        bite.near_seam_vertex
    } else {
        bite.far_seam_vertex
    };

    let Some((on_the_ridge, _)) = bite
        .feature
        .try_the_ends_of_the_crease(pool, face, bite.reach)
    else {
        // Neither end of the crease stands on the feature's own ridge, so there is no telling which
        // way the trace runs out of the part.
        return Err(does_not_read(
            pool.position_of(face.creases.borrow()[bite.reach as usize].0),
        ));
    };

    let count = outline.len();
    let at = outline.iter().position(|&v| v == corner);
    let ridge_at = outline.iter().position(|&v| v == on_the_ridge);

    let (Some(at), Some(ridge_at)) = (at, ridge_at) else {
        return Err(does_not_read(pool.position_of(corner)));
    };

    if (ridge_at + 1) % count != at && (at + 1) % count != ridge_at {
        // The band's own tangent line cut this crease, so the walkback left the hole's corner
        // standing on the outline right beside the ridge end. Anything else - the corner missing,
        // or something else standing between the two - is a reading of one crease this cannot
        // replace one side by.
        return Err(does_not_read(pool.position_of(corner)));
    }

    // Which way round the outline the side goes, because the trace stands in its place and has to
    // run the same way: laid the other way it winds the extension backwards.
    let ridge_first = (ridge_at + 1) % count == at;

    // The trace, read SEAM end first: the hole's corner, its rim rail by rail inward from the seam,
    // and the tip the ridge runs out through. The corner itself is left to the outline, which
    // already carries it - the walkback split the crease there.
    let mut trace = Vec::with_capacity(bite.rail_of.len() + 1);

    for step in 1..bite.rail_of.len() {
        let rail = bite.rail_of[step] as usize;

        trace.push(if draws_the_near_side {
            rows.near_at[rail]
        } else {
            rows.far_at[rail]
        });
    }

    trace.push(bite.tip.vertex);

    if ridge_first {
        trace.reverse();
    }

    // The outline with the ridge end left out, walked from the corner that follows the side the
    // trace replaces - which is the hole's own corner where the ridge comes first, and the corner
    // after the ridge end where it does not.
    let mut ring = Vec::with_capacity(count + trace.len());

    let from = if ridge_first {
        at
    } else {
        (ridge_at + 1) % count
    };

    for step in 0..count.saturating_sub(1) {
        ring.push(outline[(from + step) % count]);
    }

    // Something the band did NOT take may still stand on the old ridge end, and dropping it then
    // would leave a T junction in the middle of the feature's own ridge. Kept where it does,
    // standing part way along the straight edge the extended ridge now is - on the side of the
    // trace the outline's own direction puts it.
    let carried = is_carried(faces, on_the_ridge);

    if ridge_first && carried {
        ring.push(on_the_ridge);
    }

    ring.extend(trace);

    if !ridge_first && carried {
        ring.push(on_the_ridge);
    }

    Ok(ring)
}

/// One ring, fanned the way the face faces - or cut into ears where no corner of it sees
/// the whole.
fn emit(
    pool: &mut SurfaceVertexPool,
    faces: &mut Vec<(i32, i32, i32)>,
    ring: &[i32],
    facing: Vector3,
) -> Result<(), BevelAdvisory> {
    if !no_apex_covers_it(ring, pool, facing) {
        emit_facing(ring, pool, faces, facing);
        return Ok(());
    }

    // The extended face CAVES IN - the trace it runs out on bends back inside it - so no corner
    // sees the whole of it and a fan laid from any of them covers part of it twice. Ears cover it
    // once instead, which is a construction and not a repair: nothing is deleted and no point is
    // moved.
    if !try_triangulate_the_ring(ring, pool, faces, facing, false) {
        return Err(BevelAdvisory::new(
            BevelAdvisoryCode::SurfaceStandingFeatureExtensionFolds,
            vec![pool.position_of(ring[0]).into()],
        ));
    }

    Ok(())
}

/// The advisory a bite that does not read is named with.
fn does_not_read(at: Vector3) -> BevelAdvisory {
    BevelAdvisory::new(BevelAdvisoryCode::SurfaceBiteDoesNotRead, vec![at.into()])
}
