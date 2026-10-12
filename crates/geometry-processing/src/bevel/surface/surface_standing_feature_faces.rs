//! A feature standing inside a band's footprint, EXTENDED through the corner the band took: each
//! of its withheld faces emitted as ONE ring - the whole coplanar REGION as the part had it, with
//! the crease run it lost its footing on replaced by the trace it now runs out on. Ports
//! `Bevel/Surface/SurfaceStandingFeatureFaces.cs` (C# `internal static class`; free functions
//! here): [`try_emit`], [`no_apex_covers_it`], [`is_carried`] and the ring readings they are
//! built from. The notch it fills is `SurfaceStripNotchedByAFeature`'s.
//!
//! Lars's ruling, and what the Onshape control builds. The face used to die on a crease standing on
//! the landing face; the band removed the material under that crease, so the face is carried on in
//! its own plane until it meets the fillet. The old crease becomes INTERIOR to the extended face,
//! which is why the face and its extension go out as one ring and not as two faces meeting along
//! it - two faces would share an edge nothing else in the part stands on.
//!
//! **The ring.** The region's own outline - the boundary of the union of its facets, so that the
//! triangulation's diagonals are no edges of the result - walked from one end of the crease run
//! the long way round the face and then back along the trace: the crease's ridge end, the ridge's
//! own exit from the strip, the rail crossings the notch read, the Q where the trace meets the
//! band's end ring, and the crease's other end, which stands in that end plane already
//! (`SurfaceStripNotchedByAFeature`). Every one of those points is the notch's own, so the
//! extension and the hole it fills are bounded by the same vertices and there is nothing to weld.
//!
//! **Fanned by `surface_fan::emit_facing`** rather than from a corner picked here. The ring has
//! straight sides with corners standing ON them - the vertex opposite the crease, the crease's end
//! on the band's end plane and the Q are all three on the line where the face's plane meets that
//! end plane, so a fan from any of them lays a triangle covering nothing - and the fan's own apex
//! rule is what keeps such a triple's edges used once each instead of putting a diagonal along a
//! side.

use std::rc::Rc;

use vector_math::Vector3;

use super::surface_fan::emit_facing;
use super::surface_ring_ears::try_triangulate_the_ring;
use super::{
    SurfaceStandingFeatureWithheldFace, SurfaceStrip, SurfaceStripNotchedByAFeature,
    SurfaceVertexPool,
};
use crate::bevel::{BevelAdvisory, BevelAdvisoryCode};

type Face = SurfaceStandingFeatureWithheldFace;

/// C# `TryEmit`: emits every face of one notched feature, extended out to the strip.
pub fn try_emit(
    pool: &mut SurfaceVertexPool,
    faces: &mut Vec<(i32, i32, i32)>,
    strip: &SurfaceStrip,
    notch: &SurfaceStripNotchedByAFeature,
) -> Result<(), BevelAdvisory> {
    for face in &notch.feature.faces {
        let ring = if !notch.face_of.iter().any(|of| Rc::ptr_eq(of, face)) {
            // Not cut by this notch at all: the band ran under the feature's OTHER side only, and
            // this face still stands on the whole of its own crease. It was held back all the same
            // - the walkback holds a feature back as one thing, since its faces share a ridge - so
            // it goes in exactly as the part had it. Dropped instead, which is what this did while
            // every size read two faces, it leaves the feature open along its own ridge. (The
            // walkback has taken every withheld face's outline before any notch is emitted.)
            face.ring.borrow().clone().unwrap_or_default()
        } else {
            try_the_ring(pool, faces, strip, notch, face)?
        };

        if no_apex_covers_it(&ring, pool, face.plane_normal) {
            // The extended face CAVES IN - the trace it runs out on bends back inside the face,
            // which is what fifteen millimetres does to the nose's lower wall - so no corner of it
            // sees the whole of it and a fan laid from any of them covers part of it twice. The fan
            // will not say so itself: it winds every triangle it lays to the normal it is handed,
            // so a backwards triangle comes out forwards and doubled. Ears cover it once instead
            // (SurfaceRingEars), which is a construction and not a repair - nothing is deleted and
            // no point is moved.
            if !try_triangulate_the_ring(&ring, pool, faces, face.plane_normal, false) {
                // No triangulation to find: the ring crosses itself, so the face's own outline and
                // the trace it leaves on are not one simple wall at all, and what would be
                // published is the fold the advisory is named for.
                return Err(BevelAdvisory::new(
                    BevelAdvisoryCode::SurfaceStandingFeatureExtensionFolds,
                    vec![pool.position_of(ring[0]).into()],
                ));
            }

            continue;
        }

        emit_facing(&ring, pool, faces, face.plane_normal);
    }

    Ok(())
}

/// One refusal by name, at one point.
fn does_not_extend(at: Vector3) -> BevelAdvisory {
    BevelAdvisory::new(
        BevelAdvisoryCode::SurfaceStandingFeatureDoesNotExtend,
        vec![at.into()],
    )
}

/// C# `TryTheRing`: one face's ring - round the face the long way from the crease's END-PLANE end
/// to its RIDGE end, then back along the trace the extension runs out on.
fn try_the_ring(
    pool: &mut SurfaceVertexPool,
    faces: &[(i32, i32, i32)],
    strip: &SurfaceStrip,
    notch: &SurfaceStripNotchedByAFeature,
    face: &Rc<Face>,
) -> Result<Vec<i32>, BevelAdvisory> {
    let reach = notch.reach as usize;

    // Which end of the crease stands on the feature's own ridge - that is where the trace starts,
    // because the ridge is what carries on out of the part into the exit.
    let Some((on_the_ridge, on_the_end)) =
        notch
            .feature
            .try_the_ends_of_the_crease(pool, face, notch.reach)
    else {
        return Err(does_not_extend(
            pool.position_of(face.creases.borrow()[reach].0),
        ));
    };

    if notch.is_one_faced() {
        return try_the_one_faced_ring(pool, strip, notch, face, on_the_ridge, on_the_end);
    }

    let outline = face.ring.borrow().clone().unwrap_or_default();
    let within = face.withins.borrow()[reach].clone();

    // Which way the crease runs in the region's OWN outline, because the trace replaces that side
    // of it and has to run the same way round: a path laid in the other direction winds the
    // extension backwards and the fan reads every triangle of it as a fold.
    let ridge_first = the_crease_run_goes(&outline, on_the_ridge, on_the_end, &within);

    if !ridge_first && !the_crease_run_goes(&outline, on_the_end, on_the_ridge, &within) {
        // The crease run is not one unbroken side of the outline - the walkback put a crossing in
        // the middle of it, or the region stands on it in two places - so there is no one side for
        // the trace to stand in place of.
        return Err(does_not_extend(pool.position_of(on_the_ridge)));
    }

    let Some(mut kept) = the_face_without_its_crease(
        &outline,
        if ridge_first {
            on_the_end
        } else {
            on_the_ridge
        },
        if ridge_first {
            on_the_ridge
        } else {
            on_the_end
        },
    ) else {
        return Err(does_not_extend(pool.position_of(on_the_ridge)));
    };

    // The trace itself, always read ridge-end first: the exit comes off the ridge, then the rail
    // crossings outward, then the Q on the band's end ring.
    let runs_up = the_trace_runs_up(notch, face);
    let mut trace = the_trace(
        pool,
        strip,
        notch,
        face,
        notch.exit.below + if runs_up { 1 } else { 0 },
        runs_up,
    );

    if !ridge_first {
        trace.reverse();
    }

    // The old ridge END is no corner any more where the ridge runs PAST it out to the exit: the
    // extended ridge is one straight edge from the vertex opposite the crease to the exit, shared
    // by the feature's two faces the other way round, and carrying the old end as a ring corner as
    // well puts that edge on four faces rather than two. It is left in only where the exit IS that
    // point, which is what the smaller sizes read (at fifteen millimetres the ridge runs 1.24 past
    // it).
    if (pool.position_of(notch.exit.vertex) - pool.position_of(on_the_ridge)).length()
        > SurfaceVertexPool::near_enough_for(pool.vertices())
    {
        if is_carried(faces, on_the_ridge) {
            // Something the band did NOT take still stands on that point, so dropping it would
            // leave a T junction in the middle of the feature's own ridge. Named rather than left.
            return Err(does_not_extend(pool.position_of(on_the_ridge)));
        }

        // C#'s `List.Remove`: the first occurrence only.
        if let Some(at) = kept.iter().position(|&vertex| vertex == on_the_ridge) {
            kept.remove(at);
        }
    }

    kept.extend(trace);

    Ok(kept)
}

/// C# `NoApexCoversIt`: whether NO corner of a ring sees the whole of it - which is the ring a fan
/// cannot cover once.
///
/// Asked by trying every corner as the apex and reading the sign of each triangle the fan would
/// lay against the plane the ring is to face. A corner that gives every one of them a forward face
/// sees the whole ring; a ring with no such corner is not star shaped from any of its own corners,
/// and `surface_fan` falls back to the ring's middle - which for a caved-in ring stands outside
/// it. Triangles of no area are let through: a corner standing part way along a straight side
/// makes one, and those are the very points the fan exists to keep.
pub fn no_apex_covers_it(ring: &[i32], pool: &SurfaceVertexPool, facing: Vector3) -> bool {
    if ring.len() < 3 {
        return false;
    }

    for apex in 0..ring.len() {
        let mut covers = true;

        let mut step = 1;
        while step + 1 < ring.len() && covers {
            let first = pool.position_of(ring[apex]);
            let second = pool.position_of(ring[(apex + step) % ring.len()]);
            let third = pool.position_of(ring[(apex + step + 1) % ring.len()]);

            covers = (second - first).cross(third - first).dot(facing) >= 0.0;
            step += 1;
        }

        if covers {
            return false;
        }
    }

    true
}

/// C# `TryTheOneFacedRing`: the extended face's ring where the band undercut ONE of the feature's
/// faces - the region's own outline with the stretch of crease the band took (from the crease's
/// CROSSING out to its end in the band's end plane) replaced by the trace, and the rest of the
/// crease left standing.
///
/// The difference from the two-faced ring is that the crease is only PART gone. The band reached
/// the crease's end-plane end and ran back along it as far as the crossing; from there to the
/// ridge end the landing face is still under it. So the crossing is a corner of the ring like any
/// other, the crease's remainder is one of its sides, and the ridge end stays exactly where it
/// was: there is no ridge running past it out to an exit here, because the band never reached the
/// ridge.
fn try_the_one_faced_ring(
    pool: &mut SurfaceVertexPool,
    strip: &SurfaceStrip,
    notch: &SurfaceStripNotchedByAFeature,
    face: &Rc<Face>,
    on_the_ridge: i32,
    on_the_end: i32,
) -> Result<Vec<i32>, BevelAdvisory> {
    let crossing = notch.exit.vertex;
    let runs_up = notch.the_crease_rail == 0;
    let mut trace = the_trace(
        pool,
        strip,
        notch,
        face,
        notch.the_crease_rail + if runs_up { 1 } else { -1 },
        runs_up,
    );

    let outline = face.ring.borrow().clone().unwrap_or_default();

    // Which way round the outline the stretch the band TOOK runs. The walkback split the crease
    // edge at the crossing when it cut the landing face, so that point is already a corner of this
    // region's outline and the stretch is one step of it - or a few, where the crease is several
    // mesh edges long.
    let most_steps = face.withins.borrow()[notch.reach as usize].len() + 1;

    let ring = if the_run_reaches(&outline, on_the_end, crossing, most_steps) {
        let ring = the_face_without_its_crease(&outline, crossing, on_the_end);

        trace.reverse();
        trace.pop();

        ring
    } else if the_run_reaches(&outline, crossing, on_the_end, most_steps) {
        let ring = the_face_without_its_crease(&outline, on_the_end, crossing);

        trace.remove(0);

        ring
    } else {
        // The crossing and the crease's end-plane end are not one side of the outline, so there is
        // no one stretch for the trace to stand in place of.
        return Err(does_not_extend(pool.position_of(crossing)));
    };

    let Some(mut ring) = ring else {
        return Err(does_not_extend(pool.position_of(on_the_ridge)));
    };

    ring.extend(trace);

    Ok(ring)
}

/// C# `TheRunReaches`: whether one corner of an outline is reached from another within so many
/// steps.
fn the_run_reaches(outline: &[i32], from: i32, to: i32, most_steps: usize) -> bool {
    let Some(at) = outline.iter().position(|&vertex| vertex == from) else {
        return false;
    };

    (1..=most_steps).any(|step| outline[(at + step) % outline.len()] == to)
}

/// Whether the face a `Between` carries is `face` itself (C#'s `ReferenceEquals`).
fn is_the_face(carried: &Option<Rc<Face>>, face: &Rc<Face>) -> bool {
    carried
        .as_ref()
        .is_some_and(|carried| Rc::ptr_eq(carried, face))
}

/// C# `TheTraceRunsUp`: which way along the rails a two-faced notch's trace runs out from its exit.
fn the_trace_runs_up(notch: &SurfaceStripNotchedByAFeature, face: &Rc<Face>) -> bool {
    let q = if is_the_face(&notch.at_the_low_rail.face, face) {
        &notch.at_the_low_rail
    } else {
        &notch.at_the_high_rail
    };

    q.below >= notch.exit.below
}

/// C# `TheTrace`: the trace one face's extension runs out on, inner end first - the notch's own
/// inner point, then that face's rail crossings outward, then the Q where it meets the band's end
/// ring.
fn the_trace(
    pool: &mut SurfaceVertexPool,
    strip: &SurfaceStrip,
    notch: &SurfaceStripNotchedByAFeature,
    face: &Rc<Face>,
    first_rail: i32,
    runs_up: bool,
) -> Vec<i32> {
    let mut trace = vec![notch.exit.vertex];

    let mut rail = first_rail;
    while rail >= 0 && rail < strip.rail_count() {
        // `param_on` answers for every rail `notches` does, so the `None` arm is the same break.
        let Some(index) = notch.rail_of.iter().position(|&on| on == rail) else {
            break;
        };

        if !Rc::ptr_eq(&notch.face_of[index], face) {
            break;
        }

        let Some(param) = notch.param_on(rail) else {
            break;
        };

        trace.push(share(pool, strip.point_on(rail, param)));
        rail += if runs_up { 1 } else { -1 };
    }

    trace.push(if is_the_face(&notch.at_the_low_rail.face, face) {
        notch.at_the_low_rail.vertex
    } else {
        notch.at_the_high_rail.vertex
    });

    trace
}

/// C# `IsCarried`: whether anything already emitted stands on one vertex - asked before a corner
/// the extension no longer needs is dropped, since a corner something else still names is a T
/// junction if it goes.
pub fn is_carried(faces: &[(i32, i32, i32)], vertex: i32) -> bool {
    faces
        .iter()
        .any(|face| face.0 == vertex || face.1 == vertex || face.2 == vertex)
}

/// C# `TheCreaseRunGoes`: whether the crease RUN goes this way round the outline - from `first`
/// onward through the run's own between points (`within`), and through nothing else, to `second`.
///
/// A run rather than an edge because a face whose root line is cut into several mesh edges stands
/// on several creases, and all of them are replaced by the one trace. With no between points this
/// is the plain question of whether one vertex follows the other.
fn the_crease_run_goes(ring: &[i32], first: i32, second: i32, within: &[i32]) -> bool {
    let Some(at) = ring.iter().position(|&vertex| vertex == first) else {
        return false;
    };

    for step in 1..=within.len() + 1 {
        let next = ring[(at + step) % ring.len()];

        if next == second {
            // Every between point has to have been walked over on the way, or what is left of the
            // run stands somewhere else on the outline entirely.
            return step == within.len() + 1;
        }

        if !within.contains(&next) {
            return false;
        }
    }

    false
}

/// C# `TheFaceWithoutItsCrease`: the region's own outline walked from `from` the LONG way to `to` -
/// the way that goes round the far side of the face - so the crease run itself, and every between
/// point of it, is left off and becomes interior to the extended face. `None` where the two are not
/// both on the ring.
fn the_face_without_its_crease(outline: &[i32], from: i32, to: i32) -> Option<Vec<i32>> {
    let start = outline.iter().position(|&vertex| vertex == from)?;
    let finish = outline.iter().position(|&vertex| vertex == to)?;

    if start == finish {
        return None;
    }

    let mut walked = Vec::new();

    for step in 0..outline.len() {
        let at = outline[(start + step) % outline.len()];

        walked.push(at);

        if at == to {
            return Some(walked);
        }
    }

    None
}

/// C# `Share`: the part's own vertex at one point where it has one, and a new one only where it
/// has not.
fn share(pool: &mut SurfaceVertexPool, at: Vector3) -> i32 {
    let near = SurfaceVertexPool::near_enough_for(pool.vertices());

    match pool.try_find_near(at, near, None) {
        Some(found) => found,
        None => pool.add(at),
    }
}
