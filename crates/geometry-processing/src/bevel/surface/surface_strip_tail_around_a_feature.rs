//! The tail of a band's end that a standing feature's extension notches: where the strip stops,
//! clear of the notch, and the rings that close what lies past that cross section. Ports
//! `Bevel/Surface/SurfaceStripTailAroundAFeature.cs` (C# `internal static class`; a module of free
//! functions here). What is here are the cross section ([`try_the_section`],
//! [`try_the_section_clear_of_the_notch`]), the rings past it ([`try_the_rings`]) and the sewing
//! itself ([`try_sew`]). Every corner of a ring is a vertex already in the
//! pool - a corner that cannot be found is a refusal by name, never a new vertex.

use super::{
    surface_fan, surface_ring, SurfaceStrip, SurfaceStripNotchedByAFeature,
    SurfaceStripTrimmedAtTheCrossing, SurfaceVertexPool,
};
use crate::bevel::{BevelAdvisory, BevelAdvisoryCode};
use vector_math::Vector3;

/// C# `TryTheSection`: where the strip STOPS - a parameter standing strictly clear of the notch's
/// deepest point (`exit`) on one side and of everything the strip's OTHER end reaches on the other,
/// which is half way between the two because that is the furthest it can stand from both at once.
///
/// Pure, and measured against EVERY other end the strip has rather than against the run-off's
/// staircase alone: a band that is also trimmed at a crossing, or notched a second time, reaches
/// further in than the staircase does, and a section past that has the strip stitched backwards
/// through itself. `None` where there is no room left between the two at all.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
pub fn try_the_section(exit: f64, the_other_end: &[f64], near: f64) -> Option<f64> {
    let first = *the_other_end.first()?;

    // Which way the notch faces is which side of it the other end is on, and every one of those
    // parameters has to be on that same side or there is no window between them.
    let runs_up = first < exit;
    let mut furthest = first;

    for &at in the_other_end {
        if runs_up != (at < exit) {
            return None;
        }

        furthest = if runs_up {
            furthest.max(at)
        } else {
            furthest.min(at)
        };
    }

    if !((exit - furthest).abs() > near * 2.0) {
        return None;
    }

    Some((furthest + exit) / 2.0)
}

/// C# `TryTheSectionClearOfTheNotch`: where the strip stops, read for one notched END - the guards
/// that say the notch is a single hole with the exit at the bottom of it, and then
/// [`try_the_section`] against everything the strip's OTHER end reaches (`the_other_end_on`, rail by
/// rail, plus a crossing's own parameter where another chain's run-out took the same rails).
///
/// Lifted out of the run-off's own end so that a LANDING end can stop at the same cross section by
/// the same statement rather than by a copy of it.
pub fn try_the_section_clear_of_the_notch(
    pool: &SurfaceVertexPool,
    strip: &SurfaceStrip,
    notch: &SurfaceStripNotchedByAFeature,
    notched_at_the_other_end: Option<&SurfaceStripNotchedByAFeature>,
    crossed_at_the_other_end: Option<&SurfaceStripTrimmedAtTheCrossing>,
    the_other_end_on: &[f64],
) -> Result<f64, BevelAdvisory> {
    let does_not_extend = |rail: i32, param: f64| {
        BevelAdvisory::new(
            BevelAdvisoryCode::SurfaceStandingFeatureDoesNotExtend,
            vec![strip.point_on(rail, param).into()],
        )
    };

    if notched_at_the_other_end.is_some() {
        // A second feature notching the SAME strip at its other end. Each wants a cross section clear
        // of its own notch, and the strip has only one of them to give. Named rather than built on
        // whichever was read first.
        return Err(does_not_extend(notch.rail_of[0], notch.exit.param));
    }

    // That the exit really is the deepest point of the notch is what lets one cross section stand
    // clear of the whole of it. Measured rather than assumed: a rail cut back further than the exit
    // would leave its own crossing BEHIND the cross section, and the band past it would double over.
    let deepest = notch.exit.param.min(notch.end_param);
    let shallowest = notch.exit.param.max(notch.end_param);

    let mut the_other_end = Vec::with_capacity(notch.rail_of.len() * 2);

    for (index, &rail) in notch.rail_of.iter().enumerate() {
        // C# `ParamOn(rail)`: the first entry for this rail, which is this one or an earlier one.
        let param = notch.param_on(rail).unwrap_or(notch.param_of[index]);

        if param < deepest - pool.near_enough() || param > shallowest + pool.near_enough() {
            return Err(does_not_extend(rail, param));
        }

        the_other_end.push(the_other_end_on[rail as usize]);

        if let Some(crossed) = crossed_at_the_other_end.and_then(|crossed| crossed.param_on(rail)) {
            the_other_end.push(crossed);
        }
    }

    // The notch reaches back as far as the strip's other end, or past it, so there is no cross
    // section between the two and no strip left to stitch.
    try_the_section(notch.exit.param, &the_other_end, pool.near_enough())
        .ok_or_else(|| does_not_extend(notch.rail_of[0], notch.exit.param))
}

/// C# `TryTheRings`: the rings that close what lies past the cross section, chord by chord - each
/// pair of neighbouring rails runs from the station up the higher rail, along the far rim, and back
/// down the lower rail. Four shapes come out of that: both rails un-notched (no ring); both notched
/// (the trace runs straight from one crossing to the other); both notched AND the ridge leaves
/// between them (the trace dips through the exit, a pentagon); one of each (the far rim runs from
/// the notched rail's crossing out to the Q on the end ring).
///
/// `station` is the cross section rail by rail, `notched` which rails the feature took, `crossing`
/// each notched rail's crossing (or -1), `straddle` the lower rail of the pair the RIDGE leaves
/// between, `exit` where it leaves, and `q_on` the Q on each chord of the end ring by lower rail (or
/// -1). `Err` is the rail the reading gave up at (C#'s `refusedAt`).
pub fn try_the_rings(
    station: &[i32],
    notched: &[bool],
    crossing: &[i32],
    straddle: i32,
    exit: i32,
    q_on: &[i32],
) -> Result<Vec<Vec<i32>>, i32> {
    let mut rings = Vec::new();

    let mut lower = 0;
    while lower + 1 < station.len() {
        let higher = lower + 1;

        if !notched[lower] && !notched[higher] {
            // The stitch's own last row already reached the end plane on both of these rails.
            lower += 1;
            continue;
        }

        let mut ring = vec![station[lower], station[higher]];

        if notched[higher] {
            ring.push(crossing[higher]);
        }

        if notched[lower] && notched[higher] {
            if lower as i32 == straddle {
                ring.push(exit);
            }
        } else if q_on[lower] >= 0 {
            ring.push(q_on[lower]);
        } else {
            // A notched rail standing beside an un-notched one with no Q between them: the trace
            // leaves the strip on this chord and nothing says where. Refused rather than closed on a
            // chord that is not the rim.
            return Err(lower as i32);
        }

        if notched[lower] {
            ring.push(crossing[lower]);
        }

        if ring.len() < 3 {
            return Err(lower as i32);
        }

        rings.push(ring);
        lower += 1;
    }

    Ok(rings)
}

/// C# `TrySew`: the tail past the cross section `station` (rail by rail) closed around the
/// feature's notch - each notched rail's crossing found in the pool (the crease rail's by identity,
/// the notch's own seam vertex), each standing Q on its chord of the end ring - as the rings
/// [`try_the_rings`] reads, fanned the way the rest of the strip faces. Every corner is a vertex
/// already in the pool; one that cannot be found is a refusal by name.
pub fn try_sew(
    pool: &mut SurfaceVertexPool,
    faces: &mut Vec<(i32, i32, i32)>,
    strip: &SurfaceStrip,
    notch: &SurfaceStripNotchedByAFeature,
    station: &[i32],
) -> Result<(), BevelAdvisory> {
    let does_not_extend = |at: Vector3| {
        BevelAdvisory::new(
            BevelAdvisoryCode::SurfaceStandingFeatureDoesNotExtend,
            vec![at.into()],
        )
    };
    let rails = strip.rail_count();
    let mut notched = vec![false; rails.max(0) as usize];
    let mut crossing = vec![-1; rails.max(0) as usize];
    let mut q_on = vec![-1; (rails - 1).max(0) as usize];

    for rail in 0..rails {
        let r = rail as usize;
        notched[r] = notch.notches(rail);
        if !notched[r] {
            continue;
        }

        if rail == notch.the_crease_rail {
            // The crease rail's crossing is on hand BY IDENTITY - the walkback's own seam vertex.
            crossing[r] = notch.exit.vertex;
            continue;
        }

        let param = notch
            .param_on(rail)
            .expect("a notched rail has a parameter");
        let at = strip.point_on(rail, param);
        match pool.try_find_near(
            at,
            SurfaceVertexPool::near_enough_for(pool.vertices()),
            None,
        ) {
            Some(found) => crossing[r] = found,
            // The extension was drawn to this very crossing a moment ago; not finding it means the
            // two readings have come apart.
            None => return Err(does_not_extend(at)),
        }
    }

    for q in [&notch.at_the_low_rail, &notch.at_the_high_rail] {
        if !q.stands() {
            // A ONE-FACED notch's missing Q: no chord of the end ring for it to stand on.
            continue;
        }

        if q.below < 0 || q.below as usize >= q_on.len() {
            return Err(does_not_extend(pool.position_of(q.vertex)));
        }

        q_on[q.below as usize] = q.vertex;
    }

    let rings = match try_the_rings(
        station,
        &notched,
        &crossing,
        notch.exit.below,
        notch.exit.vertex,
        &q_on,
    ) {
        Ok(rings) => rings,
        Err(refused_at) => {
            let rail = refused_at.min(rails - 1).max(0);
            return Err(does_not_extend(strip.point_on(rail, notch.end_param)));
        }
    };

    for ring in rings {
        // Faced the way the rest of the strip is faced: each band is a piece of the strip's surface.
        surface_fan::emit_facing(
            &surface_ring::without_repeats(&ring),
            pool,
            faces,
            strip.outward_reference,
        );
    }

    Ok(())
}
