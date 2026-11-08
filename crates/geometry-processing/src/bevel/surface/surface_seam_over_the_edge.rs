//! Which run of a walked-back tangent line one strip sews to. Kept apart from `SurfaceBevelBuilder`
//! so the builder's wiring stays readable; the builder asks [`try_pick_the_run_over_the_edge`] once
//! per side of every strip.

// The negated comparisons are deliberate: each is false for NaN, which is the point.
#![allow(clippy::neg_cmp_op_on_partial_ord)]

use solidmesh_math::Vector3;

use super::{
    surface_bevel_stitch, SurfaceBandEnd, SurfaceEdgePlan, SurfaceStrip, SurfaceVertexPool,
};

/// How square to the walk a band end plane has to stand before the crossing it names means
/// anything. Both directions are unit, so this is a pure cosine and needs no part to scale it:
/// every honest band end is a mitre normal, whose slope is the cosine of half the turn and so above
/// 0.0087 for every turn the mitre does not refuse outright, while a plane lying along the rail
/// sits at the 1e-8 that [`SurfaceStrip::try_cross`]'s own absolute guard lets through.
const ALONG_ENOUGH_TO_BOUND: f64 = 1e-6;

/// The one run of a tangent line this edge's strip sews to - the one lying OVER the edge, which is
/// the one whose stretch of the line overlaps the edge's own span of it. `Ok` is the seam, each
/// vertex with its parameter along the strip; `Err` is where to report the refusal: the nearest end
/// of the nearest run the band's span does not reach, or the middle of the edge where the face gave
/// up over the whole of it and there is no run to name.
///
/// A tangent line is infinite and a face can hold more than one stretch of it - a notch bitten out
/// of the face's side, or a bore the band runs either side of - so the walkback reports runs
/// (`SurfaceWalkbackResult::tangent_runs`) and this is where one of them is chosen. The edge spans
/// parameter 0 to its own length along the strip, and exactly one run can overlap that, because the
/// runs are disjoint stretches of one line. No run there is a refusal: it means the face gave up
/// everything over the edge, and there is nothing to sew the strip to.
///
/// The window read here is the BAND's own span, not the edge's. A strip is bounded by its two mitre
/// planes, and where two beads of one chain set back by different amounts those planes stand past
/// the edge's corners rather than on them: on the mouth of a raked bore the setback steps by up to
/// 0.077 between chords of 0.3 at nine degree turns, which puts the mitre point about half a chord
/// past its corner, so a band's whole stretch of its tangent line can stand clear of the chord it
/// belongs to. Read against the edge's span that band overlaps nothing and every strip but one is
/// refused; read against the span its own mitre planes cut off the outermost rail - which is
/// exactly where the strip will be trimmed to - it is the run the walkback left, and there is one
/// for each.
///
/// A free end has no plane to be read off, and there the edge's own rim-to-rim span stands in as it
/// always did: one picked edge of a cube's top is sewn to the whole of its tangent line.
///
/// `band_ends` are the planes that bound this side of the band along the chain
/// (`SurfaceBevelPlan::band_ends_of`), or `None` where both ends of the edge are free; `runs` is
/// `None` where the walkback left none. `left` is which side of the walk is being sewn, which says
/// which rail the window is read on.
pub fn try_pick_the_run_over_the_edge(
    pool: &SurfaceVertexPool,
    strip: &SurfaceStrip,
    edge: &SurfaceEdgePlan,
    runs: Option<&[Vec<i32>]>,
    band_ends: Option<&[SurfaceBandEnd]>,
    left: bool,
) -> Result<Vec<(i32, f64)>, Vector3> {
    let mut seam: Option<Vec<(i32, f64)>> = None;
    let mut refused_at = (edge.from() + edge.to()) / 2.0;

    let (starts, ends) = the_bands_span(strip, edge, band_ends, left);

    let mut most_overlap = surface_bevel_stitch::one_station(pool);
    let mut nearest_miss = f64::INFINITY;

    for run in runs.unwrap_or(&[]) {
        if run.len() < 2 {
            continue;
        }

        let found: Vec<(i32, f64)> = run
            .iter()
            .map(|&vertex| (vertex, strip.param_of(pool.position_of(vertex))))
            .collect();

        let overlap = min_or_nan(found[found.len() - 1].1, ends) - max_or_nan(found[0].1, starts);

        if overlap > most_overlap {
            most_overlap = overlap;
            seam = Some(found);
        } else if -overlap < nearest_miss {
            // How far this run stands OUTSIDE the edge's span, and which of its ends is the near
            // one: the place a refusal should be reported at, because the run is what the strip
            // would have sewn to and its near end is where it stopped short.
            nearest_miss = -overlap;
            let near_end = if found[0].1 > ends {
                0
            } else {
                found.len() - 1
            };
            refused_at = pool.position_of(found[near_end].0);
        }
    }

    seam.ok_or(refused_at)
}

/// The stretch of the strip the band really covers, as parameters along the edge (`(starts,
/// ends)`): where each of the band's own end planes crosses the outermost rail on this side, and
/// the edge's own corner at an end with no plane.
///
/// Read on the rail rather than through the edge's corners because that rail IS the tangent line
/// the run lies along, so the two numbers compared are of one line. An end plane the rail runs
/// along has no crossing to read ([`SurfaceStrip::try_cross`]), and the corner stands in for it -
/// that is a chain doubling back on itself, which the mitre refuses in its own words.
///
/// Two readings that are arithmetic rather than geometry are made here on purpose, and
/// `SurfaceSeamWindowTests` holds both. A plane the rail runs ALMOST along is not turned down by
/// [`SurfaceStrip::try_cross`], whose guard is an absolute 1e-9 on the slope, so it answers with a
/// crossing of order 1e8 - and a window opened that wide bounds nothing and refuses every run on
/// the region. What tells the two apart is the SLOPE and not a length: a band end's normal is the
/// mitre's, so its slope against the walk is the cosine of half the turn, which is near one on a
/// gentle corner and still 0.0087 at a hundred and seventy nine degrees - past which the mitre
/// refuses in its own words. A length would be the wrong ruler here, because a mitre honestly
/// carries a rail its own offset over the TANGENT of half the turn, which on a six degree corner of
/// a chorded loop is eighteen times the setback and a good twenty chords past the corner. And two
/// ends may face the same way - two mitre planes at one corner both bounding the start - in which
/// case the band is inside BOTH, so the tighter is the band's; taken last-one-wins the window would
/// depend on the order the plan happened to list its ends in.
pub fn the_bands_span(
    strip: &SurfaceStrip,
    edge: &SurfaceEdgePlan,
    band_ends: Option<&[SurfaceBandEnd]>,
    left: bool,
) -> (f64, f64) {
    let rim_to_rim = strip.param_of(edge.to());

    let rail = if left { 0 } else { strip.rail_count() - 1 };

    let mut the_start: Option<f64> = None;
    let mut the_finish: Option<f64> = None;

    for end in band_ends.unwrap_or(&[]) {
        let slope = end.inward.dot(strip.along());

        if !(slope.abs() > ALONG_ENOUGH_TO_BOUND) {
            continue;
        }

        let Some(at) = strip.try_cross(rail, end.point, end.inward) else {
            continue;
        };

        // Which end it is comes from the plane's own facing: it points INTO the band, so a normal
        // that runs with the walk bounds the band's start and one that runs against it the finish.
        // These max / min answer NaN where either argument is NaN.
        if slope > 0.0 {
            the_start = Some(the_start.map_or(at, |start| max_or_nan(start, at)));
        } else {
            the_finish = Some(the_finish.map_or(at, |finish| min_or_nan(finish, at)));
        }
    }

    // Held as optional and only now taken, so that a single end carrying the band PAST a corner -
    // which is the ordinary reading at an unequal mitre - is kept, where a plain max against the
    // rim-to-rim default would have thrown it away.
    (the_start.unwrap_or(0.0), the_finish.unwrap_or(rim_to_rim))
}

/// The larger of two numbers, NaN where either argument is (Rust's `f64::max` ignores it).
fn max_or_nan(a: f64, b: f64) -> f64 {
    if a.is_nan() || b.is_nan() {
        f64::NAN
    } else {
        a.max(b)
    }
}

/// The smaller of two numbers, NaN where either argument is (Rust's `f64::min` ignores it).
fn min_or_nan(a: f64, b: f64) -> f64 {
    if a.is_nan() || b.is_nan() {
        f64::NAN
    } else {
        a.min(b)
    }
}
