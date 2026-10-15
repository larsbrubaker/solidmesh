//! The SURFACE of one strip, as free functions: its quads, from the ring at one end to the ring at
//! the other, through a station at every interior tangent vertex of either of its two seams.
//!
//! Kept apart from `SurfaceBevelBuilder`, which decides WHAT to build - which chains, which corners,
//! which crossings, and which rings each strip is to run between. This is the one thing that happens
//! after all of that is settled, and it is the same statement for every strip whatever named it: walk
//! the two seams together, and wherever either of them has a vertex the other does not, put a station
//! there so the quads either side of it meet the face the walkback left rather than crossing it.
//!
//! The stations are the seams' own and nothing else's. A point the cut left part way along a chord of
//! a ring is carried in that ring's [`SurfaceStripStation::between`] instead, because it belongs to one
//! rail's window and not to the whole width of the strip. The quads go out through [`surface_fan`];
//! a vertex a later stage left on a seam is caught by [`surface_seam_re_read`].

use std::cmp::Ordering;

use vector_math::ieee_math;

use super::surface_mitre_joint_step::SideSeams;
use super::{surface_fan, surface_ring, surface_seam_re_read};
use super::{SurfaceStrip, SurfaceStripStation, SurfaceVertexPool};
use crate::bevel::{BevelAdvisory, BevelAdvisoryCode};

/// Parameters along an edge nearer than this are one station - the pool's weld, read
/// along the edge rather than across space, so that two stations a weld apart are not asked to be two.
pub fn one_station(pool: &SurfaceVertexPool) -> f64 {
    pool.weld_distance()
}

/// The quads of one strip, from its start ring to its end ring, through a station at
/// every interior tangent vertex of either of its two seams.
///
/// `seams` holds each strip's two seams as `(vertex, parameter)` runs, indexed by `edge_index`. A
/// seam missing there is refused as the seam not meeting: the builder lays both
/// seams of every strip it stitches, so no real input reaches that.
#[allow(clippy::too_many_arguments)]
pub fn try_stitch(
    pool: &mut SurfaceVertexPool,
    faces: &mut Vec<(i32, i32, i32)>,
    later_splits: &[i32],
    strip: &SurfaceStrip,
    seams: &[SideSeams],
    edge_index: i32,
    start_ring: &SurfaceStripStation,
    end_ring: &SurfaceStripStation,
) -> Result<(), BevelAdvisory> {
    let side = |at: usize| {
        seams
            .get(edge_index as usize)
            .and_then(|sides| sides[at].as_deref())
            .filter(|seam| !seam.is_empty())
    };

    let (Some(left), Some(right)) = (side(0), side(1)) else {
        return Err(BevelAdvisory::new(
            BevelAdvisoryCode::SurfaceStripSeamDidNotMeet,
            vec![pool.position_of(start_ring.rail[0]).into()],
        ));
    };

    let rail_count = strip.rail_count() as usize;

    if start_ring.rail[0] != left[0].0
        || end_ring.rail[0] != left[left.len() - 1].0
        || start_ring.rail[rail_count - 1] != right[0].0
        || end_ring.rail[rail_count - 1] != right[right.len() - 1].0
    {
        // The strip has to sew to EVERY vertex the walkback left on its tangent lines, so its first
        // and last stations have to be the first and last of them. Anything else means a vertex of
        // the walked back face would be left standing part way along one of the strip's own edges.
        return Err(BevelAdvisory::new(
            BevelAdvisoryCode::SurfaceStripSeamDidNotMeet,
            vec![pool.position_of(left[0].0).into()],
        ));
    }

    for seam in [left, right] {
        if let Some(missed) =
            surface_seam_re_read::try_find_one_standing_off_the_seam(pool, later_splits, seam)
        {
            // A vertex a later stage left standing part way along this seam, with no station of the
            // strip at it: stitched over, it would be a T junction and a crack in the published
            // surface. The re-read above is what takes these in (SurfaceSeamReRead); one still
            // standing here means it could not, so refuse at the vertex rather than publish.
            return Err(BevelAdvisory::new(
                BevelAdvisoryCode::SurfaceStripSeamDidNotMeet,
                vec![pool.position_of(missed).into()],
            ));
        }
    }

    for rail in 0..rail_count {
        if end_ring.param[rail] < start_ring.param[rail] - one_station(pool) {
            // This rail's two end planes have crossed over inside the strip - the two mitres want
            // more of it than it has - so there is no window to stitch through and no cross section
            // that is not inside out.
            return Err(BevelAdvisory::new(
                BevelAdvisoryCode::SurfaceStripsDoNotMitre,
                vec![strip.point_on(rail as i32, end_ring.param[rail]).into()],
            ));
        }
    }

    for rail in 0..rail_count.saturating_sub(1) {
        if start_ring.between[rail].is_empty() || end_ring.between[rail].is_empty() {
            continue;
        }

        let leaving = start_ring.between[rail][start_ring.between[rail].len() - 1];
        let arriving = end_ring.between[rail][0];

        // A bent end carries its extra point BETWEEN two rails rather than on one, so the window
        // test above cannot see it: an edge short enough, with a skew corner at either end, can have
        // rails with room to spare and the two points its ends bend at still crossed over - a ring
        // that runs back through itself. A chamfered corner's mitre is what bends an end
        // (SurfaceCornerMitre); two of them crossed leave the strip between them no ring to sew.
        if strip.param_of(pool.position_of(arriving))
            < strip.param_of(pool.position_of(leaving)) - one_station(pool)
        {
            return Err(BevelAdvisory::new(
                BevelAdvisoryCode::SurfaceStripsDoNotMitre,
                vec![pool.position_of(arriving).into()],
            ));
        }
    }

    let mut stations = vec![start_ring.clone()];

    for param in interior_stations(left, right, one_station(pool)) {
        let mut ring = vec![0; rail_count];
        let mut at = vec![0.0; rail_count];

        for rail in 0..rail_count {
            // Each rail is stitched only over ITS OWN window, between the parameters it meets the two
            // end planes at. A station outside that window belongs to the other seam, whose rail
            // reaches further along the edge; taken at face value it would send this rail backward
            // and hand SurfaceFan a quad wound the other way, which EmitFacing dutifully re-winds
            // into a surface doubled over on itself. Clamped, the rail simply stands still there and
            // the quad collapses to a triangle, which is what the seam actually looks like.
            at[rail] = ieee_math::min_f64(
                ieee_math::max_f64(param, start_ring.param[rail]),
                end_ring.param[rail],
            );

            ring[rail] = if rail == 0 {
                carrying(left, at[rail], one_station(pool))
            } else if rail == rail_count - 1 {
                carrying(right, at[rail], one_station(pool))
            } else if start_ring.rail[rail] == end_ring.rail[rail] {
                // A rail whose two ends are ONE vertex covers nothing, so it stands on that
                // vertex at every station between them rather than minting a point of its
                // own. Minting one would be a point a little off the vertex - the ends may
                // stand on a corner that is not on this rail's line at all, which is what a
                // mitre eating the rail leaves (SurfaceStaircaseMitre) - and the column
                // either side of it would then draw a sliver where the surface collapsed.
                start_ring.rail[rail]
            } else {
                pool.add(strip.point_on(rail as i32, at[rail]))
            };
        }

        stations.push(SurfaceStripStation::clean(ring, at));
    }

    stations.push(end_ring.clone());

    // What a collapsed column owes the column after it, rail by rail: the points the end cut left on
    // its chord. They are the end FACE's own rim crossing this strip's chord, so the face beside the
    // strip is bounded by them already; dropped, the chord is covered whole on one side and broken
    // on the other, which is a T junction - and emitted where they stand, on a chord of no width,
    // they can only be needles whose normals are whatever the floats say. Handed to the first column
    // that HAS width, they are extra corners of its ring: the trim ring and the strip then pair edge
    // for edge, and every triangle has width to face by.
    let mut carried: Vec<Vec<i32>> = vec![Vec::new(); rail_count.saturating_sub(1)];

    for station in 0..stations.len() - 1 {
        // `rail` indexes both stations' rails and their chords, not only `carried`.
        #[allow(clippy::needless_range_loop)]
        for rail in 0..rail_count.saturating_sub(1) {
            let here = &stations[station];
            let there = &stations[station + 1];

            let stood_still =
                here.rail[rail] == there.rail[rail] && here.rail[rail + 1] == there.rail[rail + 1];

            if stood_still && station + 2 < stations.len() {
                // This column has no width and there is another to carry to. Nothing is emitted for
                // it: what lies on its chord goes forward, and the two rails are the same two
                // vertices at the next station, so the chord it goes forward onto is this one.
                carried[rail].extend_from_slice(&here.between[rail]);

                continue;
            }

            // The quad as a ring rather than as two triangles, because either of its two ends may
            // carry points the end cut left on the profile; both ends are straight, so a fan over the
            // ring covers it exactly and every one of those points is used.
            let mut quad = vec![here.rail[rail]];

            quad.extend_from_slice(on_the_chord(&carried[rail], &here.between[rail]));
            quad.push(here.rail[rail + 1]);
            quad.push(there.rail[rail + 1]);

            for i in (0..there.between[rail].len()).rev() {
                quad.push(there.between[rail][i]);
            }

            quad.push(there.rail[rail]);

            let ring = surface_ring::without_repeats(&quad);

            carried[rail].clear();

            if stood_still {
                // BOTH ends of this chord stood still between the two stations - the later one was
                // clamped onto the earlier at each of them, and the vertex pool handed back the
                // vertex already there rather than minting a second one at it - so whatever is left
                // of the ring lies ON that chord: the chord out one way and back the other, broken
                // wherever one of the two rings carries points along it. This is the LAST column, so
                // there is nothing further along to carry those breaks to and they have to be zipped
                // here: the column has no width anywhere and nothing but needles can be laid in it.
                // Which is still not nothing to emit - their EDGES are what close the shell across
                // it - but it is nothing to COVER, and the fan told to cover it reads the drift of a
                // point off the line - a ten-millionth of a millimetre, measured on the nose-in-notch
                // part whose crease arrives chorded - as a triangle worth keeping and winds it by a
                // normal that is numerical noise. Fanned as a NEEDLE instead, the ring's own edges
                // are used once each and nothing is wound by noise.
                surface_fan::emit_covering_nothing(&ring, pool, faces);

                continue;
            }

            surface_fan::emit_facing(&ring, pool, faces, strip.outward_reference);
        }
    }

    Ok(())
}

/// The points standing on one station's chord between two rails, in the order they
/// stand along it - what earlier collapsed columns handed on, or what this station carries of its own.
///
/// Never both. Only an END ring has anything between its rails - every station the stitch lays is
/// [`SurfaceStripStation::clean`] - so the only `here` with something on it is the start ring's, and
/// nothing has been carried to that one yet. The carry's other meeting with an end ring's own breaks
/// is at the far end, where those arrive on the ring's own side of the quad.
fn on_the_chord<'a>(carried: &'a [i32], here: &'a [i32]) -> &'a [i32] {
    if carried.is_empty() {
        here
    } else {
        carried
    }
}

/// The parameters a cross section is needed at between the two end rings:
/// every tangent vertex of either seam that is not one of its own ends, in order, with two at the
/// same place counted once.
fn interior_stations(left: &[(i32, f64)], right: &[(i32, f64)], one_station: f64) -> Vec<f64> {
    let mut found = Vec::new();

    for seam in [left, right] {
        // Every vertex but the seam's own two ends.
        for on in seam.iter().take(seam.len().saturating_sub(1)).skip(1) {
            found.push(on.1);
        }
    }

    // A total order with NaN before every number, so a NaN parameter cannot break the sort. Equal
    // parameters are indistinguishable, so the unstable sort's tie order cannot reach the output.
    found.sort_unstable_by(|a, b| nan_first_compare(*a, *b));

    let mut stations = Vec::new();

    for i in 0..found.len() {
        if i == 0 || found[i] - found[i - 1] > one_station {
            stations.push(found[i]);
        }
    }

    stations
}

/// Which vertex of a seam the strip is still on at `param` - the last one it has
/// reached. A station the other seam asked for does not put a new point on this one, which is what
/// keeps the walked back face's boundary edges whole.
fn carrying(seam: &[(i32, f64)], param: f64, one_station: f64) -> i32 {
    let mut carrying = seam[0].0;

    for on in seam {
        if on.1 <= param + one_station {
            carrying = on.0;
        }
    }

    carrying
}

/// How two parameters order: NaN before every number, so the order is total.
fn nan_first_compare(a: f64, b: f64) -> Ordering {
    a.partial_cmp(&b)
        .unwrap_or_else(|| a.is_nan().cmp(&b.is_nan()).reverse())
}
