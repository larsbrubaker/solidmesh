//! The ring where a top and a bottom bevel meet with no straight wall between them
//! (`extrude.rs` builds the stack of rings, `band.rs` the band between two neighbouring rings).
//!
//! The meeting ring is the outer ring of both its bands, the one below rising out to it and the
//! one above shrinking in from it. Each band is the 2D fill of the area between its rings, and a
//! fill triangle whose corners are all points of the meeting ring lies flat at the meeting height:
//! a feature, a narrow neck or the bulge of a finely traced edge that a neighbouring inset has
//! already lost is covered by such triangles. Where the band below covers an area flat (facing
//! down) and the band above covers the same area flat (facing up), the solid there is a sheet of
//! no thickness; where a flat triangle below and a flat triangle above only share an edge, the
//! material switches from above the meeting height to below it across that edge, which then has
//! four faces. Neither holds any material, so those triangles are cut out of the meeting ring
//! before either band is built: the ring becomes the outline of what is really there at that
//! height, and both bands are built on the cut ring, which they share, so the solid stays closed
//! by construction. A neck of material narrower than either neighbouring ring's first inset (a
//! bridge between two holes) is crossed by a chord of both bands' fills with sloping faces on
//! either side, so the solid touches itself along that chord; the neck is cut through with a
//! triangle of ring points beside the chord. Each round cuts whole flat triangles of one band, or
//! failing those such neck triangles, so the cut ring is made of the ring's own points and no new
//! ones; cutting can let the fills choose new flat triangles along the new edges, so this repeats
//! until none is left.

use std::collections::HashSet;
use std::rc::Rc;

use clipper2_rust::{area, FillRule, Path64, Paths64, Point64};
use clipper_library::PolygonsExtensions;

use crate::processors::cap_fill::{doubled_area, fill_triangles, Triangle};
use crate::processors::ring_loops::{simple_loops, without_crossings};

/// How many times the meeting ring is cut before it is taken as it is.
const CUT_ROUNDS: usize = 16;

/// The meeting ring `meeting` with the areas cut out where its two bands - the band from `below`
/// up to it and the band from it up to `above` - would both lie flat on it, overlapping or
/// meeting along an edge. A ring with nothing to cut comes back as the same `Rc`, so such a
/// meeting builds exactly as it did without this step.
///
/// A neck no clean triangle cuts through (none of its candidates lies wholly in the material clear
/// of both neighbouring rings) stays as it is: its chord keeps four faces, so the extrusion is not
/// watertight, and the scene tree's watertight badge (with Repair) shows that to the user as it
/// does for every solid that is not closed. The rest of the solid is still built.
pub(super) fn cut_meeting_ring(
    below: &Paths64,
    meeting: &Rc<Paths64>,
    above: &Paths64,
) -> Rc<Paths64> {
    let mut ring = Rc::clone(meeting);
    if let Some(cut_ring) = vanished_loops(below, &ring, above)
        .and_then(|vanished| ring.subtract(&vanished).ok())
        .and_then(|cut_ring| without_crossings(cut_ring).ok())
    {
        ring = Rc::new(simple_loops(cut_ring));
    }
    for _ in 0..CUT_ROUNDS {
        // the band below holds the lower ring first, the band above the meeting ring first, as
        // `band.rs` fills them
        let fill_below = band_fill(below, &ring, true);
        let fill_above = band_fill(&ring, above, false);
        // Each round cuts one kind of triangle: whole flat triangles of one band (they come from
        // one fill, so they never overlap one another) or, failing those, neck triangles (chosen
        // so that none overlaps another). Either way every corner is a point of the meeting ring
        // and every triangle lies in its material: the cut ring is made of the ring's own points
        // and no new ones, so every round takes area away for good and the cutting ends.
        let Some(cut) = fin_triangles(&fill_below, &fill_above, &ring, above)
            .or_else(|| fin_triangles(&fill_above, &fill_below, &ring, below))
            .or_else(|| pinch_triangles(&fill_below, &fill_above, &ring, below, above))
        else {
            break;
        };
        let Ok(cut_ring) = ring.subtract(&cut).and_then(without_crossings) else {
            break;
        };
        ring = Rc::new(simple_loops(cut_ring));
    }
    ring
}

/// The outlines of the meeting ring that neither neighbouring ring overlaps: a feature both
/// bevels' first insets have lost. Both bands would cover it flat, from below and from above,
/// whichever way they are built (a band filled in 2D or one walled loop to loop caps a loop it
/// cannot pair), so it is a sheet of no thickness and is cut away whole. `None` when every
/// outline holds material on at least one side.
fn vanished_loops(below: &Paths64, meeting: &Paths64, above: &Paths64) -> Option<Paths64> {
    let overlaps = |outline: &Path64, ring: &Paths64| {
        vec![outline.clone()]
            .intersect(ring, FillRule::EvenOdd)
            .map_or(true, |common| common.iter().map(area).sum::<f64>() != 0.0)
    };
    let vanished: Paths64 = meeting
        .iter()
        .filter(|outline| area(outline) > 0.0)
        .filter(|outline| !overlaps(outline, below) && !overlaps(outline, above))
        .cloned()
        .collect();
    (!vanished.is_empty()).then_some(vanished)
}

/// The flat triangles of `fill` that make a fin with the other band's fill `other`: those that
/// overlap one of its flat triangles (a sheet of no thickness) or share with it a chord between
/// two meeting ring points (an edge with four faces), as paths. A triangle of the band below lies
/// outside its lower ring, but it may reach into the other band's ring `other_ring`; cutting it
/// there would make the meeting ring cross that ring, so only triangles clear of it are given.
/// `None` when there is none.
fn fin_triangles(
    fill: &BandFill,
    other: &BandFill,
    meeting: &Paths64,
    other_ring: &Paths64,
) -> Option<Paths64> {
    let ring_edges: HashSet<Edge> = meeting
        .iter()
        .flat_map(|path| (0..path.len()).map(move |i| (path[i], path[(i + 1) % path.len()])))
        .map(|(a, b)| edge((a.x, a.y), (b.x, b.y)))
        .collect();
    let shared: HashSet<Edge> = fill
        .chords
        .intersection(&other.chords)
        .filter(|chord| !ring_edges.contains(chord))
        .copied()
        .collect();
    let as_path = |triangle: &Triangle| -> Path64 {
        triangle.iter().map(|&(x, y)| Point64::new(x, y)).collect()
    };
    // in the fill's own order, so the cut (and the mesh) never depends on hash order
    let fins: Paths64 = fill
        .flat
        .iter()
        .filter(|&&triangle| {
            (0..3).any(|k| shared.contains(&edge(triangle[k], triangle[(k + 1) % 3])))
                || other
                    .flat
                    .iter()
                    .any(|&flat| triangles_overlap(triangle, flat))
        })
        .filter(|triangle| {
            vec![as_path(triangle)]
                .intersect(other_ring, FillRule::EvenOdd)
                .is_ok_and(|common| common.iter().map(area).sum::<f64>() == 0.0)
        })
        .map(as_path)
        .collect();
    (!fins.is_empty()).then_some(fins)
}

/// Cuts that part the material where it pinches to an edge: a chord between two meeting ring
/// points that both bands' fills cross inside their area (a neck of material, such as a bridge
/// between two holes, narrower than either neighbouring ring's first inset keeps). Each band lays
/// a face on either side of such a chord, sloping away from the meeting height, so the chord has
/// four faces and the solid above and below it touches along an edge only. The neck holds
/// material at the meeting height alone, so it is cut through: for each such chord the smallest
/// triangle of the chord and a ring neighbour of one of its ends that lies wholly in the meeting
/// ring's material, clear of both neighbouring rings (so the cut ring crosses neither) and adds
/// no new point to the ring. After the cut the chord is an edge of the ring, walked once by each
/// band. The chords are taken in sorted order and a triangle overlapping one already chosen is
/// left for the next round, so the cut never depends on hash order. `None` when there is none.
fn pinch_triangles(
    fill_below: &BandFill,
    fill_above: &BandFill,
    meeting: &Paths64,
    below: &Paths64,
    above: &Paths64,
) -> Option<Paths64> {
    let ring_points: HashSet<(i64, i64)> = meeting.iter().flatten().map(|p| (p.x, p.y)).collect();
    let ring_edges: HashSet<Edge> = meeting
        .iter()
        .flat_map(|path| (0..path.len()).map(move |i| (path[i], path[(i + 1) % path.len()])))
        .map(|(a, b)| edge((a.x, a.y), (b.x, b.y)))
        .collect();
    let mut pinches: Vec<Edge> = fill_below
        .chords
        .intersection(&fill_above.chords)
        .filter(|chord| !ring_edges.contains(chord))
        .copied()
        .collect();
    pinches.sort_unstable();
    // every ring neighbour of a point, in ring order
    let neighbours = |point: (i64, i64)| -> Vec<(i64, i64)> {
        let mut found = Vec::new();
        for path in meeting {
            let count = path.len();
            for i in 0..count {
                if (path[i].x, path[i].y) == point {
                    for p in [path[(i + count - 1) % count], path[(i + 1) % count]] {
                        found.push((p.x, p.y));
                    }
                }
            }
        }
        found
    };
    let as_path = |triangle: &Triangle| -> Path64 {
        triangle.iter().map(|&(x, y)| Point64::new(x, y)).collect()
    };
    let common_area = |triangle: &Triangle, ring: &Paths64| {
        vec![as_path(triangle)]
            .intersect(ring, FillRule::EvenOdd)
            .map(|common| common.iter().map(area).sum::<f64>())
    };
    let cuts_cleanly = |triangle: &Triangle| {
        let in_material = common_area(triangle, meeting)
            .is_ok_and(|inside| inside == doubled_area(*triangle) as f64 / 2.0);
        in_material
            && common_area(triangle, below).is_ok_and(|common| common == 0.0)
            && common_area(triangle, above).is_ok_and(|common| common == 0.0)
            && meeting.subtract(&vec![as_path(triangle)]).is_ok_and(|cut| {
                cut.iter()
                    .flatten()
                    .all(|p| ring_points.contains(&(p.x, p.y)))
            })
    };
    let mut cuts: Vec<Triangle> = Vec::new();
    for (p, q) in pinches {
        let best = neighbours(p)
            .into_iter()
            .chain(neighbours(q))
            .filter(|&r| r != p && r != q)
            .map(|r| {
                let triangle = [p, q, r];
                // counter clockwise, so the overlap and area tests read it the right way round
                if doubled_area(triangle) < 0 {
                    [q, p, r]
                } else {
                    triangle
                }
            })
            .filter(|&triangle| doubled_area(triangle) > 0 && cuts_cleanly(&triangle))
            // the smallest cut; ties go to the earlier candidate (`min_by_key` keeps the first)
            .min_by_key(|&triangle| doubled_area(triangle));
        if let Some(triangle) = best {
            if !cuts.iter().any(|&cut| triangles_overlap(cut, triangle)) {
                cuts.push(triangle);
            }
        }
    }
    (!cuts.is_empty()).then(|| cuts.iter().map(as_path).collect())
}

/// Whether two counter clockwise triangles share area, exactly: they do unless an edge of one
/// has the whole of the other on or outside its line (two convex shapes that share no area are
/// parted by the line of an edge of one of them).
fn triangles_overlap(a: Triangle, b: Triangle) -> bool {
    let parted_by_an_edge_of = |t: Triangle, other: Triangle| {
        (0..3).any(|k| {
            let (p, q) = (t[k], t[(k + 1) % 3]);
            other.iter().all(|&r| doubled_area([p, q, r]) <= 0)
        })
    };
    !parted_by_an_edge_of(a, b) && !parted_by_an_edge_of(b, a)
}

/// An unordered edge between two ring points.
type Edge = ((i64, i64), (i64, i64));

fn edge(a: (i64, i64), b: (i64, i64)) -> Edge {
    if a < b {
        (a, b)
    } else {
        (b, a)
    }
}

/// What a band's fill puts on the meeting ring: its triangles that lie flat there, counter
/// clockwise, and every edge of its triangles that joins two meeting ring points at the meeting
/// height.
struct BandFill {
    flat: Vec<Triangle>,
    chords: HashSet<Edge>,
}

/// The fill of the band between `lower` and `upper` (in that order, as the band fills it) as it
/// meets the meeting ring, which is `upper` when `meeting_is_upper`. A band lifts a point both
/// rings pass through to its lower ring, so below the meeting ring a corner is at the meeting
/// height when it is not on the lower ring, and above it when it is on the meeting ring. A band
/// whose rings cross is not filled in 2D (`band.rs` walls it loop to loop), so it puts nothing
/// there.
fn band_fill(lower: &Paths64, upper: &Paths64, meeting_is_upper: bool) -> BandFill {
    let mut fill = BandFill {
        flat: Vec::new(),
        chords: HashSet::new(),
    };
    let loops: Vec<&Path64> = lower.iter().chain(upper.iter()).collect();
    let Ok(triangles) = fill_triangles(&loops) else {
        return fill;
    };
    let on_lower: HashSet<(i64, i64)> = lower.iter().flatten().map(|p| (p.x, p.y)).collect();
    let at_meeting = |p: &(i64, i64)| on_lower.contains(p) != meeting_is_upper;
    for triangle in triangles {
        for k in 0..3 {
            let (a, b) = (triangle[k], triangle[(k + 1) % 3]);
            if at_meeting(&a) && at_meeting(&b) {
                fill.chords.insert(edge(a, b));
            }
        }
        if doubled_area(triangle) > 0 && triangle.iter().all(at_meeting) {
            fill.flat.push(triangle);
        }
    }
    fill
}
