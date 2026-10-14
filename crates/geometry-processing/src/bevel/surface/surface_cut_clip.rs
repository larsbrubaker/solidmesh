//! Clipping a ring of pooled vertices by a half space: the one piece of arithmetic every cut in
//! the construction is drawn with, written once so the tangent cut and the end splits classify a
//! point the same way. A module of free functions.
//!
//! It owns no state on purpose - a cut hands it a plane and its own profile, and what comes back
//! is named by vertex index, so two facets cut at the same place come back with the same index.
//! `SurfaceFacetCut::clip` and `SurfaceFacetCut::clip_to_pieces` are this module read through one
//! cut's planes. It works on the welding [`SurfaceVertexPool`] and tidies what it leaves with
//! [`surface_ring::without_repeats`].

use vector_math::Vector3;

use super::surface_ring;
use super::SurfaceVertexPool;

/// How near the cut a vertex may stand and still count as ON it. It is the pool's own
/// [`SurfaceVertexPool::weld_distance`] on purpose: a point the pool would weld to one standing on
/// the cut must be read as standing on it too, or the clip carves a sliver that thin off it - and,
/// because the pool reads its weld off the part's size, this follows the part's size too.
pub fn on_the_cut(pool: &SurfaceVertexPool) -> f64 {
    pool.weld_distance()
}

/// The same clip against any half space - the plane through `point` with `inward` naming the side
/// that is kept. The tangent cut and the end splits are the same operation on two different
/// planes, and writing it once is what keeps the crossing arithmetic identical between them.
///
/// **One point is classified one way.** A crossing landing on one of the side's two ENDS is that
/// end (`is_the_crossing`), rather than a new vertex a hair beside it - and that end is then kept
/// whichever side of the cut it measures on. Two cuts a fraction of the ruler apart otherwise
/// leave two vertices that every later reading calls one place and no reading calls one vertex:
/// the first cut splits an edge, the second finds its own crossing a hair along it, and the two
/// pieces either side are bounded by different points. That is a crack along the edge, and it is
/// the shape a mixed corner's footprint makes on a tall thin facet, where the trim curve's chords
/// flatten out and cross the facet's steep diagonal a fraction of the ruler apart
/// (`SurfaceClipClassifiesOnePointOneWayTests`).
///
/// **And a crossing is never MINTED where the construction already has the point.** The cut's own
/// profile - the rim of the bite being taken, whose vertices this very clip is drawing the piece
/// out to - is handed in as `snap_to`, and a crossing landing within
/// [`SurfaceVertexPool::snap_near`] of one of those vertices IS that vertex. Read here, at the
/// crossing, rather than over the finished piece afterwards
/// (`surface_end_notch::snapped_to_the_profile`): a snap that runs after the vertex is in the pool
/// leaves the orphan behind for the sewing pass to find, and the sliver it sews on is the defect.
/// The gap this closes is a chord's SAGITTA and not rounding - a point the construction stood on an
/// edge, met again as the crossing of that edge with a chord of the curve it came off - so it is a
/// ruler or two wide, which is why the reach is the snap and not the ruler. It is what takes a 9 mm
/// fillet of "Round Side Does not bevel attached edges.mcx"'s barrel off the radius sweep's queue
/// (`BevelRadiusSweepTests`, `SurfaceClipClassifiesOnePointOneWayTests`).
///
/// `the_cuts_own_ends` are the two ends of the chord the cut is drawn along, where the caller
/// knows them by index (`SurfaceFacetCutOwnEndsTests`).
pub fn clip_by(
    polygon: &[i32],
    pool: &mut SurfaceVertexPool,
    point: Vector3,
    inward: Vector3,
    snap_to: Option<&[i32]>,
    the_cuts_own_ends: Option<&[i32]>,
) -> Vec<i32> {
    let on_the_cut = on_the_cut(pool);
    let mut kept: Vec<i32> = Vec::with_capacity(polygon.len() + 2);

    // Where each side's crossing lands ON one of its own two ends that end IS the crossing
    // (is_the_crossing). Read for the whole ring first, because a vertex the rule names is kept
    // whichever side it measures on, and either of the two sides meeting there may be the one
    // that names it.
    let mut stands_on_the_cut = vec![false; polygon.len()];
    let mut crossing_at = vec![Vector3::ZERO; polygon.len()];
    let mut crosses = vec![false; polygon.len()];

    // THE CUT'S OWN ENDS FIRST, AND BY IDENTITY. Where the chord this cut is drawn along runs
    // between two vertices the ring already carries, the boundary is already drawn and there is
    // nothing on it to find: the corner IS that end, so it stands on the cut whatever a dot
    // product says of it. Read by distance instead, two ends written by two constructions measure
    // a rounding either side of the line - six ten-millionths of a cross product over a chord of
    // five hundredths, on the fixture this was found on - and the arithmetic answers with a
    // crossing part way along, a third point on a line that already had two. The face across
    // keeps the whole chord, and the T junction between them is what a shell does not close over.
    if let Some(ends) = the_cuts_own_ends {
        for (i, &corner) in polygon.iter().enumerate() {
            if ends.contains(&corner) {
                stands_on_the_cut[i] = true;
            }
        }
    }

    for i in 0..polygon.len() {
        let next = (i + 1) % polygon.len();

        if stands_on_the_cut[i] && stands_on_the_cut[next] {
            // Both ends of this side are the cut's own, so the side IS the cut: the boundary is
            // drawn already and there is nothing on it to find. Both ends are kept whichever side
            // they measure on, which is what the identity above is for.
            continue;
        }

        let from = pool.position_of(polygon[i]);
        let to = pool.position_of(polygon[next]);
        let at_current = (from - point).dot(inward);
        let at_next = (to - point).dot(inward);

        if !((at_current > on_the_cut && at_next < -on_the_cut)
            || (at_current < -on_the_cut && at_next > on_the_cut))
        {
            continue;
        }

        let at = from + ((to - from) * (at_current / (at_current - at_next)));

        if is_the_crossing(pool, at, from, at_current) {
            stands_on_the_cut[i] = true;
            continue;
        }

        if is_the_crossing(pool, at, to, at_next) {
            stands_on_the_cut[next] = true;
            continue;
        }

        crosses[i] = true;
        crossing_at[i] = at;

        // And a vertex ALREADY standing there - left on this very side by a cut that came before, a
        // hair along it - is the crossing too. Without this the second cut stands a new vertex a
        // fraction of the ruler from the first one's, the two pieces either side are bounded by
        // different points, and the crack between them is one no later reading can find, because
        // every reading that would notice calls those two points one place.
        //
        // What makes a vertex an answer is asked of every candidate rather than of the nearest one
        // (SurfaceVertexPool::try_find_near): it has to stand ON this side, within the ruler of the
        // cut itself, and NOT already be a corner of the ring - a ring that visits one vertex twice
        // is pinched, and a pinch fans as two lobes covering each other. Asked the other way about,
        // the nearest vertex might be one of those and the answer would be "none" where a usable
        // one stands a hair further off - and it would change as the pool grows, which it does
        // between the two clips PiecesOutsideOf makes of one side.
        //
        // The CUT'S OWN PROFILE is asked first, and asked of nothing but how near it comes. A
        // vertex of it standing within the snap of the crossing is a point the construction put
        // THERE - on the very edge this cut is drawing its piece out to - so it is the crossing
        // whether or not it stands on the side being cut. It often does not: the side is a chord
        // of the trim curve, and the point the construction stood on the edge is off that chord by
        // its sagitta.
        let already = the_profile_vertex_nearest(pool, snap_to, at).or_else(|| {
            let near_enough = pool.near_enough();
            let qualifies = |candidate: i32| {
                let position = pool.position_of(candidate);
                on_the_side(position, from, to, near_enough)
                    && (position - point).dot(inward).abs() <= near_enough
            };
            pool.try_find_near(at, pool.snap_near(), Some(&qualifies))
        });

        if let Some(already) = already {
            match polygon.iter().position(|&corner| corner == already) {
                None => crossing_at[i] = pool.position_of(already),
                Some(corner) => {
                    // The vertex standing there is a CORNER of this very ring. It cannot be added
                    // again - a ring that visits one vertex twice is pinched, and a pinch fans as
                    // two lobes covering each other - and standing a new one beside it is the
                    // crack this rule is about. So the crossing is not a new place at all: it is
                    // that corner, which the ring already carries, and the corner is kept
                    // whichever side it measures on like any other point the rule names.
                    crosses[i] = false;
                    stands_on_the_cut[corner] = true;
                }
            }
        }
    }

    for i in 0..polygon.len() {
        let at_current = (pool.position_of(polygon[i]) - point).dot(inward);

        if at_current >= -on_the_cut || stands_on_the_cut[i] {
            kept.push(polygon[i]);
        }

        if crosses[i] {
            kept.push(pool.add(crossing_at[i]));
        }
    }

    surface_ring::without_repeats(&kept)
}

/// Whether a side's own END is where its crossing lands: within the pool's ruler of it, or as far
/// as the SNAP along the side where the end really stands on the cut.
///
/// The two reaches are the two rulers the pool keeps and they measure different things
/// ([`SurfaceVertexPool::snap_near`]). Across the cut, a point is on the plane or it is not, and
/// that is the ruler - so the wider reach is allowed only to an end standing ON the plane, which
/// is what `across` says. ALONG the side the reach is the snap: where a side runs nearly along the
/// cutting plane the arithmetic puts the crossing a ruler or two from the end it really is, and
/// without the second reading there is a dead band between the ruler and the snap where the
/// crossing is neither the end nor findable by the search below - whose `on_the_side` asks for a
/// point strictly BETWEEN a side's ends and so can never answer with one of them. What stood there
/// instead was a new vertex a hair off the end, which is the crack the rule is about. A side
/// running ACROSS the plane is a different matter: its end a ruler and a half along it stands a
/// ruler and a half OFF the plane, and that is a second point rather than the same one.
fn is_the_crossing(pool: &SurfaceVertexPool, at: Vector3, end: Vector3, across: f64) -> bool {
    let along = (at - end).length();

    along <= pool.near_enough() || (along <= pool.snap_near() && across.abs() <= pool.near_enough())
}

/// The vertex of `profile` NEAREST `at` standing within the pool's
/// [`SurfaceVertexPool::snap_near`] of it, and `None` where none does - the reading that keeps a
/// clip from minting a second vertex where the construction already has one.
///
/// The nearest and not the first: a profile is a run of samples along a curve, and near the ends
/// of a tight one two of them can both stand within the snap. Taking the first would answer
/// differently depending on which way the caller happened to have listed them, and the two clips a
/// bite makes of one side would then be bounded by different vertices - the very crack this is
/// for. A later vertex at exactly the nearest distance so far replaces it, so a tie always lands
/// on the same vertex.
fn the_profile_vertex_nearest(
    pool: &SurfaceVertexPool,
    profile: Option<&[i32]>,
    at: Vector3,
) -> Option<i32> {
    let profile = profile?;
    let mut nearest = pool.snap_near();
    let mut index = -1;

    for &vertex in profile {
        let how_far = (pool.position_of(vertex) - at).length();

        if how_far <= nearest {
            nearest = how_far;
            index = vertex;
        }
    }

    (index >= 0).then_some(index)
}

/// Whether `at` stands ON the segment from `from` to `to` - within `near` of it, and between its
/// two ends. What it is for is the snap above: a vertex near the crossing is only the crossing
/// when it lies on the side being cut, and not when it merely stands near it in space.
// `!(x > 0)` is true for NaN too, which the rewritten comparison would not be.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
fn on_the_side(at: Vector3, from: Vector3, to: Vector3, near: f64) -> bool {
    let run = to - from;

    if !(run.length_squared() > 0.0) {
        return false;
    }

    let along = (at - from).dot(run) / run.length_squared();

    along > 0.0 && along < 1.0 && ((at - from) - (run * along)).length() <= near
}
