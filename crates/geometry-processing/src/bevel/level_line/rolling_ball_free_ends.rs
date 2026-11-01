//! An open chain's free end on the rolling ball's route: each side's setbacks taper to nothing at
//! the chain's end vertex, so the round closes onto that vertex (`rolling_ball_setbacks` calls
//! [`close_onto_the_end_vertex`] at every free end of a pass the ball takes, whether or not the
//! two sides agree there).
//!
//! **Why close onto the end vertex.** At a free end every face round the end vertex is one side's
//! or the other's: the picked edge carries on past it as an unpicked crease. The ball's round ends
//! where the pick does, so it closes at the end vertex, on the crease, where the crease stops being
//! filleted; both sides' rails end on that one vertex and the round between them has one point to
//! close onto. The rule does not depend on the sides agreeing, so a small change in the surface
//! never moves the round's end by a whole radius. With the switch off nothing here runs, and the
//! level line keeps its round end one setback past the end.
//!
//! **The taper.** Each side's setback at chain point i becomes the smaller of its own and a cap
//! `W_i`: point i's distance from the end vertex E, lowered only where the field's reading would
//! otherwise let a ball reach E (below). On a straight run the cap is exactly the distance, so the
//! band near the end is the union of balls through E: where the ball's setback S stands, the round
//! ends as the sphere of radius S touching E, and a rail at distance y from E along the chain
//! stands sqrt(2 S y - y^2) from it, meeting the full-width rail tangentially at y = S. No step,
//! no notch, no shoulder. A chain shorter than about 2 S free at both ends gets a lens: both ends'
//! caps bind everywhere and the round thins to nothing at each end.
//!
//! **No ball reaches E.** The band's field is the union of balls (`ChainField`): the least, over
//! every chain point c, of the distance to c less the setback at c, the setback read linearly along
//! each segment, plus the mitre rays at sharp vertices, each carrying its vertex's setback. E reads
//! zero (to the last bits of the field's own arithmetic, never below) when no term is negative
//! there:
//! - **Segments.** Any line lying under the distance from E all along a segment, read at its two
//!   ends, is a pair of endpoint caps whose linear reading stays under the distance (the distance
//!   is convex along the segment, so its own chord does not). Three such lines qualify: the
//!   projection (q - E) . u on the segment's direction u, where it has one sign at both ends
//!   (exact on a run pointing at E); the tangent line of the distance at the endpoint nearer E,
//!   where it stays at or above zero at the other; and the segment's least distance from E, a
//!   constant. Each segment takes the one largest at its endpoint nearer E, whole (mixing the
//!   larger end of one with the larger end of another can rise above the distance between them),
//!   so a segment running square to the line from its near end to E (the projection reading zero
//!   there) never pinches the round to nothing at a turn short of a free end. Each vertex takes
//!   the least of its own distance and both adjacent segments' caps for it.
//! - **Mitre rays.** A sharp vertex (turning past the pass's mitre threshold) carries its setback
//!   along each adjacent segment's line run on past it, read on its side of the bisector
//!   (`chain_field_rays::find_mitres`); where E lies on that side, the vertex is capped by E's
//!   distance from the ray as well. Smooth joints are not told apart here: every vertex past the
//!   threshold is capped, which only lowers it.
//! - **The other end's end ray.** Where the chain's other end O carries an end ray (an oblique end
//!   face there), the chain's line runs on past O carrying O's setback; O is capped by E's
//!   distance from that half-line too (its reach ignored, which only lowers the cap).
//! - **Rounding.** The caps are lowered by [`CAP_MARGIN`] of themselves, so every term other than
//!   E's own reads strictly positive at E whatever the last bits of the field's arithmetic, and
//!   each stored setback per unit of size is stepped down until it times the size is at most the
//!   cap.
//!
//! The cap is then lowered to the largest function under it that changes by no more than each
//! edge's length from vertex to vertex (a sweep each way), so the tapered setbacks stay 1-Lipschitz
//! along the chain wherever the ball's own are. The field stays a union of balls, exactly
//! 1-Lipschitz in space, so every bounded search over it stays sound.
//!
//! **What the rest of the pass sees.** Both sides' fields read zero at E and positive
//! everywhere past it, so each side's rail ends on E, which stands on the chain's last edge between
//! the two sides' band triangles: the two open runs meet end to end on the ridge between their
//! sides (`open_gap_fill::ends_meet`) and the fill closes the round onto that one point. The field's
//! least setback (`ChainField::min_setback`) is then zero: `gap_fill::monotone_slack` falls back to
//! its share of the chain's length (a rail's parameter may step back only by that), and the chord
//! test that tells a rail cutting across a bend (`chain_field_queries`, a chord short of the arc
//! by a tenth of the least setback) counts any shortfall at all.

use vector_math::{ieee_math, Vector3};

use super::ChainField;

/// How much of itself every cap is lowered by, so no term but E's own can read zero or below at E
/// in the field's last bits. A rail moves by this share of its distance at most: a nanometre on a
/// metre.
pub const CAP_MARGIN: f64 = 1e-9;

/// The cap on an open chain's setbacks that closes the round onto its end vertex, one value per
/// chain point (`points` in chain order), as an absolute distance: zero at the end vertex (the
/// last point where `last_end`, else the first), and elsewhere as the file header says, with the
/// chain's sharp vertices those turning by more than `sharp_turn_degrees`.
pub fn end_vertex_cap(
    points: &[Vector3],
    last_end: bool,
    sharp_turn_degrees: f64,
    other_end_ray: bool,
) -> Vec<f64> {
    let count = points.len();
    let mut cap = vec![0.0; count];
    if count < 2 {
        return cap;
    }
    let e = if last_end { count - 1 } else { 0 };
    let end = points[e];
    for (i, limit) in cap.iter_mut().enumerate() {
        *limit = (points[i] - end).length();
    }
    // Each segment's pair of endpoint caps whose linear reading stays under the distance from E.
    for j in 0..count - 1 {
        if let Some((ca, cb)) = segment_caps(points[j], points[j + 1], end) {
            cap[j] = ieee_math::min_f64(cap[j], ca);
            cap[j + 1] = ieee_math::min_f64(cap[j + 1], cb);
        }
    }
    // The other end's end ray, the chain's line run on past it carrying its setback.
    if other_end_ray {
        let (o, inner) = if last_end {
            (0, 1)
        } else {
            (count - 1, count - 2)
        };
        let along = points[o] - points[inner];
        if along.length_squared() > 0.0 {
            let direction = along.get_normal();
            let from = end - points[o];
            let t = ieee_math::max_f64(from.dot(direction), 0.0);
            cap[o] = ieee_math::min_f64(cap[o], (from - (direction * t)).length());
        }
    }
    // Each sharp vertex's mitre rays, where E stands on a ray's kept side.
    if sharp_turn_degrees.is_finite() {
        let cos_threshold = (sharp_turn_degrees * std::f64::consts::PI / 180.0).cos();
        for j in 1..count - 1 {
            let (d_in, d_out) = (points[j] - points[j - 1], points[j + 1] - points[j]);
            if d_in.length_squared() <= 0.0 || d_out.length_squared() <= 0.0 {
                continue;
            }
            let (d_in, d_out) = (d_in.get_normal(), d_out.get_normal());
            let cos = d_in.dot(d_out);
            if cos >= cos_threshold || 1.0 + cos < 1e-9 {
                continue;
            }
            let bisector = d_in + d_out;
            for (direction, keep) in [(d_in, bisector * -1.0), (d_out * -1.0, bisector)] {
                let from = end - points[j];
                if from.dot(keep) < 0.0 {
                    continue;
                }
                let t = ieee_math::max_f64(from.dot(direction), 0.0);
                let off = (from - (direction * t)).length();
                cap[j] = ieee_math::min_f64(cap[j], off);
            }
        }
    }
    for limit in cap.iter_mut() {
        *limit = *limit - (*limit * CAP_MARGIN);
    }
    cap[e] = 0.0;
    // The largest cap under that which changes by no more than each edge's length: a sweep each way.
    for i in 1..count {
        let edge = (points[i] - points[i - 1]).length();
        cap[i] = ieee_math::min_f64(cap[i], cap[i - 1] + edge);
    }
    for i in (0..count - 1).rev() {
        let edge = (points[i + 1] - points[i]).length();
        cap[i] = ieee_math::min_f64(cap[i], cap[i + 1] + edge);
    }
    cap
}

/// One segment `a -> b`'s pair of endpoint caps whose linear reading along it stays at or under
/// the distance from `end`, or `None` for a segment of no length. Three pairs qualify, each a
/// line under the distance (which is convex along the segment): the projection on the segment's
/// direction, where it has one sign at both ends (exact on a run pointing at `end`); the tangent
/// line of the distance at the endpoint nearer `end`, where it stays at or above zero at the
/// other; and the segment's least distance, a constant. The pair taken is the one largest at the
/// nearer endpoint (then at the other), whole: taking the larger value at each end from different
/// pairs can rise above the distance between them.
fn segment_caps(a: Vector3, b: Vector3, end: Vector3) -> Option<(f64, f64)> {
    let along = b - a;
    let length = along.length();
    if length <= 0.0 {
        return None;
    }
    let u = along * (1.0 / length);
    let (pa, pb) = ((a - end).dot(u), (b - end).dot(u));
    let least = (ChainField::closest_on_segment(end, a, b) - end).length();
    let (da, db) = ((a - end).length(), (b - end).length());
    // Pairs as (at the nearer endpoint, at the other).
    let a_nearer = da <= db;
    let mut pairs: Vec<(f64, f64)> = vec![(least, least)];
    if (pa >= 0.0 && pb >= 0.0) || (pa <= 0.0 && pb <= 0.0) {
        pairs.push(if a_nearer {
            (pa.abs(), pb.abs())
        } else {
            (pb.abs(), pa.abs())
        });
    }
    let (near, toward_far, d_near) = if a_nearer {
        (a, u, da)
    } else {
        (b, u * -1.0, db)
    };
    if d_near > 0.0 {
        let slope = (near - end).dot(toward_far) / d_near;
        let far = d_near + (slope * length);
        if far >= 0.0 {
            pairs.push((d_near, far));
        }
    }
    let mut best = pairs[0];
    for &pair in &pairs[1..] {
        if pair.0 > best.0 || (pair.0 == best.0 && pair.1 > best.1) {
            best = pair;
        }
    }
    Some(if a_nearer { best } else { (best.1, best.0) })
}

/// Tapers `setbacks` (per unit of `size`) under [`end_vertex_cap`] at one end of an open chain
/// whose field reads `points`, so the side's band closes onto the end vertex there; every setback
/// the cap does not bind is left exactly as it was. A tapered setback is stepped down until it
/// times `size` (as the band's field reads it) is at most the cap.
pub fn close_onto_the_end_vertex(
    setbacks: &mut [f64],
    points: &[Vector3],
    last_end: bool,
    size: f64,
    sharp_turn_degrees: f64,
    other_end_ray: bool,
) {
    let cap = end_vertex_cap(points, last_end, sharp_turn_degrees, other_end_ray);
    for (setback, &limit) in setbacks.iter_mut().zip(&cap) {
        if *setback * size <= limit {
            continue;
        }
        let mut unit = limit / size;
        while unit > 0.0 && unit * size > limit {
            unit = unit.next_down();
        }
        *setback = ieee_math::max_f64(unit, 0.0);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The field of `points` with `setbacks`, mitred past `sharp_turn_degrees`, read at `at`.
    fn reading(points: &[Vector3], setbacks: &[f64], sharp_turn_degrees: f64, at: Vector3) -> f64 {
        reading_with_rays(points, setbacks, sharp_turn_degrees, (false, false), at)
    }

    /// [`reading`] with end rays past the ends `end_rays` names.
    fn reading_with_rays(
        points: &[Vector3],
        setbacks: &[f64],
        sharp_turn_degrees: f64,
        end_rays: (bool, bool),
        at: Vector3,
    ) -> f64 {
        let field = ChainField::from_polyline_bounded(
            points,
            false,
            setbacks,
            sharp_turn_degrees,
            end_rays,
            (None, None),
        )
        .expect("a chain");
        field.evaluate(at).expect("no budget").value
    }

    /// Whether a reading at the end vertex is zero to the field's last bits and never inside.
    fn at_the_end(value: f64) -> bool {
        (0.0..=1e-12).contains(&value)
    }

    /// On a straight chain the cap is each point's distance from the end vertex, less the margin:
    /// the round near the end is the union of balls through it.
    #[test]
    fn on_a_straight_chain_the_cap_is_the_distance_from_the_end() {
        let points: Vec<Vector3> = (0..5)
            .map(|i| Vector3::new(f64::from(i) * 0.5, 0.0, 0.0))
            .collect();
        let near = |cap: Vec<f64>, expected: [f64; 5]| {
            for (c, x) in cap.iter().zip(expected) {
                assert!((c - x).abs() <= x * 2e-9, "{cap:?}");
            }
        };
        near(
            end_vertex_cap(&points, true, f64::INFINITY, false),
            [2.0, 1.5, 1.0, 0.5, 0.0],
        );
        near(
            end_vertex_cap(&points, false, f64::INFINITY, false),
            [0.0, 0.5, 1.0, 1.5, 2.0],
        );
    }

    /// On a chain that bends round, the field of the capped setbacks reads zero at the end vertex
    /// (never below), and the cap stays 1-Lipschitz along the chain.
    #[test]
    fn on_a_bending_chain_no_ball_reaches_the_end_vertex() {
        let points: Vec<Vector3> = (0..12)
            .map(|i| {
                let angle = f64::from(i) * 0.4;
                Vector3::new(angle.cos(), angle.sin(), 0.0)
            })
            .collect();
        let cap = end_vertex_cap(&points, true, f64::INFINITY, false);
        let end = points[points.len() - 1];
        let setbacks: Vec<f64> = cap.iter().map(|c| c.min(5.0)).collect();
        assert!(at_the_end(reading(&points, &setbacks, f64::INFINITY, end)));
        for s in 0..points.len() - 1 {
            let edge = (points[s + 1] - points[s]).length();
            assert!((cap[s + 1] - cap[s]).abs() <= edge + 1e-12, "edge {s}");
        }
        assert_eq!(cap[cap.len() - 1], 0.0);
    }

    /// A mitre ray carries its vertex's setback past the vertex along a segment's line. In the
    /// review's chain (-1, 0), (0, 0), (-0.174, 0.985) to the end (0.3, -0.5), the ray at (0, 0)
    /// running on along +x passes 0.5 from the end on its kept side, nearer than the vertex's own
    /// 0.583: the field still reads zero at the end, never below. Turned sharper, to (-0.94, 0.34), with
    /// the end far out along the ray at (2, -0.5), every segment's cap leaves the vertex about 2
    /// while the ray passes 0.5 from the end: capped by the segments alone the field reaches past
    /// the end vertex, capped by the ray too it reads zero there, never below.
    #[test]
    fn a_mitre_ray_never_reaches_the_end_vertex() {
        let sharp = 30.0;
        let under = |cap: Vec<f64>| -> Vec<f64> { cap.iter().map(|c| c.min(5.0)).collect() };
        let chain = |o1: (f64, f64), end: (f64, f64)| {
            [
                Vector3::new(-1.0, 0.0, 0.0),
                Vector3::new(0.0, 0.0, 0.0),
                Vector3::new(o1.0, o1.1, 0.0),
                Vector3::new(end.0, end.1, 0.0),
            ]
        };
        let review = chain((-0.174, 0.985), (0.3, -0.5));
        let capped = under(end_vertex_cap(&review, true, sharp, false));
        assert!(at_the_end(reading(&review, &capped, sharp, review[3])));

        let sharper = chain((-0.94, 0.34), (2.0, -0.5));
        let segments_only = under(end_vertex_cap(&sharper, true, f64::INFINITY, false));
        assert!(segments_only[1] > 1.9, "{segments_only:?}");
        assert!(reading(&sharper, &segments_only, sharp, sharper[3]) < 0.0);
        let with_rays = under(end_vertex_cap(&sharper, true, sharp, false));
        assert!(at_the_end(reading(&sharper, &with_rays, sharp, sharper[3])));
    }

    /// At a size that is no power of two, every tapered setback per unit of size, read back at the
    /// size as the band's field reads it, stands at or under its cap.
    #[test]
    fn a_tapered_setback_read_at_the_size_never_passes_its_cap() {
        let points: Vec<Vector3> = (0..40)
            .map(|i| {
                let angle = f64::from(i) * 0.037;
                Vector3::new(7.3 * angle.cos(), 7.3 * angle.sin(), 0.1 * f64::from(i))
            })
            .collect();
        for size in [2.3, 0.7, 13.1] {
            let cap = end_vertex_cap(&points, true, f64::INFINITY, false);
            let mut units = vec![1.0; points.len()];
            close_onto_the_end_vertex(&mut units, &points, true, size, f64::INFINITY, false);
            for (u, c) in units.iter().zip(&cap) {
                assert!(
                    u * size <= *c || (*u == 1.0 && size <= *c),
                    "{u} at {size}: {c}"
                );
            }
        }
    }

    /// A straight run (-3, 0) to (0, 0) that turns by theta and ends free half an edge on: the
    /// corner stands 0.5 from the end vertex, and its cap is that, never zero (the segment into
    /// the corner runs square to the line to the end at 90 degrees, so its projection reads zero
    /// at the corner; the least distance and the tangent line read 0.5). No capped setback reaches
    /// the end vertex at any point of any segment.
    #[test]
    fn a_turn_just_short_of_a_free_end_keeps_its_corner_ball() {
        for degrees in [90.0_f64, 75.0, 60.0] {
            let turn = degrees.to_radians();
            let points = [
                Vector3::new(-3.0, 0.0, 0.0),
                Vector3::new(-2.0, 0.0, 0.0),
                Vector3::new(-1.0, 0.0, 0.0),
                Vector3::new(0.0, 0.0, 0.0),
                Vector3::new(0.5 * turn.cos(), 0.5 * turn.sin(), 0.0),
            ];
            let cap = end_vertex_cap(&points, true, f64::INFINITY, false);
            assert!((cap[3] - 0.5).abs() < 1e-8, "{degrees}: {cap:?}");
            let end = points[4];
            for s in 0..points.len() - 1 {
                for step in 0..=64 {
                    let t = f64::from(step) / 64.0;
                    let at = points[s] + ((points[s + 1] - points[s]) * t);
                    let setback = cap[s] + ((cap[s + 1] - cap[s]) * t);
                    assert!(
                        setback <= (at - end).length() + 1e-12,
                        "{degrees}: segment {s} at {t}: {setback} reaches the end"
                    );
                }
            }
            let setbacks: Vec<f64> = cap.iter().map(|c| c.min(5.0)).collect();
            assert!(at_the_end(reading(&points, &setbacks, f64::INFINITY, end)));
        }
    }

    /// The chain's other end O = (0, 0) carries an end ray along -x (the chain leaves it along
    /// +x), which passes 0.3 from the end vertex (-1, 0.3) though O stands 1.04 from it: capped by
    /// the segments alone the ray reaches past the end vertex; capped by the ray too it does not.
    #[test]
    fn the_other_ends_end_ray_never_reaches_the_end_vertex() {
        let points = [
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(1.0, 0.3, 0.0),
            Vector3::new(-1.0, 0.3, 0.0),
        ];
        let end = points[3];
        let under = |cap: Vec<f64>| -> Vec<f64> { cap.iter().map(|c| c.min(5.0)).collect() };
        let rays = (true, false);
        let without = under(end_vertex_cap(&points, true, f64::INFINITY, false));
        assert!(without[0] > 1.0, "{without:?}");
        assert!(reading_with_rays(&points, &without, f64::INFINITY, rays, end) < 0.0);
        let with = under(end_vertex_cap(&points, true, f64::INFINITY, true));
        assert!((with[0] - 0.3).abs() < 1e-8, "{with:?}");
        assert!(at_the_end(reading_with_rays(
            &points,
            &with,
            f64::INFINITY,
            rays,
            end
        )));
    }
}
