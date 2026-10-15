//! The room a side of the band has in ITSELF, as free functions. Its tolerances, the reading of a
//! side's own room
//! ([`self_room_of_side`]) with its bounded peak search ([`peak_over_side`]) and a varying side's
//! climb ([`climb_from_varying_peak`]), the side's last surviving point
//! ([`last_surviving_point_of_side`]), the chain's ceiling, the corner bound over a triangle
//! ([`greatest_possible_over`]), a side's reach ([`reach_of_side`]) and the per-vertex sample cache
//! ([`sample_at`]).

use vector_math::{ieee_math, Vector3};

use super::{FieldSample, LevelLineMesh, LevelLineTriangle, LevelLineWork, RoomField, WorkError};

/// The FINE pass's tolerance, as a fraction of the peak distance it reads.
pub const PEAK_RELATIVE_TOLERANCE: f64 = 1.0 / 20.0;

/// The COARSE pass's tolerance, as a fraction of the longest a peak's distance could possibly be:
/// a length of the PART, so the coarse pass is scale-free too.
pub const COARSE_RELATIVE_TOLERANCE: f64 = 1.0 / 16.0;

/// Above this many chain points the ceiling is taken from the chain's length, not its pairs.
pub const CEILING_PAIR_LIMIT: usize = 3000;

/// How many times a VARYING side's room is re-read at the size it has just proved.
pub const VARYING_STEPS: i32 = 6;

/// The room a side has in ITSELF - the greatest field distance
/// standing anywhere inside it, read as the size at which the band swallows the side rather than
/// meeting anything foreign. Positive infinity for a side with no triangles, for a side standing
/// at no positive setback, and for a room past `ceiling` (this side is not what limits the pick).
///
/// Where the setback is one number the room is `distance * size / setback` at the peak. Where it
/// VARIES that division is not a reading at all (the additive peak's minimiser answers a
/// different question, and can read HIGH), so the room is the found point's swallow size, which
/// then ITERATES upward ([`climb_from_varying_peak`]).
#[allow(clippy::neg_cmp_op_on_partial_ord)]
pub fn self_room_of_side(
    mesh: &LevelLineMesh,
    field: &RoomField,
    side: i32,
    size: f64,
    samples: &mut [Option<FieldSample>],
    ceiling: f64,
) -> Result<f64, WorkError> {
    let (value, peak) = peak_over_side(mesh, field, side, size, samples, ceiling)?;
    if value == f64::NEG_INFINITY {
        // No triangles on this side at all - nothing to run out of.
        return Ok(f64::INFINITY);
    }

    let self_room;
    if field.setback_varies() {
        (self_room, _) = climb_from_varying_peak(mesh, field, side, size, peak, ceiling)?;
        if self_room.is_nan() {
            return Ok(f64::INFINITY);
        }
    } else {
        let at = field.evaluate(peak)?;
        let setback = field.setback_of(&at);
        if !(setback > 0.0) {
            return Ok(f64::INFINITY);
        }

        self_room = at.distance * size / setback;
    }

    // An interior peak can stand past the ceiling where no vertex did; it is the same statement
    // either way - this side is not what limits the pick.
    Ok(if self_room > ceiling {
        f64::INFINITY
    } else {
        self_room
    })
}

/// A total order in which NaN stands below every number and equal to itself, so a NaN reading
/// never wins a search.
fn compare_to(x: f64, y: f64) -> std::cmp::Ordering {
    use std::cmp::Ordering;
    if x < y {
        Ordering::Less
    } else if x > y {
        Ordering::Greater
    } else if x == y {
        Ordering::Equal
    } else if x.is_nan() {
        if y.is_nan() {
            Ordering::Equal
        } else {
            Ordering::Less
        }
    } else {
        Ordering::Greater
    }
}

/// The greatest the field stands anywhere inside the side, and where -
/// the bounded search [`self_room_of_side`] rests on, and the one a varying side's climb repeats
/// at each size it proves. `(negative infinity, origin)` for a side with no live triangles.
///
/// The side's triangles are searched in descending order of their corner bound
/// ([`greatest_possible_over`]), ties by the lowest triangle index, each seeded with the best
/// value found so far, so a triangle whose bound cannot reach it is spared. A COARSE pass at
/// [`COARSE_RELATIVE_TOLERANCE`] of the side's reach (the greatest corner distance plus the
/// longest edge) finds roughly where and how high the peak stands; a FINE pass at
/// [`PEAK_RELATIVE_TOLERANCE`] of the coarse peak's own distance from the chain, seeded with the
/// coarse answer, reads it. Nothing above `ceiling` needs a number, only a yes: the search is told
/// to stop at the value that proves a point past it.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
pub fn peak_over_side(
    mesh: &LevelLineMesh,
    field: &RoomField,
    side: i32,
    size: f64,
    samples: &mut [Option<FieldSample>],
    ceiling: f64,
) -> Result<(f64, Vector3), WorkError> {
    // The ceiling as a DISTANCE and as a VALUE. A point stands off for a size of
    // dist * size / setback, so a point farther than ceiling * maxSetback / size is above the
    // ceiling whatever setback it reads - and since a value is dist less a setback of at least
    // minSetback, a VALUE past that distance less minSetback proves the same thing.
    let ceiling_distance = if ceiling.is_finite() {
        ceiling * field.max_setback() / size
    } else {
        f64::INFINITY
    };
    let stop_above = if ceiling_distance.is_finite() {
        ceiling_distance - field.min_setback()
    } else {
        f64::INFINITY
    };

    // Only ONE triangle of the side holds the peak, and a triangle can be spared the bounded
    // search whenever its own cheap upper bound already stands below the best found.
    let mut order: Vec<(f64, i32)> = Vec::new();
    let mut vertex_peak = f64::NEG_INFINITY;
    let mut vertex_peak_at = Vector3::ZERO;

    // The farthest any point of the side can stand from the chain: the greatest a CORNER reads
    // plus the side's longest edge, the distance field being 1-Lipschitz.
    let mut farthest_corner = 0.0;
    let mut longest_edge = 0.0;
    for triangle in 0..mesh.triangle_count() {
        let t = mesh.triangles()[triangle as usize];
        if t.tag != side || mesh.is_removed(triangle) {
            continue;
        }

        // Every corner is read once anyway, and the greatest of those readings is a value the side
        // ATTAINS - a floor the searches below prune against from their first step. A VERTEX past
        // the ceiling is deliberately not short-circuited: a room must be the same number read off
        // any mesh of the same surface.
        let indices = [t.a, t.b, t.c];
        for i in 0..3 {
            let at = mesh.position(indices[i]);
            let here = sample_at(field, samples, indices[i], at)?;
            if here.value > vertex_peak {
                vertex_peak = here.value;
                vertex_peak_at = at;
            }

            farthest_corner = ieee_math::max_f64(farthest_corner, here.distance);
            longest_edge = ieee_math::max_f64(
                longest_edge,
                (mesh.position(indices[(i + 1) % 3]) - at).length(),
            );
        }

        order.push((greatest_possible_over(mesh, field, t, samples)?, triangle));
    }

    // Bound descending, and ties by the LOWEST triangle index so the walk below is the same walk
    // every time.
    order.sort_by(|x, y| {
        if x.0 == y.0 {
            x.1.cmp(&y.1)
        } else {
            compare_to(y.0, x.0)
        }
    });

    // The peak over each triangle, in bound order, seeded with a value already attained somewhere
    // on the side - a triangle whose bound cannot reach it is spared the search entirely.
    let search =
        |tolerance: f64, seed: f64, seed_at: Vector3| -> Result<(f64, Vector3), WorkError> {
            let mut greatest_value = seed;
            let mut found = seed_at;
            for &(bound, triangle) in &order {
                if bound < greatest_value {
                    // Sorted, so nothing left can reach the best either.
                    break;
                }

                let t = mesh.triangles()[triangle as usize];
                let greatest = field.max_over_triangle(
                    mesh.position(t.a),
                    mesh.position(t.b),
                    mesh.position(t.c),
                    tolerance,
                    stop_above,
                    greatest_value,
                )?;
                if greatest.value > greatest_value {
                    greatest_value = greatest.value;
                    found = greatest.point;
                }
            }

            Ok((greatest_value, found))
        };

    // COARSE first, at a fraction of the longest a peak's distance could be - deliberately NOT the
    // ceiling, which would make the peak an answer about what ELSE the pick met. Its answer is a
    // value the side really attains, so the peak is in [coarse, coarse + tolerance].
    let reach = farthest_corner + longest_edge;
    if !(reach > 0.0) || !reach.is_finite() {
        // A side lying entirely ON the chain: no distance to read a tolerance off, and no peak.
        return Ok((vertex_peak, vertex_peak_at));
    }

    let (coarse, coarse_at) = search(
        reach * COARSE_RELATIVE_TOLERANCE,
        vertex_peak,
        vertex_peak_at,
    )?;
    if coarse == f64::NEG_INFINITY {
        return Ok((coarse, coarse_at));
    }

    // FINE, about the coarse peak's own distance from the chain, seeded with the coarse value
    // itself: it is ATTAINED at coarse_at, so where nothing beats it that point is the answer.
    let coarse_distance = field.evaluate(coarse_at)?.distance;
    if !(coarse_distance > 0.0) {
        // The peak stands ON the chain - no room, and no distance to read a finer tolerance off.
        return Ok((coarse, coarse_at));
    }

    search(coarse_distance * PEAK_RELATIVE_TOLERANCE, coarse, coarse_at)
}

/// The room of a side whose units VARY, from the point a
/// search at one size found - that point's own swallow size, then the same search again at THAT
/// size, and again (at most [`VARYING_STEPS`] times), while any point of the side still stands
/// above the tolerance where it was read. Returns the size proved and the point proving it.
///
/// Every size it returns is one some point of the side demonstrably survives, so the walk can only
/// ever be LOW, never high, however early it stops; each step is strictly larger than the last.
/// Where the handed-in peak has no swallow size (setbacks that DWARF the part), the climb starts
/// from the DISTANCE field's peak - the side's deepest point - read unceilinged.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
pub fn climb_from_varying_peak(
    mesh: &LevelLineMesh,
    field: &RoomField,
    side: i32,
    size: f64,
    peak: Vector3,
    ceiling: f64,
) -> Result<(f64, Vector3), WorkError> {
    let mut proved = field.swallow_scale(peak)? * size;
    let mut proved_at = peak;
    if !(proved > 0.0) {
        // The peak was read off a field whose setbacks DWARF the part, so it stands ON the chain
        // with a swallow size of zero. The DISTANCE field's peak is the side's deepest point
        // instead, which has a positive swallow size at any scale. UNCEILINGED: a ceiling is a
        // SIZE, and turning it into a distance needs the setback scaled away here.
        LevelLineWork::charge(mesh.vertex_count() as i64)?;
        let mut deepest_samples = vec![None; mesh.vertex_count() as usize];
        let (deepest_value, deepest) = peak_over_side(
            mesh,
            &field.with_setbacks_scaled(0.0)?,
            side,
            size,
            &mut deepest_samples,
            f64::INFINITY,
        )?;
        if deepest_value != f64::NEG_INFINITY {
            proved = field.swallow_scale(deepest)? * size;
            proved_at = deepest;
        }
    }

    for _ in 0..VARYING_STEPS {
        // Zero is an ANSWER here, not a failure to find one: a side whose DEEPEST point stands on
        // the chain has no room, and a climb from zero can only ever prove zero again.
        if !(proved > 0.0) || !proved.is_finite() || proved > ceiling {
            break;
        }

        let at = field.with_setbacks_scaled(proved / size)?;

        // The cache the search below fills is a reading of the side's vertices all over again.
        LevelLineWork::charge(mesh.vertex_count() as i64)?;
        let mut samples = vec![None; mesh.vertex_count() as usize];
        let (value, found) = peak_over_side(mesh, &at, side, proved, &mut samples, ceiling)?;
        if value == f64::NEG_INFINITY || value <= PEAK_RELATIVE_TOLERANCE * proved {
            // Nothing of the side stands clear of the band at this size worth another pass.
            break;
        }

        let next = at.swallow_scale(found)? * proved;
        if !(next > proved) {
            break;
        }

        proved = next;
        proved_at = found;
    }

    Ok((proved, proved_at))
}

/// The side's LAST SURVIVING POINT - the search a room is
/// made from, run for its POINT rather than its number, so a build under that room can put the
/// point into the mesh (see the side witness). `None` for a side with no live triangles.
///
/// On the mesh the reading ran on, the room IS this point's swallow size; on a DIFFERENT mesh of
/// the same surface the same tolerance can settle on a point whose swallow size is smaller, so a
/// caller must read the field at the point rather than assume it survives.
pub fn last_surviving_point_of_side(
    mesh: &LevelLineMesh,
    field: &RoomField,
    side: i32,
    size: f64,
    ceiling: f64,
) -> Result<Option<Vector3>, WorkError> {
    LevelLineWork::charge(mesh.vertex_count() as i64)?;
    let mut samples = vec![None; mesh.vertex_count() as usize];
    let (value, peak) = peak_over_side(mesh, field, side, size, &mut samples, ceiling)?;
    if value == f64::NEG_INFINITY {
        return Ok(None);
    }

    if !field.setback_varies() {
        return Ok(Some(peak));
    }

    let (proved, at) = climb_from_varying_peak(mesh, field, side, size, peak, ceiling)?;
    Ok(Some(if proved.is_nan() { peak } else { at }))
}

/// The greatest distance between two of `chain_points`, read in full up to
/// [`CEILING_PAIR_LIMIT`] points and bounded by the chain's own length past that - the largest
/// size the pick could possibly have room for. A pick with no extent at all (every chain vertex on
/// one point) names no ceiling: infinity.
pub fn chain_ceiling(chain_points: &[Vector3]) -> f64 {
    let mut greatest = 0.0;

    if chain_points.len() <= CEILING_PAIR_LIMIT {
        for i in 0..chain_points.len() {
            for j in (i + 1)..chain_points.len() {
                let apart = (chain_points[j] - chain_points[i]).length();
                greatest = ieee_math::max_f64(greatest, apart);
            }
        }
    } else {
        for i in 1..chain_points.len() {
            greatest += (chain_points[i] - chain_points[i - 1]).length();
        }
    }

    // A self-room is a size, and a size is a length in the part's own units - the same units this
    // distance is in.
    if greatest > 0.0 {
        greatest
    } else {
        f64::INFINITY
    }
}

/// An upper bound on the field over a whole triangle, from its
/// three corners alone: the distance field is 1-Lipschitz, so no point of the triangle stands
/// farther from the chain than a corner does plus the corner's farthest reach into the triangle,
/// and the setback taken off anywhere is at least the chain's least. Exactly the bounded maximum's
/// own first-step bound, so a triangle spared by it is a triangle that search would have spared
/// too - and the corners are read once per build (`samples`).
pub fn greatest_possible_over(
    mesh: &LevelLineMesh,
    field: &RoomField,
    t: LevelLineTriangle,
    samples: &mut [Option<FieldSample>],
) -> Result<f64, WorkError> {
    let corners = [mesh.position(t.a), mesh.position(t.b), mesh.position(t.c)];
    let indices = [t.a, t.b, t.c];
    let mut bound = f64::INFINITY;
    for i in 0..3 {
        let sample = sample_at(field, samples, indices[i], corners[i])?;
        let mut farthest = 0.0;
        for corner in corners {
            farthest = ieee_math::max_f64(farthest, (corner - corners[i]).length());
        }

        bound = ieee_math::min_f64(bound, sample.distance + farthest - field.min_setback());
    }

    Ok(bound)
}

/// A ceiling for a side that is TRUE of the side itself rather than of
/// the pick: how far the side can reach from the chain at all, as a size - what an OPEN chain's
/// reading uses, since a fade's band runs far past the pick it was asked for.
///
/// A self room is `distance / unit setback` at the peak, so a distance no point of the side can
/// exceed, over the SMALLEST setback the field asks, is a size no self room of this side can
/// exceed. It must cover the triangles' INTERIORS, which the corners alone do not (the distance to a
/// chain is not convex), so the side's longest edge is added once: distance to a fixed set is
/// 1-Lipschitz. One field reading per vertex of the side, into the SAME cache the search then
/// reads.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
pub fn reach_of_side(
    mesh: &LevelLineMesh,
    field: &RoomField,
    side: i32,
    size: f64,
    samples: &mut [Option<FieldSample>],
) -> Result<f64, WorkError> {
    let mut farthest = 0.0;
    let mut longest_edge = 0.0;
    let mut found = false;
    for triangle in 0..mesh.triangle_count() {
        let t = mesh.triangles()[triangle as usize];
        if t.tag != side || mesh.is_removed(triangle) {
            continue;
        }

        found = true;
        let corners = [mesh.position(t.a), mesh.position(t.b), mesh.position(t.c)];
        let vertices = [t.a, t.b, t.c];
        for i in 0..3 {
            farthest = ieee_math::max_f64(
                farthest,
                sample_at(field, samples, vertices[i], corners[i])?.distance,
            );
            longest_edge =
                ieee_math::max_f64(longest_edge, (corners[(i + 1) % 3] - corners[i]).length());
        }
    }

    if !found || !(field.min_setback() > 0.0) {
        return Ok(f64::INFINITY);
    }

    Ok((farthest + longest_edge) * size / field.min_setback())
}

/// The field at a vertex, read once per build - a vertex stands in every
/// triangle round it. A vertex outside the cache is read afresh.
pub fn sample_at(
    field: &RoomField,
    samples: &mut [Option<FieldSample>],
    vertex: i32,
    position: Vector3,
) -> Result<FieldSample, WorkError> {
    if vertex >= 0 && (vertex as usize) < samples.len() {
        if let Some(sample) = samples[vertex as usize] {
            return Ok(sample);
        }

        let sample = field.evaluate(position)?;
        samples[vertex as usize] = Some(sample);
        return Ok(sample);
    }

    field.evaluate(position)
}

/// [`chain_ceiling`] over the chain edges' vertices, each once, in
/// the order they are first met.
pub fn self_room_ceiling(mesh: &LevelLineMesh, chain_edges: &[(i32, i32)]) -> f64 {
    let mut seen = std::collections::HashSet::new();
    let mut points = Vec::new();

    for &(from, to) in chain_edges {
        for vertex in [from, to] {
            if seen.insert(vertex) {
                points.push(mesh.position(vertex));
            }
        }
    }

    chain_ceiling(&points)
}
