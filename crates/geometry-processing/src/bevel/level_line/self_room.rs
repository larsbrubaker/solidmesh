//! The room a side of the band has in ITSELF (`Bevel/LevelLine/SelfRoom.cs`, a static class: free
//! functions here). Its tolerances, the chain's ceiling, the corner bound over a triangle
//! ([`greatest_possible_over`]), a side's reach ([`reach_of_side`]) and the per-vertex sample cache
//! ([`sample_at`]); the reading of a side's own room lands with later packages.

use vector_math::{cs_math, Vector3};

use super::{FieldSample, LevelLineMesh, LevelLineTriangle, RoomField, WorkError};

/// The FINE pass's tolerance, as a fraction of the peak distance it reads.
pub const PEAK_RELATIVE_TOLERANCE: f64 = 1.0 / 20.0;

/// The COARSE pass's tolerance, as a fraction of the longest a peak's distance could possibly be:
/// a length of the PART, so the coarse pass is scale-free too.
pub const COARSE_RELATIVE_TOLERANCE: f64 = 1.0 / 16.0;

/// Above this many chain points the ceiling is taken from the chain's length, not its pairs.
pub const CEILING_PAIR_LIMIT: usize = 3000;

/// How many times a VARYING side's room is re-read at the size it has just proved.
pub const VARYING_STEPS: i32 = 6;

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
                greatest = cs_math::max_f64(greatest, apart);
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

/// C# `SelfRoom.GreatestPossibleOver`: an upper bound on the field over a whole triangle, from its
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
            farthest = cs_math::max_f64(farthest, (corner - corners[i]).length());
        }

        bound = cs_math::min_f64(bound, sample.distance + farthest - field.min_setback());
    }

    Ok(bound)
}

/// C# `SelfRoom.ReachOfSide`: a ceiling for a side that is TRUE of the side itself rather than of
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
            farthest = cs_math::max_f64(
                farthest,
                sample_at(field, samples, vertices[i], corners[i])?.distance,
            );
            longest_edge =
                cs_math::max_f64(longest_edge, (corners[(i + 1) % 3] - corners[i]).length());
        }
    }

    if !found || !(field.min_setback() > 0.0) {
        return Ok(f64::INFINITY);
    }

    Ok((farthest + longest_edge) * size / field.min_setback())
}

/// C# `SelfRoom.SampleAt`: the field at a vertex, read once per build - a vertex stands in every
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

/// C# `SelfRoom.SelfRoomCeiling`: [`chain_ceiling`] over the chain edges' vertices, each once, in
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
