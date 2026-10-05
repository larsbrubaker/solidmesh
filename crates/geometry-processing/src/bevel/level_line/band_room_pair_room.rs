//! The room two chains of one pick leave each other. Ports `Bevel/LevelLine/BandRoom.PairRoom.cs`
//! (a partial of the C# static class `BandRoom`, whose main file is `band_room.rs`): so far its
//! readings over plain positions - the uniform-setback test ([`is_uniform`]), a point's distance to
//! a chain ([`distance_to_polyline`]), a vertex's place in a chain ([`index_in`]), two bounding
//! spheres' gap ([`sphere_gap`]) and the pair-prune bound ([`pair_cannot_lower_the_room`]); the
//! pair reading lands with later packages.

use vector_math::{cs_math, Vector3};

use super::chain_setback;

/// C# `BandRoom.PairPruneSlack`: the relative slack the pair-prune bound must clear the least room
/// by, so a pair whose bound TIES the least cannot be pruned or kept by which way the last bit of
/// the division rounded.
pub const PAIR_PRUNE_SLACK: f64 = 1e-9;

/// C#'s `Enumerable.Min` over doubles: NaN where any value is NaN.
fn linq_min(values: &[f64]) -> f64 {
    let mut least = values[0];
    for &v in values {
        if v.is_nan() {
            return v;
        }

        if v < least {
            least = v;
        }
    }

    least
}

/// C#'s `Enumerable.Max` over doubles: NaN only where every value is NaN.
fn linq_max(values: &[f64]) -> f64 {
    let mut most = values[0];
    for &v in &values[1..] {
        if v > most || most.is_nan() {
            most = v;
        }
    }

    most
}

/// C# `BandRoom.IsUniform`: whether a chain asks ONE setback per unit of size - every
/// square-cornered rim, and every chamfer. The same test `ForeignEdgeRoom.RoomFor` makes before
/// reading a room in one pass instead of bisecting for it, so a candidate calls its reading exact
/// exactly when that reading was.
pub fn is_uniform(unit_setback: &[f64]) -> bool {
    if unit_setback.is_empty() {
        return false;
    }

    chain_setback::is_uniform(linq_min(unit_setback), linq_max(unit_setback))
}

/// C# `BandRoom.DistanceToPolyline`: the distance from a point to a chain's polyline.
pub fn distance_to_polyline(
    positions: &[Vector3],
    chain: &[i32],
    closed: bool,
    point: Vector3,
) -> f64 {
    let mut best = f64::INFINITY;
    let edges = if closed {
        chain.len() as i32
    } else {
        chain.len() as i32 - 1
    };
    for i in 0..edges.max(0) as usize {
        let a = positions[chain[i] as usize];
        let b = positions[chain[(i + 1) % chain.len()] as usize];
        let along = b - a;
        let t = if along.length_squared() > 0.0 {
            cs_math::min_f64(
                1.0,
                cs_math::max_f64(0.0, (point - a).dot(along) / along.length_squared()),
            )
        } else {
            0.0
        };
        best = cs_math::min_f64(best, (point - (a + (along * t))).length());
    }

    best
}

/// C# `BandRoom.IndexIn`: where `vertex` first stands in `chain`, or -1.
pub fn index_in(chain: &[i32], vertex: i32) -> i32 {
    chain
        .iter()
        .position(|&v| v == vertex)
        .map_or(-1, |i| i as i32)
}

/// C# `BandRoom.SphereGap`: the gap between two bounding spheres - zero where they overlap - which
/// is a lower bound on the distance between anything the two spheres contain.
pub fn sphere_gap(centre_k: Vector3, radius_k: f64, centre_j: Vector3, radius_j: f64) -> f64 {
    cs_math::max_f64(0.0, (centre_k - centre_j).length() - radius_k - radius_j)
}

/// C# `BandRoom.PairCannotLowerTheRoom`: true when the ordered pair (k, j) provably cannot be the
/// least reading of the pick, so the pair room may skip its `ForeignEdgeRoom.RoomFor` call
/// outright.
///
/// The bound: an edge's value at size s is `min_P [dist(P) - s * unit(P)] - s * extra`, so with
/// every unit setback in [lowest, highest] and positive, the size at which it falls to zero is at
/// least `approach / (highest_k + extra)`. Taking the smallest approach any edge of j can have to k
/// (the gap between the two chains' bounding spheres) and the largest extra any of j's edges
/// carries, `gap / (highest_k + most_extra)` is a lower bound on the WHOLE pair's room. A pair
/// whose lower bound already stands above the least room offered so far cannot lower it.
///
/// A unit setback that is zero or negative breaks the "value decreases in s" step, and a negative
/// extra breaks the division; the pair is then read as before rather than pruned on a derivation
/// that does not hold. The comparison is guarded by [`PAIR_PRUNE_SLACK`].
#[allow(clippy::neg_cmp_op_on_partial_ord)]
pub fn pair_cannot_lower_the_room(
    least_so_far: f64,
    gap: f64,
    unit_setback_k: &[f64],
    most_extra: f64,
) -> bool {
    if !least_so_far.is_finite() || least_so_far <= 0.0 || !(gap > 0.0) || !(most_extra >= 0.0) {
        return false;
    }

    if unit_setback_k.is_empty() || !(linq_min(unit_setback_k) > 0.0) {
        return false;
    }

    let highest_unit = linq_max(unit_setback_k);
    let denominator = highest_unit + most_extra;
    if !(denominator > 0.0) {
        return false;
    }

    gap / denominator > least_so_far * (1.0 + PAIR_PRUNE_SLACK)
}
