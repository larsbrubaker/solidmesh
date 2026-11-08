//! The room two chains of one pick leave each other (a part of the band room, whose main module is
//! `band_room.rs`): its
//! readings over plain positions - the uniform-setback test ([`is_uniform`]), a point's distance to
//! a chain ([`distance_to_polyline`]), a vertex's place in a chain ([`index_in`]), two bounding
//! spheres' gap ([`sphere_gap`]), the pair-prune bound ([`pair_cannot_lower_the_room`]), the edges
//! running out of shared corners ([`edges_running_out_of_corners`]), the pieces of one chain split
//! at setback steps ([`pieces_of_one_chain`]) and the ends stopping at an end face
//! ([`ends_with_an_end_region`]) and the end-to-end approach test ([`approach_is_end_to_end`]).
//! The pick's room itself ([`room_of_pick`]) is in `band_room_pair_room_pick.rs`.

use std::collections::HashSet;

use indexmap::IndexSet;
use solidmesh_math::{ieee_math, Vector3};

use super::{chain_setback, ChainField, EndRayBound, ForeignEdgeRegions, LevelLineMesh, WorkError};

pub use super::band_room_pair_room_pick::{room_of_pick, PickRoom};

/// The relative slack the pair-prune bound must clear the least room
/// by, so a pair whose bound TIES the least cannot be pruned or kept by which way the last bit of
/// the division rounded.
pub const PAIR_PRUNE_SLACK: f64 = 1e-9;

/// The least of `values`: NaN where any value is NaN, so a broken reading is never hidden.
pub fn min_or_nan(values: &[f64]) -> f64 {
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

/// The most of `values`: NaN only where every value is NaN (a NaN never beats a number).
pub fn max_or_nan(values: &[f64]) -> f64 {
    let mut most = values[0];
    for &v in &values[1..] {
        if v > most || most.is_nan() {
            most = v;
        }
    }

    most
}

/// Whether a chain asks ONE setback per unit of size - every
/// square-cornered rim, and every chamfer. The same test `ForeignEdgeRoom.RoomFor` makes before
/// reading a room in one pass instead of bisecting for it, so a candidate calls its reading exact
/// exactly when that reading was.
pub fn is_uniform(unit_setback: &[f64]) -> bool {
    if unit_setback.is_empty() {
        return false;
    }

    chain_setback::is_uniform(min_or_nan(unit_setback), max_or_nan(unit_setback))
}

/// The distance from a point to a chain's polyline.
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
            ieee_math::min_f64(
                1.0,
                ieee_math::max_f64(0.0, (point - a).dot(along) / along.length_squared()),
            )
        } else {
            0.0
        };
        best = ieee_math::min_f64(best, (point - (a + (along * t))).length());
    }

    best
}

/// Where `vertex` first stands in `chain`, or -1.
pub fn index_in(chain: &[i32], vertex: i32) -> i32 {
    chain
        .iter()
        .position(|&v| v == vertex)
        .map_or(-1, |i| i as i32)
}

/// The gap between two bounding spheres - zero where they overlap - which
/// is a lower bound on the distance between anything the two spheres contain.
pub fn sphere_gap(centre_k: Vector3, radius_k: f64, centre_j: Vector3, radius_j: f64) -> f64 {
    ieee_math::max_f64(0.0, (centre_k - centre_j).length() - radius_k - radius_j)
}

/// True when the ordered pair (k, j) provably cannot be the
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

    if unit_setback_k.is_empty() || !(min_or_nan(unit_setback_k) > 0.0) {
        return false;
    }

    let highest_unit = max_or_nan(unit_setback_k);
    let denominator = highest_unit + most_extra;
    if !(denominator > 0.0) {
        return false;
    }

    gap / denominator > least_so_far * (1.0 + PAIR_PRUNE_SLACK)
}

/// The edges of chain j (indices into its chain edges) that
/// run out of a corner j shares with chain k: the edge on the corner itself, and every edge after
/// it, walking away from the corner, while both its ends stay BEHIND the corner as k sees it - no
/// nearer any edge of k at the corner than the corner vertex is - or while it runs AWAY from k.
/// Along such a run the distance to k IS the distance to the corner, so reading it as a pinch reads
/// j's own length from the corner, not an approach across a face: the rim's SECOND facet, one chord
/// from a corner the face turns reflex at, read a room of 0.56 where 3.756 builds ("Top & Round
/// Should be Union"). The run stops at the first edge that comes round in front of the corner, so a
/// chain that turns back and does approach k across the face is still read.
///
/// The corners are walked in their set's order,
/// which matters: a later corner's walk stops at an edge an earlier one skipped.
pub fn edges_running_out_of_corners(
    positions: &[Vector3],
    chain_j: &[i32],
    closed_j: bool,
    chain_k: &[i32],
    closed_k: bool,
    shared_corners: &IndexSet<i32>,
) -> IndexSet<i32> {
    let mut skipped: IndexSet<i32> = IndexSet::new();
    let count_j = chain_j.len() as i32;
    let edge_count = if closed_j { count_j } else { count_j - 1 };
    let count_k = chain_k.len() as i32;
    let at_j = |i: i32| positions[chain_j[i as usize] as usize];
    for &corner in shared_corners {
        let at = positions[corner as usize];
        let k_index = index_in(chain_k, corner);
        let mut along_k = Vec::new();
        for step in [-1, 1] {
            let mut n = k_index + step;
            if closed_k {
                n = (n + count_k) % count_k;
            }

            if n >= 0 && n < count_k && chain_k[n as usize] != corner {
                along_k.push(positions[chain_k[n as usize] as usize] - at);
            }
        }

        let behind = |point: Vector3| {
            let offset = point - at;
            along_k
                .iter()
                .all(|d| offset.dot(*d) <= 1e-9 * offset.length() * d.length())
        };
        let from_k = |point: Vector3| distance_to_polyline(positions, chain_k, closed_k, point);

        // Edge (vertex, next) moves AWAY from k: its middle and its far end both stand further from
        // k than its near end.
        let runs_away = |vertex: i32, next: i32| {
            let near = from_k(at_j(vertex));
            from_k(at_j(next)) > near && from_k((at_j(vertex) + at_j(next)) / 2.0) > near
        };

        let j_index = index_in(chain_j, corner);
        for step in [-1, 1] {
            // The edge from vertex i to i + 1 is edge i; walking down, the edge from i to i - 1 is
            // edge i - 1.
            let mut vertex = j_index;
            for walked in 0..edge_count.max(0) {
                let mut edge = if step > 0 { vertex } else { vertex - 1 };
                if closed_j {
                    edge = (edge + count_j) % count_j;
                }

                if edge < 0 || edge >= edge_count || (skipped.contains(&edge) && walked > 0) {
                    break;
                }

                let next = if closed_j {
                    (vertex + step + count_j) % count_j
                } else {
                    vertex + step
                };

                // The edge ON the corner is always left unread; after it only while it stays
                // behind, or while it runs AWAY from k. A chain leaving the corner at an angle to k
                // is nearest k at the corner and further off at every step, so its later edges
                // approach k nowhere the corner does not already (Inner Right Sides and Center.mcx:
                // the second edge of a concave chain cut in two set the room at 1.67 where the band
                // cuts cleanly to 2.5). An approach elsewhere - a cube face's far edge running
                // parallel to k - does not move away, and is read.
                if walked > 0
                    && !(behind(at_j(vertex)) && behind(at_j(next)))
                    && !runs_away(vertex, next)
                {
                    break;
                }

                skipped.insert(edge);
                vertex = next;
            }
        }
    }

    skipped
}

/// For each chain, the chains it was ONE chain with before the
/// split at setback steps: every chain reached from it end to end through a vertex of
/// `step_corners`, itself included (in the order they were reached; the pieces of one chain share
/// one list, so each piece carries the same list).
pub fn pieces_of_one_chain(
    chains: &[Vec<i32>],
    closed: &[bool],
    step_corners: &HashSet<i32>,
) -> Vec<Vec<i32>> {
    let mut pieces: Vec<Option<Vec<i32>>> = vec![None; chains.len()];
    for k in 0..chains.len() {
        if step_corners.is_empty() {
            pieces[k] = Some(vec![k as i32]);
        }

        if pieces[k].is_some() {
            continue;
        }

        let mut found = vec![k as i32];
        let mut i = 0;
        while i < found.len() {
            let at = found[i] as usize;
            i += 1;
            if closed[at] {
                continue;
            }

            let ends = [chains[at][0], chains[at][chains[at].len() - 1]];
            for (j, chain) in chains.iter().enumerate() {
                if !found.contains(&(j as i32))
                    && !closed[j]
                    && ends.iter().any(|v| {
                        step_corners.contains(v) && (chain[0] == *v || chain[chain.len() - 1] == *v)
                    })
                {
                    found.push(j as i32);
                }
            }
        }

        for &piece in &found {
            pieces[piece as usize] = Some(found.clone());
        }
    }

    pieces.into_iter().map(Option::unwrap_or_default).collect()
}

/// The ends of chains k and j that stop at an END FACE (an end
/// region, see [`ForeignEdgeRegions::runs_out`]) - `None` unless both chains have one, the only
/// case in which the end-to-end approach test can leave a pair's approach unread. An end whose ray
/// is bounded on a same-sense end face stops ON that face (the end-plane trim), as an end region's
/// does, though the face is left whole for the trim and so is no end region.
pub fn ends_with_an_end_region(
    regions: &ForeignEdgeRegions,
    chain_edges: &[Vec<(i32, i32)>],
    k: i32,
    j: i32,
    bounds_k: (Option<EndRayBound>, Option<EndRayBound>),
    bounds_j: (Option<EndRayBound>, Option<EndRayBound>),
) -> Option<(HashSet<i32>, HashSet<i32>)> {
    let faced = |chain: i32| {
        let bounds = if chain == k { bounds_k } else { bounds_j };
        let (mut first, mut last) = regions.runs_out(chain);
        first &= !bounds.0.is_some_and(|b| b.same_sense);
        last &= !bounds.1.is_some_and(|b| b.same_sense);
        let edges = &chain_edges[chain as usize];
        let mut ends = HashSet::new();
        if !first {
            ends.insert(edges[0].0);
        }

        if !last {
            ends.insert(edges[edges.len() - 1].1);
        }

        ends
    };

    let of_k = faced(k);
    let of_j = faced(j);
    if !of_k.is_empty() && !of_j.is_empty() {
        Some((of_k, of_j))
    } else {
        None
    }
}

/// How near, relative to the approach, a closest point must stand
/// to a chain end to be that end.
const END_TO_END_TOLERANCE: f64 = 1e-9;

/// True when chain j's edge approaches chain k closest END TO END
/// across both chains' end regions: the edge's nearest point to k is an end of j that stops at an
/// end face, and k's nearest point to it is an end of k that does too. Each band stops at its own
/// end face there, so the beads never meet and the approach is no pinch (Center Flows Through: two
/// collinear chains with the fin between them). Any approach with either point interior to its
/// chain, or at an end with no end region (a fade, a growing end), is read as it always was.
pub fn approach_is_end_to_end(
    mesh: &LevelLineMesh,
    bare_of_k: &ChainField,
    edge_of_j: (i32, i32),
    ends: &(HashSet<i32>, HashSet<i32>),
) -> Result<bool, WorkError> {
    let (of_k, of_j) = ends;
    let end_of_j = if of_j.contains(&edge_of_j.0) {
        edge_of_j.0
    } else if of_j.contains(&edge_of_j.1) {
        edge_of_j.1
    } else {
        -1
    };
    if end_of_j < 0 {
        return Ok(false);
    }

    let at_end = bare_of_k.evaluate(mesh.position(end_of_j))?;
    let least =
        bare_of_k.min_along_segment(mesh.position(edge_of_j.0), mesh.position(edge_of_j.1))?;
    let tolerance = END_TO_END_TOLERANCE * ieee_math::max_f64(at_end.distance, 1e-300);
    if (least.point - mesh.position(end_of_j)).length() > tolerance {
        return Ok(false);
    }

    Ok(of_k
        .iter()
        .any(|&v| (at_end.nearest - mesh.position(v)).length() <= tolerance))
}
