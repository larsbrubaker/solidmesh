//! The band built one chain at a time. Ports `Bevel/LevelLine/BandGap.InTurn.cs` (a partial of the
//! C# static class `BandGap`): so far the walk along a split edge's pieces ([`walk_along`]), the
//! in-turn test ([`takes_open_chains_in_turn`]), a chain's ends carried onto the mesh an earlier
//! cut left ([`ends_on_the_mesh`], [`nearest_on_the_edge`]) and the built mesh's coplanar turn; the
//! build lands with later packages.

use std::collections::HashMap;

use indexmap::IndexMap;
use vector_math::Vector3;

use super::triangle_edges::{self, StandingMap};
use super::BandGapResult;

/// C# `BandGap.CoplanarAfterABuild`: how far (one minus the normals' dot) two faces of a built
/// mesh may turn and still be one face.
pub const COPLANAR_AFTER_A_BUILD: f64 = 1e-9;

/// C# `BandGap.TakesOpenChainsInTurn`: true when `gap`, the pick-wide cut, is to be built in turn:
/// several chains, every one open, the cut built (so the pick-wide room cleared the size), and no
/// loop shared between chains - a loop two chains' fronts both reach is a handover the chains
/// alone cannot build.
pub fn takes_open_chains_in_turn(gap: Option<&BandGapResult>, closed: &[bool]) -> bool {
    gap.is_some_and(|gap| {
        gap.built()
            && closed.len() > 1
            && closed.iter().all(|&c| !c)
            && !gap
                .loops
                .iter()
                .any(|l| l.has_mixed_owners || l.owners.len() > 1)
    })
}

/// C# `BandGap.EndsOnTheMesh`: an open chain whose END an earlier chain's band cut away (the two
/// met at that vertex, so it lay in the earlier band) starts instead where its first edge now
/// enters the mesh - the vertex the earlier cut left on that edge, at the earlier band's rim.
/// `None` where no such vertex is: the chain then lies inside the earlier band and the two
/// interact. The neighbour map keeps C#'s `Dictionary` order, which [`nearest_on_the_edge`]
/// breaks its ties by.
pub fn ends_on_the_mesh(
    chain: &[i32],
    positions: &[Vector3],
    neighbours: &IndexMap<i32, Vec<i32>>,
) -> Option<Vec<i32>> {
    if neighbours.contains_key(&chain[0]) && neighbours.contains_key(&chain[chain.len() - 1]) {
        return Some(chain.to_vec());
    }

    let mut list = chain.to_vec();
    for _ in 0..2 {
        if list.len() < 2 {
            return None;
        }

        if !neighbours.contains_key(&list[0]) {
            let entry = nearest_on_the_edge(list[0], list[1], positions, neighbours);
            if entry < 0 {
                return None;
            }

            if entry == list[1] {
                list.remove(0);
            } else {
                list[0] = entry;
            }
        }

        list.reverse();
    }

    if list.len() < 2 {
        None
    } else {
        Some(list)
    }
}

/// C# `BandGap.NearestOnTheEdge`: the mesh vertex (a key of `neighbours`) on the segment from
/// `from` to `to` nearest `from`, or -1; the first in the map's order wins a tie.
pub fn nearest_on_the_edge(
    from: i32,
    to: i32,
    positions: &[Vector3],
    neighbours: &IndexMap<i32, Vec<i32>>,
) -> i32 {
    let start = positions[from as usize];
    let along = positions[to as usize] - start;
    let length = along.length();
    if length <= 0.0 {
        return -1;
    }

    let direction = along / length;
    let on_the_line = 1e-7 * length;
    let mut nearest = -1;
    let mut nearest_t = f64::INFINITY;
    for &v in neighbours.keys() {
        let offset = positions[v as usize] - start;
        let t = offset.dot(direction);
        if t > on_the_line
            && t <= length + on_the_line
            && t < nearest_t
            && (offset - direction * t).length() <= on_the_line
        {
            nearest = v;
            nearest_t = t;
        }
    }

    nearest
}

/// The vertices from `from` (exclusive) to `to` (inclusive) along the straight line between them,
/// stepping to the nearest neighbour ahead on the line each time: `[to]` where the edge still
/// stands, `None` where the walk cannot be made.
pub fn walk_along(
    from: i32,
    to: i32,
    positions: &[Vector3],
    standing: &StandingMap,
    neighbours: &HashMap<i32, Vec<i32>>,
) -> Option<Vec<i32>> {
    if standing.contains_key(&triangle_edges::key(from, to)) {
        return Some(vec![to]);
    }

    let start = positions[from as usize];
    let along = positions[to as usize] - start;
    let length = along.length();
    if length <= 0.0 {
        return None;
    }

    let direction = along / length;
    let on_the_line = 1e-7 * length;
    let mut walk = Vec::new();
    let mut at = from;
    let mut reached = 0.0;
    while at != to {
        let mut next = -1;
        let mut next_reach = f64::INFINITY;
        let around = neighbours.get(&at)?;
        for &n in around {
            let offset = positions[n as usize] - start;
            let t = offset.dot(direction);
            if t <= reached + on_the_line
                || t > length + on_the_line
                || (offset - direction * t).length() > on_the_line
            {
                continue;
            }

            // The nearest vertex on ahead: a split edge's pieces run end to end, so the nearest is
            // the next.
            if t < next_reach {
                next = n;
                next_reach = t;
            }
        }

        if next < 0 {
            return None;
        }

        walk.push(next);
        at = next;
        reached = next_reach;
    }

    Some(walk)
}
