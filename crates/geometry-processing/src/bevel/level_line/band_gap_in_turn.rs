//! The band built one chain at a time. Ports `Bevel/LevelLine/BandGap.InTurn.cs` (a partial of the
//! C# static class `BandGap`): so far the walk along a split edge's pieces ([`walk_along`]); the
//! build lands with later packages.

use std::collections::HashMap;

use vector_math::Vector3;

use super::triangle_edges::{self, StandingMap};

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
