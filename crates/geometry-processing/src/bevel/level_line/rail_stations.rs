//! A rail's stations. Ports `Bevel/LevelLine/RailStations.cs` (a C# static class): so far the
//! pieces an edge was split into ([`pieces`]) and the edge key ([`key`]); the stations land with
//! later packages.

use std::collections::HashMap;

/// The pieces edge `a b` stands as after its splits (each split edge's key mapped to the vertex
/// put in it), in order from `a` to `b`.
pub fn pieces(a: i32, b: i32, splits: &HashMap<(i32, i32), i32>) -> Vec<(i32, i32)> {
    let Some(&m) = splits.get(&key(a, b)) else {
        return vec![(a, b)];
    };

    let mut found = pieces(a, m, splits);
    found.extend(pieces(m, b, splits));
    found
}

/// The (low, high) key of an edge.
pub fn key(a: i32, b: i32) -> (i32, i32) {
    if a < b {
        (a, b)
    } else {
        (b, a)
    }
}
