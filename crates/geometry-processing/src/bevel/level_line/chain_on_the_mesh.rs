//! A picked chain read on the mesh it was picked from: its edges in its own direction and the two
//! triangles each stands between. Ports `Bevel/LevelLine/ChainOnTheMesh.cs` (a C# static class,
//! so a module of functions), with the setback-step fraction ([`SETBACK_STEP_FRACTION`]); the
//! setback steps and unit setbacks land with their method package.

use super::triangle_edges::{self, StandingMap};
use super::ChainEdgeNotOnTwoTriangles;

/// The chain's edges, each in the chain's own direction.
pub fn chain_edges(chain: &[i32], is_closed: bool) -> Vec<(i32, i32)> {
    let mut edges = Vec::new();
    let mut i = 0;
    while i + 1 < chain.len() {
        edges.push((chain[i], chain[i + 1]));
        i += 1;
    }

    if is_closed {
        edges.push((chain[chain.len() - 1], chain[0]));
    }

    edges
}

/// The two triangles a chain edge `from -> to` stands between, as (left, right): the left one
/// traverses the edge in the chain's direction. Errors where the edge does not stand on exactly
/// two triangles, or its two triangles do not wind it consistently (C# throws
/// `ChainEdgeNotOnTwoTrianglesException`, with the same messages).
pub fn sides_of(
    indices: &[(i32, i32, i32)],
    standing: &StandingMap,
    from: i32,
    to: i32,
) -> Result<(i32, i32), ChainEdgeNotOnTwoTriangles> {
    let pair = match standing.get(&triangle_edges::key(from, to)) {
        Some(pair) if pair.len() == 2 => pair,
        _ => {
            return Err(ChainEdgeNotOnTwoTriangles::new(
                from,
                to,
                format!("chain edge {from}-{to} does not stand on exactly two triangles"),
            ))
        }
    };

    let left = if triangle_edges::traverses(indices[pair[0] as usize], from, to) {
        pair[0]
    } else {
        pair[1]
    };
    if !triangle_edges::traverses(indices[left as usize], from, to) {
        return Err(ChainEdgeNotOnTwoTriangles::new(
            from,
            to,
            format!("chain edge {from}-{to} is not wound consistently by its two triangles"),
        ));
    }

    Ok((left, if left == pair[0] { pair[1] } else { pair[0] }))
}

/// C# `ChainOnTheMesh.SetbackStepFraction`: how much two neighbouring edges' setbacks may differ,
/// as a fraction of the larger, and still share a sharp turn of one chain. Below it the vertex
/// takes their mean; above it the chain is split there.
pub const SETBACK_STEP_FRACTION: f64 = 0.05;
