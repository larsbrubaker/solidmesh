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

/// C# `ChainOnTheMesh.PerEdgeUnitSetbacks`: each chain edge's setback at size 1 - the chamfer's
/// own, or the fillet's at the edge's dihedral between its two triangles.
pub fn per_edge_unit_setbacks(
    positions: &[vector_math::Vector3],
    indices: &[(i32, i32, i32)],
    chain_edges: &[(i32, i32)],
    profile: super::BandProfile,
) -> Result<Vec<f64>, ChainEdgeNotOnTwoTriangles> {
    let standing = triangle_edges::standing_map(indices);
    let mut per_edge = Vec::with_capacity(chain_edges.len());

    for &(from, to) in chain_edges {
        let (left, right) = sides_of(indices, &standing, from, to)?;
        per_edge.push(if profile == super::BandProfile::Chamfer {
            super::chain_setback::for_chamfer(1.0)
        } else {
            super::chain_setback::for_fillet(
                1.0,
                triangle_edges::normal(positions, indices[left as usize]),
                triangle_edges::normal(positions, indices[right as usize]),
            )
        });
    }

    Ok(per_edge)
}

/// C# `ChainOnTheMesh.UnitSetbacks`: each chain vertex's setback at size 1, the mean of the
/// edges either side of it (the one edge at an open chain's end).
pub fn unit_setbacks(
    positions: &[vector_math::Vector3],
    indices: &[(i32, i32, i32)],
    chain: &[i32],
    is_closed: bool,
    chain_edges: &[(i32, i32)],
    profile: super::BandProfile,
) -> Result<Vec<f64>, ChainEdgeNotOnTwoTriangles> {
    let per_edge = per_edge_unit_setbacks(positions, indices, chain_edges, profile)?;
    let edge_count = chain_edges.len() as i64;
    let mut setbacks = Vec::with_capacity(chain.len());

    for v in 0..chain.len() as i64 {
        let before = if is_closed {
            (v - 1 + edge_count) % edge_count
        } else {
            v - 1
        };
        let after = if is_closed { v % edge_count } else { v };
        let mut sum = 0.0;
        let mut count = 0.0;

        for side in [before, after] {
            if side >= 0 && (side as usize) < per_edge.len() {
                sum += per_edge[side as usize];
                count += 1.0;
            }
        }

        setbacks.push(sum / count);
    }

    Ok(setbacks)
}
