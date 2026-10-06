//! Port of `LevelLineTestGeometry.BoxWithSquareHole` (test helper `LevelLine/LevelLineTestGeometry.cs`)
//! and `OneRoomReadingTests.LPrism`, the solids the room-reading tests build from. The several-chain
//! plates are in `several_chains_fixtures.rs` and `FeatureEdgesOver` in `level_line_feature_edges.rs`.

#![allow(dead_code)]

use vector_math::Vector3;

/// A triangle-soup solid and the chains of a pick on it: the vertex positions, the triangles as
/// index triples into those positions, and each chain as a loop of vertex indices.
pub type SolidWithChains = (Vec<Vector3>, Vec<(i32, i32, i32)>, Vec<Vec<i32>>);

/// A triangle-soup solid and the one chain of a pick on it: the vertex positions, the triangles as
/// index triples into those positions, and the chain as a loop of vertex indices.
pub type SolidWithChain = (Vec<Vector3>, Vec<(i32, i32, i32)>, Vec<i32>);

/// A triangle-soup solid and two chains on it: the vertex positions, the triangles as index
/// triples into those positions, then the first and the second chain, each a loop of vertex
/// indices.
pub type SolidWithTwoChains = (Vec<Vector3>, Vec<(i32, i32, i32)>, Vec<i32>, Vec<i32>);

/// C# `BoxWithSquareHole`: a closed box with a square hole straight through it in z. The mouth rim
/// on the top face (4, 5, 6, 7) is a closed chain.
pub fn box_with_square_hole(outer_half: f64, hole_half: f64, half_height: f64) -> SolidWithChain {
    let corners = [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)];
    let mut positions = Vec::new();
    for (half, z) in [
        (outer_half, half_height),
        (hole_half, half_height),
        (outer_half, -half_height),
        (hole_half, -half_height),
    ] {
        for (x, y) in corners {
            positions.push(Vector3::new(x * half, y * half, z));
        }
    }

    (positions, picture_frame(), vec![4, 5, 6, 7])
}

/// The picture frame `BoxWithSquareHole` and `PlateWithOffsetSquareHole` are built from: rings
/// top-outer 0..4, top-inner 4..8, low-outer 8..12, low-inner 12..16.
fn picture_frame() -> Vec<(i32, i32, i32)> {
    let top_outer = |k: i32| k % 4;
    let top_inner = |k: i32| 4 + (k % 4);
    let low_outer = |k: i32| 8 + (k % 4);
    let low_inner = |k: i32| 12 + (k % 4);

    let mut indices = Vec::new();
    for k in 0..4 {
        // Top face, an annulus wound outward (+z).
        indices.push((top_outer(k), top_outer(k + 1), top_inner(k + 1)));
        indices.push((top_outer(k), top_inner(k + 1), top_inner(k)));

        // Bottom face, the same annulus the other way about (-z).
        indices.push((low_outer(k), low_inner(k), low_inner(k + 1)));
        indices.push((low_outer(k), low_inner(k + 1), low_outer(k + 1)));

        // The outer wall, facing out of the block.
        indices.push((top_outer(k), low_outer(k), low_outer(k + 1)));
        indices.push((top_outer(k), low_outer(k + 1), top_outer(k + 1)));

        // The hole's wall, facing into the hole.
        indices.push((top_inner(k), low_inner(k + 1), low_inner(k)));
        indices.push((top_inner(k), top_inner(k + 1), low_inner(k + 1)));
    }

    indices
}

/// C# `OneRoomReadingTests.LPrism`: the L-shaped prism whose room is the narrow arm's own width.
pub fn l_prism() -> SolidWithChain {
    let outline = [
        (0.0, 0.0),
        (6.0, 0.0),
        (6.0, 2.0),
        (2.0, 2.0),
        (2.0, 5.0),
        (0.0, 5.0),
    ];

    let n = outline.len() as i32;
    let height = 4.0;

    let mut positions = Vec::new();
    let mut top = Vec::new();
    let mut bottom = Vec::new();
    for (x, y) in outline {
        top.push(positions.len() as i32);
        positions.push(Vector3::new(x, y, height));
    }

    for (x, y) in outline {
        bottom.push(positions.len() as i32);
        positions.push(Vector3::new(x, y, 0.0));
    }

    // A star point inside the L, so every top triangle has a vertex off the rim.
    let top_centre = positions.len() as i32;
    positions.push(Vector3::new(1.0, 1.0, height));
    let bottom_centre = positions.len() as i32;
    positions.push(Vector3::new(1.0, 1.0, 0.0));

    let mut indices = Vec::new();
    for i in 0..n as usize {
        let j = (i + 1) % n as usize;
        indices.push((top_centre, top[i], top[j]));
        indices.push((bottom_centre, bottom[j], bottom[i]));
        indices.push((top[i], bottom[i], bottom[j]));
        indices.push((top[i], bottom[j], top[j]));
    }

    (positions, indices, top)
}
