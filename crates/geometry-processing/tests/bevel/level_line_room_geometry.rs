//! The box with a square hole and the L prism, the solids the room-reading tests build from. The
//! several-chain plates are in `several_chains_fixtures.rs` and the feature edges in
//! `level_line_feature_edges.rs`.

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

/// A closed box with a square hole straight through it in z. The mouth rim
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

/// The L-shaped prism whose room is the narrow arm's own width.
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

/// A flat top face whose outline is two square lobes
/// joined by a narrow neck, extruded down to a closed solid; the top face's whole rim is the chain.
/// The true room is half `neck_width`. The usual sizes are lobe 10, neck width 2, neck
/// length 4, half height 4.
pub fn hourglass_solid(
    lobe: f64,
    neck_width: f64,
    neck_length: f64,
    half_height: f64,
) -> SolidWithChain {
    let half = lobe / 2.0;
    let neck = neck_width / 2.0;
    let inner = neck_length / 2.0;
    let outer = inner + lobe;

    // Four columns of four points each: the two lobes' far edges and the two neck mouths.
    let xs = [-outer, -inner, inner, outer];
    let ys = [-half, -neck, neck, half];
    let mut positions = Vec::new();
    for x in xs {
        for y in ys {
            positions.push(Vector3::new(x, y, half_height));
        }
    }

    let top = positions.len() as i32;
    for i in 0..top as usize {
        positions.push(Vector3::new(positions[i].x, positions[i].y, -half_height));
    }

    let p = |column: i32, row: i32| (column * 4) + row;

    // The top face as quads: the left lobe in three bands, the neck, the right lobe in three bands.
    let mut quads = Vec::new();
    for row in [0, 1, 2] {
        quads.push((p(0, row), p(1, row), p(1, row + 1), p(0, row + 1)));
        quads.push((p(2, row), p(3, row), p(3, row + 1), p(2, row + 1)));
    }

    quads.push((p(1, 1), p(2, 1), p(2, 2), p(1, 2)));

    let mut indices = Vec::new();
    for (a, b, c, d) in quads {
        indices.push((a, b, c));
        indices.push((a, c, d));
        indices.push((top + a, top + c, top + b));
        indices.push((top + a, top + d, top + c));
    }

    // The rim, walked counter-clockwise seen from +z, and a wall quad under each of its edges.
    let rim = vec![
        p(0, 0),
        p(1, 0),
        p(1, 1),
        p(2, 1),
        p(2, 0),
        p(3, 0),
        p(3, 1),
        p(3, 2),
        p(3, 3),
        p(2, 3),
        p(2, 2),
        p(1, 2),
        p(1, 3),
        p(0, 3),
        p(0, 2),
        p(0, 1),
    ];
    for i in 0..rim.len() {
        let a = rim[i];
        let b = rim[(i + 1) % rim.len()];
        indices.push((a, top + a, top + b));
        indices.push((a, top + b, b));
    }

    (positions, indices, rim)
}
