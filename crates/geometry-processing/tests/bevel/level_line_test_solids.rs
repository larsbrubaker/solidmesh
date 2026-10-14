//! The prism on a star-shaped outline, the block with a blind square pocket, and the cube with a
//! vertex in the middle of one top edge. The cube itself is `level_line_test_geometry::cube_solid`.

#![allow(dead_code)]

use vector_math::{Vector2, Vector3};

use crate::level_line_room_geometry::SolidWithChain;
use crate::level_line_test_geometry::cube_solid;

/// A prism on a counter-clockwise outline in xy, from z = 0 to `height`,
/// capped top and bottom by a fan from `star_point`, which must see every corner of the outline.
/// Each wall is one quad of two triangles; the top rim, in outline order, is handed back as the
/// chain a rim pick takes. The usual height is 4.
pub fn prism_on_outline(outline: &[Vector2], star_point: Vector2, height: f64) -> SolidWithChain {
    let n = outline.len();

    let mut positions = Vec::new();
    let mut top = Vec::new();
    let mut bottom = Vec::new();
    for p in outline {
        top.push(positions.len() as i32);
        positions.push(Vector3::new(p.x, p.y, height));
    }

    for p in outline {
        bottom.push(positions.len() as i32);
        positions.push(Vector3::new(p.x, p.y, 0.0));
    }

    let centre = star_point;
    let top_centre = positions.len() as i32;
    positions.push(Vector3::new(centre.x, centre.y, height));
    let bottom_centre = positions.len() as i32;
    positions.push(Vector3::new(centre.x, centre.y, 0.0));

    let mut indices = Vec::new();
    for i in 0..n {
        let j = (i + 1) % n;
        indices.push((top_centre, top[i], top[j]));
        indices.push((bottom_centre, bottom[j], bottom[i]));
        indices.push((top[i], bottom[i], bottom[j]));
        indices.push((top[i], bottom[j], top[j]));
    }

    (positions, indices, top)
}

/// A closed block with a square BLIND pocket sunk into its top. The floor's rim
/// is a closed chain of four concave edges. The usual sizes are outer half 2, pocket half 1,
/// half height 1, floor z 0.
pub fn square_pocket(
    outer_half: f64,
    pocket_half: f64,
    half_height: f64,
    floor_z: f64,
) -> SolidWithChain {
    let corners = [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)];
    let mut positions = Vec::new();
    for (x, y) in corners {
        positions.push(Vector3::new(x * outer_half, y * outer_half, half_height));
    }

    for (x, y) in corners {
        positions.push(Vector3::new(x * pocket_half, y * pocket_half, half_height));
    }

    for (x, y) in corners {
        positions.push(Vector3::new(x * outer_half, y * outer_half, -half_height));
    }

    for (x, y) in corners {
        positions.push(Vector3::new(x * pocket_half, y * pocket_half, floor_z));
    }

    let top_outer = |k: i32| k % 4;
    let mouth = |k: i32| 4 + (k % 4);
    let low_outer = |k: i32| 8 + (k % 4);
    let floor = |k: i32| 12 + (k % 4);

    let mut indices = Vec::new();
    for k in 0..4 {
        // The top face, an annulus round the mouth wound outward (+z).
        indices.push((top_outer(k), top_outer(k + 1), mouth(k + 1)));
        indices.push((top_outer(k), mouth(k + 1), mouth(k)));

        // The outer wall, facing out of the block.
        indices.push((top_outer(k), low_outer(k), low_outer(k + 1)));
        indices.push((top_outer(k), low_outer(k + 1), top_outer(k + 1)));

        // The pocket's wall, facing into the pocket.
        indices.push((mouth(k), floor(k + 1), floor(k)));
        indices.push((mouth(k), mouth(k + 1), floor(k + 1)));
    }

    // The bottom of the block, wound downward (-z), and the pocket's floor, wound up into the
    // pocket.
    indices.push((low_outer(0), low_outer(2), low_outer(1)));
    indices.push((low_outer(0), low_outer(3), low_outer(2)));
    indices.push((floor(0), floor(1), floor(2)));
    indices.push((floor(0), floor(2), floor(3)));

    (positions, indices, vec![12, 13, 14, 15])
}

/// `cube_solid` with one more vertex, `Mid` (index 8), in the middle
/// of the top front edge 4-5, the two triangles on that edge split through it. A pick of
/// [4, Mid] is an OPEN chain that ends at Mid on a crease that CONTINUES (Mid-5). The usual
/// half is 1.
pub fn cube_with_mid_edge_vertex(half: f64) -> (Vec<Vector3>, Vec<(i32, i32, i32)>, i32) {
    let (cube, cube_indices) = cube_solid(half);
    let mut positions = cube.clone();
    let mid = positions.len() as i32;
    positions.push((cube[4] + cube[5]) / 2.0);
    let mut indices = Vec::new();
    for t in cube_indices {
        if t == (4, 5, 6) {
            indices.push((4, mid, 6));
            indices.push((mid, 5, 6));
        } else if t == (0, 5, 4) {
            indices.push((0, 5, mid));
            indices.push((0, mid, 4));
        } else {
            indices.push(t);
        }
    }

    (positions, indices, mid)
}
