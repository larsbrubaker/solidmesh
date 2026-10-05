//! How the band flows over a chain's end face. Ports `Bevel/LevelLine/EndFaceFlow.cs` (a C# static
//! class): so far its constants, the facet normal ([`normal_of`]), the side-turn test
//! ([`turns_along_a_side`]) and the end-face bend ([`bends`]); the flows land with later packages.

use vector_math::Vector3;

use super::triangle_edges::{self, StandingMap};

/// How many FULL facets past the end vertex's own one, each between two soft joins, make a round.
pub const ROUND_RUN_JOINS: i32 = 3;

/// How far the round's facet widths and join turns may differ, largest to smallest: loose for a
/// round tessellated a little unevenly, tight against a flat end face ahead of a rounding.
pub const ROUND_RUN_SPREAD: f64 = 1.25;

/// How far two triangles across an edge may turn and still be one END FACE. Far above the noise a
/// flat face picks up in an exported file (float coordinates put its facets thousandths of a
/// degree apart) and under the facet turn of any rounding a part is tessellated with, so a planar
/// face, however it is triangulated, is one end region and the first facet of a rounding is not.
pub const END_FACE_BEND_DEGREES: f64 = 0.5;

/// The joins of one round turn about one axis (the cylinder's): within about 10 degrees, as a
/// cosine (C# `static readonly RoundRunAxisCosine`).
pub fn round_run_axis_cosine() -> f64 {
    (10.0 * std::f64::consts::PI / 180.0).cos()
}

/// [`END_FACE_BEND_DEGREES`] as a cosine (C# `static readonly EndFaceBendCosine`).
fn end_face_bend_cosine() -> f64 {
    (END_FACE_BEND_DEGREES * std::f64::consts::PI / 180.0).cos()
}

/// True when edge `a b` stands on nothing, or one of its triangles faces one of `side_normals`
/// within 45 degrees (either way round when `either_way`).
pub fn turns_along_a_side(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    standing: &StandingMap,
    a: i32,
    b: i32,
    side_normals: &[Vector3],
    either_way: bool,
) -> bool {
    let Some(on) = standing.get(&triangle_edges::key(a, b)) else {
        return true;
    };

    on.iter().any(|&triangle| {
        side_normals.iter().any(|&side| {
            let facing = normal_of(positions, indices[triangle as usize]).dot(side);
            (if either_way { facing.abs() } else { facing }) > 0.5f64.sqrt()
        })
    })
}

/// The unit normal of an input triangle.
pub fn normal_of(positions: &[Vector3], t: (i32, i32, i32)) -> Vector3 {
    (positions[t.1 as usize] - positions[t.0 as usize])
        .cross(positions[t.2 as usize] - positions[t.0 as usize])
        .get_normal()
}

/// True when the two triangles standing on the edge turn by more than
/// [`END_FACE_BEND_DEGREES`] - see the end floods - or the edge does not stand on exactly two.
pub fn bends(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    standing: &StandingMap,
    a: i32,
    b: i32,
) -> bool {
    let on = match standing.get(&triangle_edges::key(a, b)) {
        Some(on) if on.len() == 2 => on,
        _ => return true,
    };

    let normal_of = |triangle: i32| {
        let t = indices[triangle as usize];
        (positions[t.1 as usize] - positions[t.0 as usize])
            .cross(positions[t.2 as usize] - positions[t.0 as usize])
    };

    let first = normal_of(on[0]);
    let second = normal_of(on[1]);
    let lengths = first.length() * second.length();
    // A NaN length reads as no bend, as C#'s `!(lengths > 0)` does.
    if lengths.is_nan() || lengths <= 0.0 {
        return false;
    }

    first.dot(second) < end_face_bend_cosine() * lengths
}
