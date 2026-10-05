//! The end face's plane at an open chain's end as the rails meet it. Ports
//! `Bevel/LevelLine/EndFaceCreasePlane.cs` (a C# static class), read by
//! `end_face_sense::end_ray_bounds`.

use vector_math::Vector3;

use super::end_face_sense::{shared_corners, Corners, COPLANAR_TOLERANCE};

/// C# `EndFaceCreasePlane.Through`: the plane through the end vertex and BOTH side-to-end creases
/// there, where exactly two creases leave the end vertex and are not in line; otherwise `fallback`
/// (the largest end-face triangle's normal), which also orients the answer.
///
/// Each rail lands on its side's crease, and the ray's reach is read so that rail point just meets
/// the plane - a tie by construction. A plane read off one triangle holds the other triangle's
/// crease only to the input's float noise: on the ridge's stepped corner
/// (BevelEveryFaceEdgeSubsetSurveyTests.TheRidgesSteppedCornerWithItsWallCreaseBuilds) the end at
/// (-10, -108.5, 6.67) has two end triangles of near-equal area, the larger tilted 9.5e-8 off the
/// crease 4-5, so the ray stopped 1.2e-7 short of the rail vertex on that crease, the vertex read
/// off the level line, its rail lost the run past the end and the strip refused. Both creases are
/// mesh edges, so the plane through them holds both rails, and it reads the same for a mirror pair
/// whatever the index order (HoleFillOneLoopCornerTests.TheMirroredHolesAreMirrored).
pub fn through(
    positions: &[Vector3],
    end: i32,
    side_triangles: &[Corners],
    end_triangles: &[Corners],
    fallback: Vector3,
) -> Vector3 {
    // C#'s HashSet<int> keeps insertion order here (it is only added to), so a Vec without repeats
    // reads its two members in the same order.
    let mut crease_ends: Vec<i32> = Vec::new();
    for &side in side_triangles {
        for &other in end_triangles {
            let shared = shared_corners(side, other);
            if shared.len() == 2 && shared.contains(&end) {
                if let Some(&c) = shared.iter().find(|&&c| c != end) {
                    if !crease_ends.contains(&c) {
                        crease_ends.push(c);
                    }
                }
            }
        }
    }

    if crease_ends.len() != 2 {
        return fallback;
    }

    let v = positions[end as usize];
    let mut normal = (positions[crease_ends[0] as usize] - v)
        .get_normal()
        .cross((positions[crease_ends[1] as usize] - v).get_normal());
    if normal.length() < COPLANAR_TOLERANCE {
        return fallback;
    }

    normal = normal.get_normal();
    if normal.dot(fallback) < 0.0 {
        -normal
    } else {
        normal
    }
}
