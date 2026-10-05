//! The strip clip's VALLEY mode. Ports `Bevel/LevelLine/StripClip.Valley.cs` (a partial of the C#
//! static class `StripClip`): so far which side of a plane the other face says is kept
//! ([`other_says`]), the one run of a loop lying in a single plane ([`one_plane_run`]) and the fan
//! from a corner facing the wall ([`fan_facing`]); the
//! valley cap lands with later packages.

use std::collections::HashSet;

use vector_math::Vector3;

use super::{LevelLineMesh, LevelLineMeshError};

/// Whether a signed distance stands on the side the other face keeps: above zero in a valley,
/// below zero otherwise.
pub fn other_says(distance: f64, valley: bool) -> bool {
    if valley {
        distance > 0.0
    } else {
        distance < 0.0
    }
}

/// The ONE run of `loop_vertices` between two consecutive corners (vertices on every plane) whose
/// vertices all lie in `on` and that has more than two of them; `None` where there is no such run
/// or more than one.
pub fn one_plane_run(
    loop_vertices: &[i32],
    on: &HashSet<i32>,
    planes: &[(Vector3, Vector3, HashSet<i32>)],
) -> Option<Vec<i32>> {
    let corner = |v: i32| planes.iter().all(|p| p.2.contains(&v));
    let corners: Vec<usize> = (0..loop_vertices.len())
        .filter(|&i| corner(loop_vertices[i]))
        .collect();
    let mut found: Option<Vec<i32>> = None;
    for c in 0..corners.len() {
        let mut run = Vec::new();
        let mut i = corners[c];
        loop {
            run.push(loop_vertices[i]);
            if run.len() > 1 && i == corners[(c + 1) % corners.len()] {
                break;
            }

            i = (i + 1) % loop_vertices.len();
        }

        if run.len() > 2 && run.iter().all(|v| on.contains(v)) {
            if found.is_some() {
                return None;
            }

            found = Some(run);
        }
    }

    found
}

/// C# `StripClip.FanFacing`: fans the polygon from the first corner whose every fan triangle faces
/// along the wall's `normal` (a triangle on a straight stretch of the outline has no area and faces
/// nowhere); the faces laid join `strip`. False where no corner sees the whole polygon.
pub fn fan_facing(
    mesh: &mut LevelLineMesh,
    strip: &mut HashSet<i32>,
    polygon: &[i32],
    normal: Vector3,
) -> Result<bool, LevelLineMeshError> {
    let n = polygon.len();
    // LINQ's Max: NaN reads as the smallest; an empty polygon throws (as the C# does).
    let mut scale: Option<f64> = None;
    for &v in polygon {
        let x = (mesh.position(v) - mesh.position(polygon[0])).length();
        scale = Some(match scale {
            Some(value) if !(x > value || value.is_nan()) => value,
            _ => x,
        });
    }
    let scale = scale.expect("a polygon has corners");
    for apex in 0..n {
        let faces: Vec<(i32, i32, i32)> = (1..n.saturating_sub(1))
            .map(|i| {
                (
                    polygon[apex],
                    polygon[(apex + i) % n],
                    polygon[(apex + i + 1) % n],
                )
            })
            .collect();
        let faces_along = |&(a, b, c): &(i32, i32, i32)| {
            (mesh.position(b) - mesh.position(a))
                .cross(mesh.position(c) - mesh.position(a))
                .dot(normal)
                > 1e-9 * scale * scale
        };
        if faces.iter().all(faces_along) {
            for (a, b, c) in faces {
                strip.insert(mesh.add_face(a, b, c, 0)?);
            }

            return Ok(true);
        }
    }

    Ok(false)
}
