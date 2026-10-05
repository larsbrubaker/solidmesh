//! The strip clip's VALLEY mode. Ports `Bevel/LevelLine/StripClip.Valley.cs` (a partial of the C#
//! static class `StripClip`): so far which side of a plane the other face says is kept
//! ([`other_says`]) and the one run of a loop lying in a single plane ([`one_plane_run`]); the
//! valley cap lands with later packages.

use std::collections::HashSet;

use vector_math::Vector3;

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
