//! The strip clip's VALLEY mode. Ports `Bevel/LevelLine/StripClip.Valley.cs` (a partial of the C#
//! static class `StripClip`): so far which side of a plane the other face says is kept
//! ([`other_says`]), the one run of a loop lying in a single plane ([`one_plane_run`]) and the fan
//! from a corner facing the wall ([`fan_facing`]), and the valley's cap ([`valley_cap`]).

use std::collections::HashSet;

use vector_math::Vector3;

use indexmap::{IndexMap, IndexSet};

use super::strip_clip::{directed, format_e0, note, ClipPlane};
use super::triangle_edges;
use super::{LevelLineMesh, LevelLineMeshError, LevelLineTriangle};

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

/// C# `StripClip.ValleyCap`: closes a valley's hole. The drop leaves the walls' crease ring and
/// the kept round's clip loop; each wall's cap is its stretch of the ring, the shared crease on to
/// the tie beside the stretch's end, its clip curve, and the shared crease back from the other tie
/// - fanned in the wall's plane by [`fan_facing`]. The faces laid join `strip`.
pub fn valley_cap(
    mesh: &mut LevelLineMesh,
    strip: &mut HashSet<i32>,
    planes: &[ClipPlane],
    dropped: &[LevelLineTriangle],
    crease: &IndexSet<(i32, i32)>,
) -> Result<bool, LevelLineMeshError> {
    let mut outline: IndexSet<(i32, i32)> = crease
        .iter()
        .map(|&(a, b)| triangle_edges::key(a, b))
        .collect();
    for face in dropped {
        for (p, q) in triangle_edges::corners((face.a, face.b, face.c)) {
            if mesh.triangles_on(p, q).len() == 1 {
                outline.insert(triangle_edges::key(p, q));
            }
        }
    }

    let mut next: IndexMap<i32, i32> = IndexMap::new();
    for &(low, high) in &outline {
        let standing = mesh.triangles_on(low, high);
        if standing.len() != 1 {
            continue;
        }

        let standing0 = standing[0];
        let (a, b) = directed(mesh.triangles()[standing0 as usize], low, high);
        if let Some(&other) = next.get(&b) {
            note(|| {
                let reading = |p: &ClipPlane, v: i32| {
                    if p.2.contains(&v) {
                        "on".to_string()
                    } else {
                        format_e0((mesh.position(v) - p.0).dot(p.1))
                    }
                };
                let on_planes: Vec<String> = planes
                    .iter()
                    .map(|p| format!("{},{}", reading(p, a), reading(p, other)))
                    .collect();
                format!(
                    "valley outline branches at {b} {}: from {a} {} and {other} {}; stands on {}; planes {}",
                    mesh.position(b),
                    mesh.position(a),
                    mesh.position(other),
                    if strip.contains(&standing0) { "strip" } else { "kept" },
                    on_planes.join("/")
                )
            });
            return Ok(false);
        }

        next.insert(b, a);
    }

    let mut loops: Vec<Vec<i32>> = Vec::new();
    while !next.is_empty() {
        let mut loop_ = Vec::new();
        let mut at = *next.keys().min().expect("not empty");
        while let Some(to) = next.shift_remove(&at) {
            loop_.push(at);
            at = to;
        }

        if at != loop_[0] {
            note(|| format!("valley outline does not close at {at}"));
            return Ok(false);
        }

        loops.push(loop_);
    }

    let crease_vertices: HashSet<i32> = crease.iter().flat_map(|&(a, b)| [a, b]).collect();
    let (ring, clip): (Vec<&Vec<i32>>, Vec<&Vec<i32>>) = loops
        .iter()
        .partition(|l| l.iter().any(|v| crease_vertices.contains(v)));
    if ring.len() != 1 || clip.len() != 1 {
        note(|| {
            format!(
                "valley outline: {} crease rings, {} clip loops",
                ring.len(),
                clip.len()
            )
        });
        return Ok(false);
    }

    let (ring, clip) = (ring[0].clone(), clip[0].clone());
    for w in 0..planes.len() {
        let on = &planes[w].2;
        let stretch = one_plane_run(&ring, on, planes);
        let curve = one_plane_run(&clip, on, planes);
        let (Some(stretch), Some(curve)) = (stretch, curve) else {
            note(|| format!("valley wall {w}: no single stretch or clip curve"));
            return Ok(false);
        };

        // The shared crease carries on from the stretch's end to the NEARER tie.
        let gap = |p: i32, q: i32| (mesh.position(p) - mesh.position(q)).length();
        let stretch_end = stretch[stretch.len() - 1];
        if gap(stretch_end, curve[0]) > gap(stretch_end, curve[curve.len() - 1]) {
            note(|| format!("valley wall {w}: the clip curve runs the wrong way round"));
            return Ok(false);
        }

        let polygon: Vec<i32> = stretch.iter().chain(curve.iter()).copied().collect();
        if !fan_facing(mesh, strip, &polygon, planes[w].1)? {
            note(|| format!("valley wall {w}: no fan corner sees the whole cap"));
            return Ok(false);
        }
    }

    Ok(true)
}
