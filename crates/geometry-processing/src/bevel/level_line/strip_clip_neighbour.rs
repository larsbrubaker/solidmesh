//! The strip clip's driver. Ports the entry points of `Bevel/LevelLine/StripClip.cs` (the C# static
//! class `StripClip`): `TryClipAndCap` on the laid list ([`try_clip_and_cap`]) and on the strip
//! itself ([`try_clip_and_cap_strip`]), which split the growing neighbour's open edges into one
//! neighbour per connected run, and `ClipOnOneNeighbour` ([`clip_on_one_neighbour`]), which reads
//! one neighbour's planes, ties, splits, drops and caps. The mesh work it drives is in
//! `strip_clip` (the ridge clip) and `strip_clip_valley` (the valley clip).

use std::collections::HashSet;
use std::collections::VecDeque;

use indexmap::{IndexMap, IndexSet};
use vector_math::cs_math;

use super::super::float_grid;
use super::strip_clip::{
    attached, cap, directed, grow, insert_ties, last_outcome, note, split_along, ClipPlane,
};
use super::strip_clip_valley::{insert_valley_ties, valley_cap, valley_planes};
use super::triangle_edges;
use super::{LevelLineMesh, LevelLineMeshError, LevelLineTriangle, LevelLineWork};

/// C# `TryClipAndCap(mesh, laid, start, kept)`: clips and caps where the fill ran a rail past a
/// growing neighbour; true with nothing changed when no loop edge of `kept` is virtual (stands on
/// no kept triangle), which is every fill that reached no opposite-sense crease. False when the
/// clip cannot close. The strip is `laid[start..]`.
pub fn try_clip_and_cap(
    mesh: &mut LevelLineMesh,
    laid: &mut Vec<i32>,
    start: usize,
    kept: &mut Vec<(i32, i32)>,
) -> Result<bool, LevelLineMeshError> {
    let mut strip: HashSet<i32> = laid[start..].iter().copied().collect();
    let mut changed = false;
    let result = try_clip_and_cap_strip(mesh, &mut strip, kept, &mut changed)?;
    if changed {
        // Whatever was done, the laid list names the strip as it now stands, so a refusal
        // downstream takes back exactly what is there.
        laid.truncate(start);
        let mut ordered: Vec<i32> = strip.iter().copied().collect();
        ordered.sort_unstable();
        laid.extend(ordered);
    }

    Ok(result)
}

/// C# `TryClipAndCap(mesh, strip, kept, out changed)`: the clip on the strip's triangles itself.
/// `changed` is set once any neighbour's clip has begun changing the mesh. On success the virtual
/// edges of `kept` are replaced by the neighbours' crease stretches.
pub fn try_clip_and_cap_strip(
    mesh: &mut LevelLineMesh,
    strip: &mut HashSet<i32>,
    kept: &mut Vec<(i32, i32)>,
    changed: &mut bool,
) -> Result<bool, LevelLineMeshError> {
    *changed = false;
    note(|| "none".to_string());
    let virtual_edges: Vec<(i32, i32)> = {
        let stands_on_kept =
            |a: i32, b: i32| mesh.triangles_on(a, b).iter().any(|t| !strip.contains(t));
        kept.iter()
            .copied()
            .filter(|&(a, b)| !stands_on_kept(a, b))
            .collect()
    };
    if virtual_edges.is_empty() {
        return Ok(true);
    }

    // The growing neighbour's faces: kept, untagged triangles left alone on an edge by the laid
    // strip.
    let open: Vec<(i32, i32)> = mesh
        .edges()
        .into_iter()
        .filter(|&(low, high)| {
            let standing = mesh.triangles_on(low, high);
            standing.len() == 1
                && !strip.contains(&standing[0])
                && mesh.triangles()[standing[0] as usize].tag == 0
        })
        .collect();

    // One NEIGHBOUR per connected run of those edges: the planes of one neighbour clip by their
    // max (its ridge), but two neighbours apart - a fin's two end walls, each growing at its own
    // end (Fillet Thin Front) - are clipped each on its own; read as one, no piece is inside both
    // and nothing is dropped.
    // C#'s HashSet only loses entries here, so it iterates in insertion order: an order-keeping
    // set with order-keeping removal.
    let mut neighbours: Vec<Vec<(i32, i32)>> = Vec::new();
    let mut unseen: IndexSet<(i32, i32)> = open.iter().copied().collect();
    for &seed in &open {
        if !unseen.shift_remove(&seed) {
            continue;
        }

        let mut part = vec![seed];
        let mut i = 0;
        while i < part.len() {
            let (low, high) = part[i];
            let touching: Vec<(i32, i32)> = unseen
                .iter()
                .copied()
                .filter(|o| o.0 == low || o.0 == high || o.1 == low || o.1 == high)
                .collect();
            for other in touching {
                unseen.shift_remove(&other);
                part.push(other);
            }

            i += 1;
        }

        neighbours.push(part);
    }

    // Virtual rail edges with no growing neighbour to clip them on refuse, as one neighbour with
    // no planes does: answering true here dropped the virtual edges from the loop and built a part
    // the refusal's fallback builds differently (Rung7_MixedCornerAtOneVertex(Surface) 2999.623
    // against 2999.645).
    if neighbours.is_empty() {
        note(|| {
            let edges: Vec<String> = virtual_edges
                .iter()
                .map(|&(a, b)| {
                    format!(
                        "{a}{}-{b}{} on {}",
                        mesh.position(a),
                        mesh.position(b),
                        mesh.triangles_on(a, b).len()
                    )
                })
                .collect();
            // LINQ's GroupBy: groups in the order each key first appears.
            let mut by_tag: IndexMap<i32, usize> = IndexMap::new();
            for (low, high) in mesh.edges() {
                let on = mesh.triangles_on(low, high);
                if on.len() == 1 && !strip.contains(&on[0]) {
                    *by_tag
                        .entry(mesh.triangles()[on[0] as usize].tag)
                        .or_insert(0) += 1;
                }
            }
            let tags: Vec<String> = by_tag.iter().map(|(k, n)| format!("{k}:{n}")).collect();
            format!(
                "no growing neighbour for {} virtual edges [{}], lone kept edges by tag [{}]",
                virtual_edges.len(),
                edges.join(" "),
                tags.join(",")
            )
        });
        return Ok(false);
    }

    let mut creases: Vec<(i32, i32)> = Vec::new();
    for part in &neighbours {
        if !clip_on_one_neighbour(mesh, strip, &virtual_edges, part, &mut creases, changed)? {
            return Ok(false);
        }
    }

    kept.retain(|e| !virtual_edges.contains(e));
    kept.extend(creases);
    Ok(true)
}

/// The fixed vertex rule (C#'s local `Distance`): a vertex's distance to a plane is ZERO by
/// bookkeeping - a vertex on the plane's list - and its computed signed distance otherwise.
fn plane_distance(mesh: &LevelLineMesh, planes: &[ClipPlane], plane: i32, vertex: i32) -> f64 {
    let plane = &planes[plane as usize];
    if plane.2.contains(&vertex) {
        0.0
    } else {
        (mesh.position(vertex) - plane.0).dot(plane.1)
    }
}

/// C# `StripClip.ClipOnOneNeighbour`: clips the strip on one growing neighbour (`part`, its
/// connected run of open edges) and caps it - a ridge or, where its walls meet at a concave
/// crease, a valley. The neighbour's crease stretch joins `creases` on success; `changed` is set
/// once the mesh may have changed.
pub fn clip_on_one_neighbour(
    mesh: &mut LevelLineMesh,
    strip: &mut HashSet<i32>,
    virtual_edges: &[(i32, i32)],
    part: &[(i32, i32)],
    creases: &mut Vec<(i32, i32)>,
    changed: &mut bool,
) -> Result<bool, LevelLineMeshError> {
    let mut crease: Vec<(i32, i32)> = Vec::new();
    let mut planes: Vec<ClipPlane> = Vec::new();
    for &(low, high) in part {
        let standing = mesh.triangles_on(low, high);
        let face = mesh.triangles()[standing[0] as usize];
        crease.push(directed(face, low, high));
        let origin = mesh.position(face.a);
        let normal = (mesh.position(face.b) - origin)
            .cross(mesh.position(face.c) - origin)
            .get_normal();
        // One plane of C when the normals agree to 1e-9 and each origin stands on the other's
        // plane to FLOAT NOISE - four float steps at the origins' magnitude, so a rigid motion far
        // from the origin reads the same planes. A relative 1e-9 offset was tighter than float
        // storage: Inner Right Sides and Center's slope (its vertices up to 9.5e-7 off it) read as
        // two planes round its ridge's tip and the cap's outline did not close
        // (RoundsFirstTests.TheRidgeAloneRunsOutThroughBothEndPlanes). The facing stays at 1e-9
        // (0.0026 degrees): a finely tessellated neighbour - a round of many facets - turns more
        // than that between facets, so distinct facets stay distinct planes.
        let same = planes.iter().position(|p: &ClipPlane| {
            p.1.dot(normal) >= 1.0 - 1e-9
                && (origin - p.0).dot(p.1).abs()
                    <= 4.0
                        * cs_math::max_f64(
                            float_grid::step_at_point(origin),
                            float_grid::step_at_point(p.0),
                        )
        });
        match same {
            None => planes.push((origin, normal, HashSet::from([face.a, face.b, face.c]))),
            Some(at) => planes[at].2.extend([face.a, face.b, face.c]),
        }
    }

    if planes.is_empty() {
        note(|| "no planes".to_string());
        return Ok(false);
    }

    *changed = true;

    // The sense of the crease the growing walls share decides the clip: a RIDGE (the solid behind
    // every wall) drops what is behind all of them, as every clip before did; a VALLEY (the solid
    // behind either wall - a swallowed fin's V, see SwallowedSide) drops what is behind any of
    // them.
    let valley = valley_planes(mesh, part);
    let is_valley = valley.is_some();
    if let Some(walls) = valley {
        planes = walls;
    }

    let distance = &plane_distance;
    let mut attached = attached(mesh, strip, virtual_edges, &planes, distance, is_valley)?;
    if is_valley {
        // A valley's strip is laid on made-up runs wholly inside the walls' material: all of it is
        // the neighbour's to clip, and a piece behind either wall is dropped wherever it lies.
        attached.extend(strip.iter().copied());
    }
    let crease_vertices: HashSet<i32> = crease.iter().flat_map(|&(a, b)| [a, b]).collect();
    let tied = if is_valley {
        insert_valley_ties(
            mesh,
            strip,
            &mut attached,
            &crease_vertices,
            &mut planes,
            distance,
        )?
    } else {
        insert_ties(
            mesh,
            strip,
            &mut attached,
            &crease_vertices,
            &mut planes,
            distance,
        )?
    };
    if !tied {
        return Ok(false);
    }

    // To a fixed point: a split along one face makes pieces whose edges can newly cross another
    // face INSIDE the neighbour (the parent's crossing lay outside it), so every face is split
    // again until a whole pass splits nothing. Each split resolves one crossing, so the passes
    // end.
    let mut splits = 1;
    while splits > 0 {
        grow(mesh, strip, &mut attached, &planes, distance, is_valley)?;
        splits = 0;
        let mut f = 0;
        while f < planes.len() && splits >= 0 {
            let made = split_along(
                mesh,
                strip,
                &mut attached,
                f,
                &mut planes,
                distance,
                is_valley,
            )?;
            splits = if made < 0 { -1 } else { splits + made };
            f += 1;
        }

        if splits < 0 {
            let count = planes.len();
            note(|| {
                format!(
                    "split on {count} planes: {}",
                    last_outcome().unwrap_or_default()
                )
            });
            return Ok(false);
        }
    }

    // Drop the strip inside the neighbour: each plane's distance is linear, so a piece's centroid
    // reads the mean of its corners', and a piece lies wholly on one side of every plane after the
    // splits. The drop is the CONNECTED part of the inside pieces that the attached part reaches:
    // flooded across shared edges from the attached inside pieces (each piece lies on one side of
    // every face it was split along). A piece with every corner on a clip curve or inside - none
    // strictly inside every face - was never attached, and leaving it would leave a kept piece
    // inside the lens with an outline edge running from one clip curve to the other across it.
    // Inside: no corner outside any face, and not every corner on one. A piece can still straddle
    // a face where that face's crossing is outside the neighbour (another face positive there, so
    // it was never split): it is outside there and is kept, never read by its centroid.
    let drop = {
        let mesh_ref: &LevelLineMesh = mesh;
        let planes_ref: &[ClipPlane] = &planes;
        let inside_piece = |t: i32| {
            let face = mesh_ref.triangles()[t as usize];
            let d = |f: usize, v: i32| distance(mesh_ref, planes_ref, f as i32, v);
            let behind = |f: usize| {
                d(f, face.a) <= 0.0
                    && d(f, face.b) <= 0.0
                    && d(f, face.c) <= 0.0
                    && d(f, face.a) + d(f, face.b) + d(f, face.c) < 0.0
            };
            if is_valley {
                (0..planes_ref.len()).any(behind)
            } else {
                (0..planes_ref.len()).all(behind)
            }
        };

        let mut drop: HashSet<i32> = attached
            .iter()
            .copied()
            .filter(|&t| strip.contains(&t) && inside_piece(t))
            .collect();
        let mut seeded: Vec<i32> = drop.iter().copied().collect();
        seeded.sort_unstable();
        let mut reach: VecDeque<i32> = seeded.into();
        while let Some(next) = reach.pop_front() {
            LevelLineWork::charge(1)?;
            let face = mesh_ref.triangles()[next as usize];
            for (p, q) in triangle_edges::corners((face.a, face.b, face.c)) {
                for &other in mesh_ref.triangles_on(p, q) {
                    if strip.contains(&other) && !drop.contains(&other) && inside_piece(other) {
                        drop.insert(other);
                        reach.push_back(other);
                    }
                }
            }
        }

        let mut drop: Vec<i32> = drop.into_iter().collect();
        drop.sort_unstable();
        drop
    };

    let mut dropped: Vec<LevelLineTriangle> = Vec::new();
    for t in drop {
        dropped.push(mesh.triangles()[t as usize]);
        mesh.remove_triangle(t);
        strip.remove(&t);
    }

    let crease_set: IndexSet<(i32, i32)> = crease.iter().copied().collect();
    let capped = if is_valley {
        valley_cap(mesh, strip, &planes, &dropped, &crease_set)?
    } else {
        cap(mesh, strip, &planes, &dropped, &crease_set)?
    };
    if !capped {
        let count = planes.len();
        note(|| {
            format!(
                "cap, {count} planes: {}",
                last_outcome().unwrap_or_default()
            )
        });
        return Ok(false);
    }

    let attached_count = attached.len();
    note(|| format!("done, {} planes, attached {attached_count}", planes.len()));

    creases.extend(crease);
    Ok(true)
}
