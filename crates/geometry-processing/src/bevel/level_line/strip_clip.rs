//! Flow rung 2: the strip clipped on a GROWING neighbour's planes, and the neighbour's caps. Rows
//! laid whole across a virtual rail overlap the neighbour that stands past the reached crease;
//! each of its faces on that crease clips the strip, the strip where EVERY face's distance is below
//! zero is dropped, and the neighbour grows a cap in each face's plane. Here: the
//! directed edge, the opposite corner, the cap snap and the last outcome note ([`note`],
//! [`last_outcome`]), and the clip's mesh work: the ties ([`insert_ties`]), the split along a plane
//! ([`split_along`], [`crosses_at_a_needles_apex`]), the cap ([`cap`]), and the part the neighbour
//! grows into ([`attached`], [`grow`]). The clip's driver (`TryClipAndCap`, `ClipOnOneNeighbour`) is
//! `strip_clip_neighbour`; its valley mode is `strip_clip_valley`.

use std::cell::RefCell;
use std::collections::{HashSet, VecDeque};

use indexmap::{IndexMap, IndexSet};
use solidmesh_math::Vector3;

use super::super::float_grid;
use super::strip_clip_valley::other_says;
use super::{flat_hole_fill, triangle_edges};
use super::{LevelLineMesh, LevelLineMeshError, LevelLineTriangle, LevelLineWork, WorkError};

/// One of the neighbour's face planes the strip is clipped on: its origin, its normal and the
/// vertices that are on it by bookkeeping.
pub type ClipPlane = (Vector3, Vector3, HashSet<i32>);

/// The signed distance of a vertex from a plane, by index: handed the mesh and planes it reads,
/// since the clip changes both.
pub type ClipDistance<'a> = &'a dyn Fn(&LevelLineMesh, &[ClipPlane], i32, i32) -> f64;

/// A watcher of the strip clip's caps: the drop's outline as directed edges.
pub type WatchCapHole = std::rc::Rc<dyn Fn(&[(i32, i32)])>;

/// The snap a cap's outline is read flat by: the build's own (BandGap's
/// 1e-9), so a cap whose outline is off its plane by more keeps the fan it always had.
pub const CAP_SNAP: f64 = 1e-9;

thread_local! {
    /// What the last clip on this thread did ([`last_outcome`]).
    static LAST_OUTCOME: RefCell<Option<String>> = const { RefCell::new(None) };

    /// A read-only observer of the strip clip's caps.
    /// Per thread; `None` (the default) costs nothing.
    pub static WATCH_CAP_HOLE: RefCell<Option<WatchCapHole>> = const { RefCell::new(None) };
}

/// What the last clip on this thread did, for the sweep's no-result
/// rows (`None` before any clip).
pub fn last_outcome() -> Option<String> {
    LAST_OUTCOME.with(|o| o.borrow().clone())
}

/// Sets (or, with `None`, clears) this thread's outcome, as a
/// caller does before a fill whose clip it means to read.
pub fn set_last_outcome(outcome: Option<String>) {
    LAST_OUTCOME.with(|o| *o.borrow_mut() = outcome);
}

/// Records the clip's outcome. Written always, not only under the meter: the
/// sweep classifies no-result rows by the refusing site, and a clip runs a handful of times per
/// fill, so the string costs nothing beside the mesh work.
pub fn note(outcome: impl FnOnce() -> String) {
    let text = outcome();
    LAST_OUTCOME.with(|o| *o.borrow_mut() = Some(text));
}

/// The corner of `face` that is neither `a` nor `b`.
pub fn opposite_corner(face: LevelLineTriangle, a: i32, b: i32) -> i32 {
    if face.a != a && face.a != b {
        face.a
    } else if face.b != a && face.b != b {
        face.b
    } else {
        face.c
    }
}

/// The edge `low - high` in the direction `face` traverses it.
pub fn directed(face: LevelLineTriangle, low: i32, high: i32) -> (i32, i32) {
    for corner in 0..3 {
        if face.corner(corner) == low && face.corner((corner + 1) % 3) == high {
            return (low, high);
        }
    }

    (high, low)
}

/// The strip's triangles in index order, so the set's own order never reaches the result.
fn ordered(strip: &HashSet<i32>) -> Vec<i32> {
    let mut all: Vec<i32> = strip.iter().copied().collect();
    all.sort_unstable();
    all
}

/// Mints the TIE where two faces' zeros meet inside a strip triangle -
/// the neighbour's ridge piercing the strip - so both clip curves end on one shared vertex. Solved
/// in the triangle's barycentric frame, where both distances are linear. False where the tie would
/// split an edge the kept surface stands on.
pub fn insert_ties(
    mesh: &mut LevelLineMesh,
    strip: &mut HashSet<i32>,
    attached: &mut HashSet<i32>,
    crease_vertices: &HashSet<i32>,
    planes: &mut [ClipPlane],
    distance: ClipDistance,
) -> Result<bool, LevelLineMeshError> {
    const ON_EDGE: f64 = 1e-9;
    for f in 0..planes.len() {
        for h in f + 1..planes.len() {
            // The crossing taken is the one nearest the ridge's own start on the reached crease.
            let starts: Vec<Vector3> = planes[f]
                .2
                .iter()
                .filter(|v| planes[h].2.contains(v) && crease_vertices.contains(v))
                .map(|&v| mesh.position(v))
                .collect();
            if starts.is_empty() {
                continue;
            }

            let (fi, hi) = (f as i32, h as i32);
            let mut best: Option<(i32, f64, f64, f64, f64)> = None;
            for t in ordered(strip) {
                let face = mesh.triangles()[t as usize];
                let d = |plane: i32, v: i32| distance(mesh, planes, plane, v);
                let (fa, fb, fc) = (d(fi, face.a), d(fi, face.b), d(fi, face.c));
                let (ha, hb, hc) = (d(hi, face.a), d(hi, face.b), d(hi, face.c));

                // u (b) + v (c) + (1 - u - v) (a): both distances zero.
                let (p1, q1, r1) = (fb - fa, fc - fa, -fa);
                let (p2, q2, r2) = (hb - ha, hc - ha, -ha);
                let det = (p1 * q2) - (q1 * p2);
                if det == 0.0 {
                    continue;
                }

                let u0 = ((r1 * q2) - (q1 * r2)) / det;
                let v0 = ((p1 * r2) - (r1 * p2)) / det;
                let w0 = 1.0 - u0 - v0;
                if u0 < -ON_EDGE || v0 < -ON_EDGE || w0 < -ON_EDGE {
                    continue;
                }

                let a0 = mesh.position(face.a);
                let point =
                    a0 + ((mesh.position(face.b) - a0) * u0) + ((mesh.position(face.c) - a0) * v0);
                // The least reach, a NaN reading as the smallest.
                let mut reach = f64::INFINITY;
                for (k, &s0) in starts.iter().enumerate() {
                    let x = (s0 - point).length();
                    if k == 0 || x < reach || x.is_nan() {
                        reach = x;
                    }
                }
                if best.is_none_or(|b| reach < b.4) {
                    best = Some((t, u0, v0, w0, reach));
                }
            }

            let Some((t, u, v, w, _)) = best else {
                continue;
            };

            let face = mesh.triangles()[t as usize];
            // Within ON_EDGE of a side the tie is minted on that edge; at a corner it is that corner.
            if (u <= ON_EDGE && v <= ON_EDGE)
                || (v <= ON_EDGE && w <= ON_EDGE)
                || (u <= ON_EDGE && w <= ON_EDGE)
            {
                let corner = if u > ON_EDGE {
                    face.b
                } else if v > ON_EDGE {
                    face.c
                } else {
                    face.a
                };
                planes[f].2.insert(corner);
                planes[h].2.insert(corner);
                continue;
            }

            let a = mesh.position(face.a);
            let at = a + ((mesh.position(face.b) - a) * u) + ((mesh.position(face.c) - a) * v);
            // An edge the kept surface stands on is never split here.
            let (ea, eb) = if u <= ON_EDGE {
                (face.c, face.a)
            } else if v <= ON_EDGE {
                (face.a, face.b)
            } else {
                (face.b, face.c)
            };
            if (u <= ON_EDGE || v <= ON_EDGE || w <= ON_EDGE)
                && mesh.triangles_on(ea, eb).iter().any(|o| !strip.contains(o))
            {
                note(|| format!("tie on kept edge {ea}-{eb}"));
                return Ok(false);
            }

            let before = mesh.triangle_count();
            // Exact only: a clip needs a real vertex ON its plane, not a neighbour a float step off it.
            let tie = if u <= ON_EDGE {
                mesh.split_edge(face.c, face.a, at, false)?
            } else if v <= ON_EDGE {
                mesh.split_edge(face.a, face.b, at, false)?
            } else if w <= ON_EDGE {
                mesh.split_edge(face.b, face.c, at, false)?
            } else {
                mesh.split_triangle(t, at)?
            };
            for child in before..mesh.triangle_count() {
                strip.insert(child);
                attached.insert(child);
            }

            planes[f].2.insert(tie);
            planes[h].2.insert(tie);
        }
    }

    Ok(true)
}

/// Splits the strip along plane `f`'s zero where it bounds the
/// neighbour - where every OTHER face's distance says so too (a valley splits at every crossing) -
/// one crossed edge at a time, until no such crossing is left. The count made, or -1 where a
/// crossing is on an edge the kept surface stands on or the mesh refused the split.
pub fn split_along(
    mesh: &mut LevelLineMesh,
    strip: &mut HashSet<i32>,
    attached: &mut HashSet<i32>,
    f: usize,
    planes: &mut [ClipPlane],
    distance: ClipDistance,
    valley: bool,
) -> Result<i32, LevelLineMeshError> {
    let fi = f as i32;
    let mut made = 0;
    loop {
        let mut found: Option<(i32, i32, Vector3)> = None;
        let mut candidates: Vec<i32> = attached
            .iter()
            .copied()
            .filter(|t| strip.contains(t))
            .collect();
        candidates.sort_unstable();
        for t in candidates {
            let face = mesh.triangles()[t as usize];
            let mut corner = 0;
            while corner < 3 && found.is_none() {
                let (a, b) = (face.corner(corner), face.corner((corner + 1) % 3));
                corner += 1;
                let (da, db) = (distance(mesh, planes, fi, a), distance(mesh, planes, fi, b));
                if !((da < 0.0 && db > 0.0) || (da > 0.0 && db < 0.0)) {
                    continue;
                }

                let s = da / (da - db);
                let bounds = valley
                    || (0..planes.len()).filter(|&h| h != f).all(|h| {
                        let h = h as i32;
                        let at_a = distance(mesh, planes, h, a);
                        other_says(at_a + ((distance(mesh, planes, h, b) - at_a) * s), valley)
                    });
                if bounds {
                    found = Some((
                        a,
                        b,
                        mesh.position(a) + ((mesh.position(b) - mesh.position(a)) * s),
                    ));
                }
            }

            if found.is_some() {
                break;
            }
        }

        let Some((ea, eb, point)) = found else {
            return Ok(made);
        };

        made += 1;

        if mesh.triangles_on(ea, eb).iter().any(|t| !strip.contains(t)) {
            note(|| format!("kept edge {ea}-{eb} at {point}"));
            return Ok(-1);
        }

        if crosses_at_a_needles_apex(mesh, strip, attached, planes, f, distance, ea, eb, point)? {
            continue;
        }

        let before = mesh.triangle_count();
        let vertices = mesh.vertex_count();
        // Exact only: a clip needs a real vertex ON its plane, not a neighbour a float step off it.
        let vertex = mesh.split_edge(ea, eb, point, false)?;
        if vertex < vertices && vertex != ea && vertex != eb {
            // The mesh refused the split for a corner already standing at the crossing.
            note(|| format!("refused split {ea}-{eb} at {point} -> {vertex}"));
            return Ok(-1);
        }

        for child in before..mesh.triangle_count() {
            strip.insert(child);
            attached.insert(child);
        }

        for plane in planes.iter_mut() {
            if plane.2.contains(&ea) && plane.2.contains(&eb) {
                plane.2.insert(vertex);
            }
        }

        planes[f].2.insert(vertex);
    }
}

/// Where the edge's crossing of plane `f` stands within a
/// float step of the far corner of a strip triangle on that edge, and that corner is already ON the
/// plane, the edge is split at that corner instead - the needle goes and the triangle across the
/// edge becomes two on the corner. False where no such corner stands, or the edge is not two strip
/// triangles.
#[allow(clippy::too_many_arguments)]
pub fn crosses_at_a_needles_apex(
    mesh: &mut LevelLineMesh,
    strip: &mut HashSet<i32>,
    attached: &mut HashSet<i32>,
    planes: &mut [ClipPlane],
    f: usize,
    distance: ClipDistance,
    ea: i32,
    eb: i32,
    point: Vector3,
) -> Result<bool, LevelLineMeshError> {
    let standing: Vec<i32> = mesh.triangles_on(ea, eb).to_vec();
    if standing.len() != 2 || !standing.iter().all(|t| strip.contains(t)) {
        return Ok(false);
    }

    let step = float_grid::step_at_point(point);
    for &needle in &standing {
        let apex = opposite_corner(mesh.triangles()[needle as usize], ea, eb);
        if (mesh.position(apex) - point).length() > step
            || distance(mesh, planes, f as i32, apex).abs() > step
        {
            continue;
        }

        let across = if standing[0] == needle {
            standing[1]
        } else {
            standing[0]
        };
        let across_face = mesh.triangles()[across as usize];
        let (u, v) = directed(across_face, ea.min(eb), ea.max(eb));
        let far = opposite_corner(across_face, ea, eb);
        if far == apex {
            return Ok(false);
        }

        mesh.remove_triangle(needle);
        mesh.remove_triangle(across);
        strip.remove(&needle);
        strip.remove(&across);
        let first = mesh.add_face(u, apex, far, 0)?;
        let second = mesh.add_face(apex, v, far, 0)?;
        for child in [first, second] {
            strip.insert(child);
            attached.insert(child);
        }

        for plane in planes.iter_mut() {
            if plane.2.contains(&ea) && plane.2.contains(&eb) {
                plane.2.insert(apex);
            }
        }

        planes[f].2.insert(apex);
        return Ok(true);
    }

    Ok(false)
}

/// A number in the format `0.###e0`: up to three decimals and an exponent (the outcome notes'
/// distances).
pub(super) fn format_e0(value: f64) -> String {
    solidmesh_math::number_format::format_f64_scientific_e0(value, 3)
}

/// Closes the hole the drop left. Its outline is the crease stretch plus the
/// clip curve, split at the vertices on two planes into one polygon per plane, each closed flat
/// where it lies flat (else fanned from its first vertex) in its plane; the faces laid join
/// `strip`.
pub fn cap(
    mesh: &mut LevelLineMesh,
    strip: &mut HashSet<i32>,
    planes: &[ClipPlane],
    dropped: &[LevelLineTriangle],
    crease: &IndexSet<(i32, i32)>,
) -> Result<bool, LevelLineMeshError> {
    // The outline is the DROP's: every edge of a dropped piece now standing on one triangle, and
    // the crease stretch the neighbour stands on alone.
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
        if !planes
            .iter()
            .any(|p| p.2.contains(&low) && p.2.contains(&high))
        {
            note(|| {
                let reading = |p: &ClipPlane, v: i32| (mesh.position(v) - p.0).dot(p.1);
                let on_planes: Vec<String> = planes
                    .iter()
                    .map(|p| {
                        format!(
                            "{}{}:{},{}",
                            title_case_bool(p.2.contains(&low)),
                            title_case_bool(p.2.contains(&high)),
                            format_e0(reading(p, low)),
                            format_e0(reading(p, high))
                        )
                    })
                    .collect();
                let face = mesh.triangles()[standing0 as usize];
                let corners: Vec<String> = [face.a, face.b, face.c]
                    .iter()
                    .map(|&v| {
                        let marks: Vec<String> = planes
                            .iter()
                            .map(|p| {
                                if p.2.contains(&v) {
                                    "on".to_string()
                                } else {
                                    format_e0(reading(p, v))
                                }
                            })
                            .collect();
                        format!("{v}{}[{}]", mesh.position(v), marks.join(","))
                    })
                    .collect();
                format!(
                    "outline edge {low}-{high} ({} {}, on planes {}, standing on {} {standing0} corners {}) is neither crease stretch nor clip curve",
                    mesh.position(low),
                    mesh.position(high),
                    on_planes.join("/"),
                    if strip.contains(&standing0) { "strip" } else { "kept" },
                    corners.join(" ")
                )
            });
            return Ok(false);
        }

        let (a, b) = directed(mesh.triangles()[standing0 as usize], low, high);
        if next.contains_key(&b) {
            note(|| format!("outline branches at {b}"));
            return Ok(false);
        }

        next.insert(b, a);
    }

    let watch = WATCH_CAP_HOLE.with(|w| w.borrow().clone());
    if let Some(watch) = watch {
        let edges: Vec<(i32, i32)> = next.iter().map(|(&k, &v)| (k, v)).collect();
        watch(&edges);
    }

    while !next.is_empty() {
        let mut loop_ = Vec::new();
        let mut at = *next.keys().min().expect("not empty");
        while let Some(to) = next.shift_remove(&at) {
            loop_.push(at);
            at = to;
        }

        if at != loop_[0] {
            note(|| format!("outline does not close at {at}"));
            return Ok(false);
        }

        let planes_of = |v: i32| planes.iter().filter(|p| p.2.contains(&v)).count();
        let mut corners: Vec<usize> = (0..loop_.len())
            .filter(|&i| planes_of(loop_[i]) >= 2)
            .collect();
        if corners.len() < 2 {
            corners = vec![0];
        }

        for c in 0..corners.len() {
            let from = corners[c];
            let to = corners[(c + 1) % corners.len()];
            let mut polygon = Vec::new();
            let mut i = from;
            loop {
                polygon.push(loop_[i]);
                if polygon.len() > 1 && i == to {
                    break;
                }
                i = (i + 1) % loop_.len();
            }

            if corners.len() == 1 {
                polygon.pop();
            }

            // Every cap lies in ONE of the neighbour's planes, by bookkeeping.
            if !planes
                .iter()
                .any(|p| polygon.iter().all(|v| p.2.contains(v)))
            {
                note(|| "a cap outline is not in one of the neighbour's planes".to_string());
                return Ok(false);
            }

            // Ear-clipped where the outline closes flat, not fanned from its first vertex.
            let points: Vec<Vector3> = polygon.iter().map(|&v| mesh.position(v)).collect();
            if let Some((flat, _)) = flat_hole_fill::fill(&points, CAP_SNAP, None) {
                for t in flat {
                    strip.insert(mesh.add_face(
                        polygon[t.a as usize],
                        polygon[t.b as usize],
                        polygon[t.c as usize],
                        0,
                    )?);
                }

                continue;
            }

            for i in 1..polygon.len().saturating_sub(1) {
                strip.insert(mesh.add_face(polygon[0], polygon[i], polygon[i + 1], 0)?);
            }
        }
    }

    Ok(true)
}

/// A boolean as the outcome notes write it: `True` or `False`.
fn title_case_bool(value: bool) -> &'static str {
    if value {
        "True"
    } else {
        "False"
    }
}

/// Whether input vertex `v` stands inside the neighbour: below zero on ANY face for a valley, on
/// EVERY face otherwise.
fn inside(
    mesh: &LevelLineMesh,
    planes: &[ClipPlane],
    distance: ClipDistance,
    valley: bool,
    v: i32,
) -> bool {
    if valley {
        (0..planes.len()).any(|f| distance(mesh, planes, f as i32, v) < 0.0)
    } else {
        (0..planes.len()).all(|f| distance(mesh, planes, f as i32, v) < 0.0)
    }
}

/// The strip triangles the neighbour grows into FROM THE REACHED CREASE - flooded
/// from the rows that start on the virtual stretch (`virtual_edges`), across shared edges, through
/// triangles with a corner inside the neighbour (`distance(face, vertex)` against each of `planes`).
/// A second part inside the faces, where the neighbour's ridge leaves the round again further on, is
/// not attached: the control keeps the round whole there.
pub fn attached(
    mesh: &LevelLineMesh,
    strip: &HashSet<i32>,
    virtual_edges: &[(i32, i32)],
    planes: &[ClipPlane],
    distance: ClipDistance,
    valley: bool,
) -> Result<HashSet<i32>, WorkError> {
    let touches = |face: LevelLineTriangle| {
        let inside = |v| inside(mesh, planes, distance, valley, v);
        inside(face.a) || inside(face.b) || inside(face.c)
    };

    let seeds: HashSet<i32> = virtual_edges.iter().flat_map(|&(a, b)| [a, b]).collect();
    let mut attached: HashSet<i32> = ordered(strip)
        .into_iter()
        .filter(|&t| {
            let face = mesh.triangles[t as usize];
            touches(face)
                && (seeds.contains(&face.a) || seeds.contains(&face.b) || seeds.contains(&face.c))
        })
        .collect();

    // The flood is seeded ascending, so the set's own order never
    // reaches it.
    let mut seeded: Vec<i32> = attached.iter().copied().collect();
    seeded.sort_unstable();
    let mut work: VecDeque<i32> = seeded.into();

    while let Some(next) = work.pop_front() {
        LevelLineWork::charge(1)?;
        let face = mesh.triangles[next as usize];

        for (p, q) in triangle_edges::corners((face.a, face.b, face.c)) {
            for &other in mesh.triangles_on(p, q) {
                if strip.contains(&other)
                    && !attached.contains(&other)
                    && touches(mesh.triangles[other as usize])
                {
                    attached.insert(other);
                    work.push_back(other);
                }
            }
        }
    }

    Ok(attached)
}

/// Grows the attached part across shared edges into every strip triangle the
/// neighbour reaches into - a corner strictly inside every face, or an edge crossing a face where
/// every other face is below zero (a piece whose corners all sit on the clip curves or outside,
/// with the lens running through its edge).
pub fn grow(
    mesh: &LevelLineMesh,
    strip: &HashSet<i32>,
    attached: &mut HashSet<i32>,
    planes: &[ClipPlane],
    distance: ClipDistance,
    valley: bool,
) -> Result<(), WorkError> {
    let count = planes.len();
    let d = |f: usize, v: i32| distance(mesh, planes, f as i32, v);
    let bounding = |a: i32, b: i32| {
        (0..count).any(|f| {
            let (da, db) = (d(f, a), d(f, b));

            if !((da < 0.0 && db > 0.0) || (da > 0.0 && db < 0.0)) {
                return false;
            }

            let s = da / (da - db);

            (0..count)
                .filter(|&h| h != f)
                .all(|h| other_says(d(h, a) + ((d(h, b) - d(h, a)) * s), valley))
        })
    };
    let reached = |face: LevelLineTriangle| {
        let inside = |v| inside(mesh, planes, distance, valley, v);
        inside(face.a)
            || inside(face.b)
            || inside(face.c)
            || bounding(face.a, face.b)
            || bounding(face.b, face.c)
            || bounding(face.c, face.a)
    };

    let mut seeded: Vec<i32> = attached
        .iter()
        .copied()
        .filter(|t| strip.contains(t))
        .collect();
    seeded.sort_unstable();
    let mut work: VecDeque<i32> = seeded.into();

    while let Some(next) = work.pop_front() {
        LevelLineWork::charge(1)?;
        let face = mesh.triangles[next as usize];

        for (p, q) in triangle_edges::corners((face.a, face.b, face.c)) {
            for &other in mesh.triangles_on(p, q) {
                if strip.contains(&other)
                    && !attached.contains(&other)
                    && reached(mesh.triangles[other as usize])
                {
                    attached.insert(other);
                    work.push_back(other);
                }
            }
        }
    }

    Ok(())
}
