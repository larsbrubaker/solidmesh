//! The HULL mode of the strip clip: where a concave bead's rows are run on past the part itself, no
//! neighbour face stands there to grow, so the part's own SUPPORTING planes bound it instead - every
//! plane of a kept face with the whole kept surface on its inner side - and every piece of the
//! strip OUTSIDE any of them is dropped. One supporting plane ([`HullClipPlane`]), the kept
//! surface's supporting
//! planes the strip crosses ([`crossed_supporting_planes`]), the face-directed edge ([`directed`])
//! and the thread's last outcome ([`note`], [`last_outcome`], [`set_last_outcome`]), the clip back
//! to the hull ([`clip`], splitting along each plane with [`split_along`]) and the hull end of a fill
//! ([`try_clip_and_cap`], capping each plane's hole with [`cap`] and [`ear_clip`]).

use std::cell::RefCell;
use std::collections::HashSet;

use indexmap::{IndexMap, IndexSet};
use vector_math::{ieee_math, Vector2, Vector3};

use super::{FieldCounters, LevelLineMesh, LevelLineMeshError, LevelLineTriangle};

thread_local! {
    /// Where the last hull end on this thread stopped - "done" or the stage that refused - for a
    /// test to name, per thread. Written only while the meter is on.
    static LAST_OUTCOME: RefCell<Option<String>> = const { RefCell::new(None) };

    /// A read-only observer of each plane's cap
    /// outline (its directed edges) before it is closed. Per thread; `None` by default.
    pub static WATCH_CAP_HOLE: RefCell<Option<WatchCapHole>> = const { RefCell::new(None) };
}

/// A watcher of each plane's cap outline, as directed edges.
pub type WatchCapHole = std::rc::Rc<dyn Fn(&[(i32, i32)])>;

/// Records the hull end's outcome, made only when the meter is on.
pub fn note(outcome: impl FnOnce() -> String) {
    if FieldCounters::meter_on() {
        let said = outcome();
        LAST_OUTCOME.with(|last| *last.borrow_mut() = Some(said));
    }
}

/// Where the last hull end on this thread stopped; `None` before any was noted.
pub fn last_outcome() -> Option<String> {
    LAST_OUTCOME.with(|last| last.borrow().clone())
}

/// Sets (or, with `None`, clears) this thread's outcome, as a
/// test does before a fill whose hull end it means to read.
pub fn set_last_outcome(outcome: Option<String>) {
    LAST_OUTCOME.with(|last| *last.borrow_mut() = outcome);
}

/// A supporting plane - a point on it, its outward normal, and the vertices
/// on it by bookkeeping. A vertex of the kept surface in the plane, or one minted on an edge
/// crossing it, is ON the plane by bookkeeping; every other vertex reads its computed distance.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct HullClipPlane {
    /// A point on the plane.
    pub origin: Vector3,
    /// The plane's outward unit normal.
    pub normal: Vector3,
    /// The vertices ON the plane by bookkeeping, read as distance zero.
    pub on: HashSet<i32>,
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

/// The supporting planes of the kept surface (live triangles not in `strip`) that some strip vertex
/// stands strictly outside of. The tolerance is relative to the kept surface's extent, so the same
/// planes are read at any scale or placement.
pub fn crossed_supporting_planes(mesh: &LevelLineMesh, strip: &HashSet<i32>) -> Vec<HullClipPlane> {
    let kept_triangles: Vec<i32> = (0..mesh.triangle_count())
        .filter(|&t| !mesh.is_removed(t) && !strip.contains(&t))
        .collect();
    let corners_of = |t: i32| {
        let face = mesh.triangles()[t as usize];
        [face.a, face.b, face.c]
    };
    // First-appearance order.
    let kept_vertices: Vec<i32> = kept_triangles
        .iter()
        .flat_map(|&t| corners_of(t))
        .collect::<IndexSet<i32>>()
        .into_iter()
        .collect();
    if kept_vertices.is_empty() {
        return Vec::new();
    }

    let first = mesh.position(kept_vertices[0]);
    let (mut low, mut high) = (first, first);
    for &v in &kept_vertices[1..] {
        let p = mesh.position(v);
        low = Vector3::new(
            ieee_math::min_f64(low.x, p.x),
            ieee_math::min_f64(low.y, p.y),
            ieee_math::min_f64(low.z, p.z),
        );
        high = Vector3::new(
            ieee_math::max_f64(high.x, p.x),
            ieee_math::max_f64(high.y, p.y),
            ieee_math::max_f64(high.z, p.z),
        );
    }

    let tolerance = 1e-9 * ieee_math::max_f64(1.0, (high - low).length());
    let strip_vertices: Vec<i32> = strip
        .iter()
        .flat_map(|&t| corners_of(t))
        .collect::<IndexSet<i32>>()
        .into_iter()
        .collect();

    let mut planes: Vec<HullClipPlane> = Vec::new();
    let mut rejected: Vec<(Vector3, Vector3)> = Vec::new();
    let same = |o: Vector3, n: Vector3, origin: Vector3, normal: Vector3| {
        n.dot(normal) >= 1.0 - 1e-9 && (origin - o).dot(n).abs() <= tolerance
    };
    for &t in &kept_triangles {
        let face = mesh.triangles()[t as usize];
        let origin = mesh.position(face.a);
        let cross = (mesh.position(face.b) - origin).cross(mesh.position(face.c) - origin);
        if cross.length() == 0.0 {
            continue;
        }

        let normal = cross.get_normal();
        if let Some(found) = planes
            .iter_mut()
            .find(|p| same(p.origin, p.normal, origin, normal))
        {
            found.on.extend([face.a, face.b, face.c]);
            continue;
        }

        if rejected.iter().any(|r| same(r.0, r.1, origin, normal)) {
            continue;
        }

        if kept_vertices
            .iter()
            .any(|&v| (mesh.position(v) - origin).dot(normal) > tolerance)
            || !strip_vertices
                .iter()
                .any(|&v| (mesh.position(v) - origin).dot(normal) > tolerance)
        {
            rejected.push((origin, normal));
            continue;
        }

        planes.push(HullClipPlane {
            origin,
            normal,
            on: HashSet::from([face.a, face.b, face.c]),
        });
    }

    // Every vertex of the kept surface lying in a plane is on it by bookkeeping, whichever face
    // named it.
    for plane in &mut planes {
        let on_plane = kept_vertices
            .iter()
            .copied()
            .filter(|&v| (mesh.position(v) - plane.origin).dot(plane.normal).abs() <= tolerance);
        plane.on.extend(on_plane);
    }

    planes
}

/// A vertex ON plane `plane` by bookkeeping reads zero, every other
/// its computed signed distance.
fn plane_distance(mesh: &LevelLineMesh, planes: &[HullClipPlane], plane: i32, vertex: i32) -> f64 {
    let plane = &planes[plane as usize];
    if plane.on.contains(&vertex) {
        0.0
    } else {
        (mesh.position(vertex) - plane.origin).dot(plane.normal)
    }
}

/// Splits `strip` along each plane where it bounds the hull and drops every
/// piece outside any plane. False, with the strip as far as it got, where a crossing lies on an
/// edge the kept surface stands on or a piece is left straddling a plane.
pub fn clip(
    mesh: &mut LevelLineMesh,
    strip: &mut HashSet<i32>,
    planes: &mut [HullClipPlane],
) -> Result<bool, LevelLineMeshError> {
    let distance = plane_distance;

    // The TIES first, as StripClip's: where two planes' zeros meet inside a strip triangle (the
    // part's edge between them piercing the strip), a vertex on both, so both clip curves end on
    // it. Without it the piece round the pierce straddles both planes with each crossing outside
    // the other plane, and no split is made there.
    for f in 0..planes.len() {
        for h in (f + 1)..planes.len() {
            let mut ordered: Vec<i32> = strip.iter().copied().collect();
            ordered.sort_unstable();
            for t in ordered {
                let face = mesh.triangles()[t as usize];
                let (fi, hi) = (f as i32, h as i32);
                let (fa, fb, fc) = (
                    distance(mesh, planes, fi, face.a),
                    distance(mesh, planes, fi, face.b),
                    distance(mesh, planes, fi, face.c),
                );
                let (ha, hb, hc) = (
                    distance(mesh, planes, hi, face.a),
                    distance(mesh, planes, hi, face.b),
                    distance(mesh, planes, hi, face.c),
                );
                let (p1, q1, r1) = (fb - fa, fc - fa, -fa);
                let (p2, q2, r2) = (hb - ha, hc - ha, -ha);
                let det = (p1 * q2) - (q1 * p2);
                if det == 0.0 {
                    continue;
                }

                let u = ((r1 * q2) - (q1 * r2)) / det;
                let v = ((p1 * r2) - (r1 * p2)) / det;
                const INTERIOR: f64 = 1e-9;
                if u <= INTERIOR || v <= INTERIOR || 1.0 - u - v <= INTERIOR {
                    continue;
                }

                let a = mesh.position(face.a);
                let at = a + ((mesh.position(face.b) - a) * u) + ((mesh.position(face.c) - a) * v);
                let others = (0..planes.len())
                    .filter(|&g| g != f && g != h)
                    .all(|g| (at - planes[g].origin).dot(planes[g].normal) <= 0.0);
                if !others {
                    continue;
                }

                let before = mesh.triangle_count();
                let tie = mesh.split_triangle(t, at)?;
                for child in before..mesh.triangle_count() {
                    strip.insert(child);
                }

                planes[f].on.insert(tie);
                planes[h].on.insert(tie);
            }
        }
    }

    // To a fixed point, as StripClip's: a split along one plane can make an edge newly cross
    // another inside the hull.
    let mut splits = 1;
    while splits > 0 {
        splits = 0;
        for f in 0..planes.len() {
            let made = split_along(mesh, strip, f, planes, &distance)?;
            if made < 0 {
                return Ok(false);
            }

            splits += made;
        }
    }

    // A piece with no corner inside some plane and one outside it is in the air.
    let outside = |mesh: &LevelLineMesh, planes: &[HullClipPlane], t: i32| {
        let face = mesh.triangles()[t as usize];
        (0..planes.len() as i32).any(|f| {
            let (da, db, dc) = (
                distance(mesh, planes, f, face.a),
                distance(mesh, planes, f, face.b),
                distance(mesh, planes, f, face.c),
            );
            da >= 0.0 && db >= 0.0 && dc >= 0.0 && da + db + dc > 0.0
        })
    };
    let mut air: Vec<i32> = strip
        .iter()
        .copied()
        .filter(|&t| outside(mesh, planes, t))
        .collect();
    air.sort_unstable();
    for t in air {
        mesh.remove_triangle(t);
        strip.remove(&t);
    }

    // What is left must lie inside every plane: a piece still straddling one was never split
    // because its crossing lay outside another plane, and the pieces that are left would be in
    // the air in part. (Sorted, so the note names the same first piece every run.)
    let mut straddling: Vec<i32> = strip
        .iter()
        .copied()
        .filter(|&t| {
            let face = mesh.triangles()[t as usize];
            (0..planes.len() as i32).any(|f| {
                distance(mesh, planes, f, face.a) > 0.0
                    || distance(mesh, planes, f, face.b) > 0.0
                    || distance(mesh, planes, f, face.c) > 0.0
            })
        })
        .collect();
    straddling.sort_unstable();
    if let Some(&first) = straddling.first() {
        let mesh = &*mesh;
        let planes = &*planes;
        note(|| {
            let face = mesh.triangles()[first as usize];
            let corners: Vec<String> = [face.a, face.b, face.c]
                .iter()
                .map(|&v| {
                    let readings: Vec<String> = (0..planes.len() as i32)
                        .map(|f| format!("{:.3e}", distance(mesh, planes, f, v)))
                        .collect();
                    format!("{}[{}]", mesh.position(v), readings.join(","))
                })
                .collect();
            format!(
                "{} pieces straddle, first {}",
                straddling.len(),
                corners.join(" ")
            )
        });
        return Ok(false);
    }

    Ok(true)
}

/// The hull end of a fill - clips the strip laid from `start` on the
/// part's supporting planes it crosses, drops the air, and caps the hole in each plane, bounded
/// there by the clip curve and the part's own edges round the mouth. The kept edges round the hole
/// become the end's closing run in `kept`, and the rails' virtual edges leave it. False when the
/// clip or a cap cannot close.
pub fn try_clip_and_cap(
    mesh: &mut LevelLineMesh,
    laid: &mut Vec<i32>,
    start: usize,
    kept: &mut Vec<(i32, i32)>,
) -> Result<bool, LevelLineMeshError> {
    let mut strip: HashSet<i32> = laid[start..].iter().copied().collect();
    let mut planes = crossed_supporting_planes(mesh, &strip);
    note(|| {
        let named: Vec<String> = planes
            .iter()
            .map(|p| format!("{}@{}", p.normal, p.origin))
            .collect();
        format!("planes {}: {}", planes.len(), named.join(" "))
    });
    // `&&` short-circuits: the cap runs only on a clip that held.
    let result = !planes.is_empty()
        && clip(mesh, &mut strip, &mut planes)?
        && cap(mesh, &mut strip, &planes, kept)?;
    if result {
        note(|| "done".to_string());
    }

    laid.truncate(start);
    let mut left: Vec<i32> = strip
        .iter()
        .copied()
        .filter(|&t| !mesh.is_removed(t))
        .collect();
    left.sort_unstable();
    laid.extend(left);
    Ok(result)
}

/// Closes one planar outline, traversed the way its cap runs, by ear
/// clipping in the plane's own frame. An ear is a corner turning the outline's own way with no
/// other outline vertex in or on it; a zero-area corner is no ear. The faces laid join `strip`;
/// false where no ear is left.
pub fn ear_clip(
    mesh: &mut LevelLineMesh,
    loop_: &[i32],
    normal: Vector3,
    strip: &mut HashSet<i32>,
) -> Result<bool, LevelLineMeshError> {
    let seed = if normal.x.abs() < 0.9 {
        Vector3::new(1.0, 0.0, 0.0)
    } else {
        Vector3::new(0.0, 1.0, 0.0)
    };
    let u = seed.cross(normal).get_normal();
    let v = normal.cross(u);
    let origin = mesh.position(loop_[0]);
    let at = |mesh: &LevelLineMesh, vertex: i32| {
        let d = mesh.position(vertex) - origin;
        Vector2::new(d.dot(u), d.dot(v))
    };
    let turn = |a: Vector2, b: Vector2, c: Vector2| {
        ((b.x - a.x) * (c.y - a.y)) - ((b.y - a.y) * (c.x - a.x))
    };

    let mut polygon = loop_.to_vec();
    let mut area = 0.0;
    for i in 0..polygon.len() {
        let (p, q) = (
            at(mesh, polygon[i]),
            at(mesh, polygon[(i + 1) % polygon.len()]),
        );
        area += (p.x * q.y) - (q.x * p.y);
    }

    // Math.Sign throws on NaN.
    assert!(
        !area.is_nan(),
        "Function does not accept floating point Not-a-Number values."
    );
    let sense = if area > 0.0 {
        1.0
    } else if area < 0.0 {
        -1.0
    } else {
        0.0
    };
    // The greatest length, NaN reading as the smallest.
    let mut scale: Option<f64> = None;
    for &p in &polygon {
        let x = at(mesh, p).length();
        scale = Some(match scale {
            Some(value) if !(x > value || value.is_nan()) => value,
            _ => x,
        });
    }
    let scale = scale.expect("an outline has vertices") + 1.0;
    let tiny = 1e-12 * scale * scale;
    while polygon.len() > 3 {
        let mut clipped = false;
        let mut i = 0;
        while i < polygon.len() && !clipped {
            let count = polygon.len();
            let (ia, ib, ic) = (
                polygon[(i + count - 1) % count],
                polygon[i],
                polygon[(i + 1) % count],
            );
            let (a, b, c) = (at(mesh, ia), at(mesh, ib), at(mesh, ic));
            if turn(a, b, c) * sense <= tiny {
                i += 1;
                continue;
            }

            let blocked = polygon
                .iter()
                .filter(|&&p| p != ia && p != ib && p != ic)
                .map(|&p| at(mesh, p))
                .any(|p| {
                    turn(a, b, p) * sense >= 0.0
                        && turn(b, c, p) * sense >= 0.0
                        && turn(c, a, p) * sense >= 0.0
                });
            if blocked {
                i += 1;
                continue;
            }

            strip.insert(mesh.add_face(ia, ib, ic, 0)?);
            polygon.remove(i);
            clipped = true;
            i += 1;
        }

        if !clipped {
            return Ok(false);
        }
    }

    if polygon.len() == 3 {
        strip.insert(mesh.add_face(polygon[0], polygon[1], polygon[2], 0)?);
    }

    Ok(true)
}

/// Caps the open boundary lying in the hull planes, one plane at a time - each
/// plane's outline (its clip curve, the part's edges round the mouth, and the part's edge it shares
/// with the next plane's cap) closed on its own by [`ear_clip`]. The kept edges of the outline
/// become the end's closing run in `kept`.
pub fn cap(
    mesh: &mut LevelLineMesh,
    strip: &mut HashSet<i32>,
    planes: &[HullClipPlane],
    kept: &mut Vec<(i32, i32)>,
) -> Result<bool, LevelLineMeshError> {
    {
        let mesh = &*mesh;
        let strip = &*strip;
        kept.retain(|&(a, b)| mesh.triangles_on(a, b).iter().any(|t| !strip.contains(t)));
    }

    let mut per_plane: Vec<IndexMap<i32, i32>> = planes.iter().map(|_| IndexMap::new()).collect();
    for (low, high) in mesh.edges() {
        let standing = mesh.triangles_on(low, high);
        if standing.len() != 1 {
            continue;
        }

        // An edge of the part's own on two planes is capped in the plane its kept triangle is NOT
        // in; an edge of a kept face in ONE plane is that plane's.
        let standing0 = standing[0];
        let standing_face = mesh.triangles()[standing0 as usize];
        let holding: Vec<usize> = (0..planes.len())
            .filter(|&f| planes[f].on.contains(&low) && planes[f].on.contains(&high))
            .collect();
        let mut on: Vec<usize> = holding
            .iter()
            .copied()
            .filter(|&f| {
                !(planes[f].on.contains(&standing_face.a)
                    && planes[f].on.contains(&standing_face.b)
                    && planes[f].on.contains(&standing_face.c))
            })
            .collect();
        if on.is_empty() {
            on = holding;
        }

        if on.is_empty() {
            continue;
        }

        if on.len() > 1 {
            note(|| {
                format!(
                    "open edge {}-{} on {} planes",
                    mesh.position(low),
                    mesh.position(high),
                    on.len()
                )
            });
            return Ok(false);
        }

        let (a, b) = directed(standing_face, low, high);
        if !strip.contains(&standing0) {
            kept.push((a, b));
        }

        let next = &mut per_plane[on[0]];
        if next.contains_key(&b) {
            note(|| format!("outline branches at {}", mesh.position(b)));
            return Ok(false);
        }

        next.insert(b, a);
    }

    for f in 0..planes.len() {
        let mut next = std::mem::take(&mut per_plane[f]);
        let outline = |mesh: &LevelLineMesh, strip: &HashSet<i32>| {
            mesh.edges()
                .into_iter()
                .filter(|&(low, high)| mesh.triangles_on(low, high).len() == 1)
                .map(|(low, high)| {
                    let planes_on: Vec<String> = (0..planes.len())
                        .filter(|&g| planes[g].on.contains(&low) && planes[g].on.contains(&high))
                        .map(|g| g.to_string())
                        .collect();
                    let side = if strip.contains(&mesh.triangles_on(low, high)[0]) {
                        "s"
                    } else {
                        "k"
                    };
                    format!(
                        "{}-{}[{}]{}",
                        mesh.position(low),
                        mesh.position(high),
                        planes_on.join(","),
                        side
                    )
                })
                .collect::<Vec<_>>()
                .join(" ")
        };

        // Two caps meeting along the part's own edge between their planes: each plane's outline
        // runs open from one vertex on that line to another, joined here, E back to S.
        let starts: Vec<i32> = next
            .keys()
            .copied()
            .filter(|k| !next.values().any(|v| v == k))
            .collect();
        let ends: Vec<i32> = next
            .values()
            .copied()
            .filter(|v| !next.contains_key(v))
            .collect();
        if starts.len() == 1
            && ends.len() == 1
            && (0..planes.len()).any(|g| {
                g != f && planes[g].on.contains(&starts[0]) && planes[g].on.contains(&ends[0])
            })
        {
            next.insert(ends[0], starts[0]);
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

            let closed = at == loop_[0];
            if !closed || !ear_clip(mesh, &loop_, planes[f].normal, strip)? {
                let mesh = &*mesh;
                let strip = &*strip;
                note(|| {
                    let points: Vec<String> = loop_
                        .iter()
                        .map(|&i| mesh.position(i).to_string())
                        .collect();
                    format!(
                        "plane {}: loop {} {}; outline {}",
                        planes[f].normal,
                        if !closed { "open" } else { "no ear" },
                        points.join(" "),
                        outline(mesh, strip)
                    )
                });
                return Ok(false);
            }
        }
    }

    Ok(true)
}

/// The signed distance of a vertex from a plane, by index: handed the mesh and planes it reads,
/// since [`split_along`] changes both.
pub type PlaneDistance<'a> = &'a dyn Fn(&LevelLineMesh, &[HullClipPlane], i32, i32) -> f64;

/// Splits the strip's edges crossing plane `f` where the crossing lies
/// inside every other plane, until none is left; the count made, or -1 where a kept edge crosses
/// or a corner already stands at the crossing.
pub fn split_along(
    mesh: &mut LevelLineMesh,
    strip: &mut HashSet<i32>,
    f: usize,
    planes: &mut [HullClipPlane],
    distance: PlaneDistance,
) -> Result<i32, LevelLineMeshError> {
    let mut made = 0;
    loop {
        let mut found: Option<(i32, i32, Vector3)> = None;
        let mut ordered: Vec<i32> = strip.iter().copied().collect();
        ordered.sort_unstable();
        for t in ordered {
            let face = mesh.triangles()[t as usize];
            let mut corner = 0;
            while corner < 3 && found.is_none() {
                let (a, b) = (face.corner(corner), face.corner((corner + 1) % 3));
                corner += 1;
                let (da, db) = (
                    distance(mesh, planes, f as i32, a),
                    distance(mesh, planes, f as i32, b),
                );
                if !((da < 0.0 && db > 0.0) || (da > 0.0 && db < 0.0)) {
                    continue;
                }

                let s = da / (da - db);
                let inside = (0..planes.len()).filter(|&h| h != f).all(|h| {
                    let h = h as i32;
                    distance(mesh, planes, h, a)
                        + ((distance(mesh, planes, h, b) - distance(mesh, planes, h, a)) * s)
                        <= 0.0
                });
                if inside {
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

        if mesh.triangles_on(ea, eb).iter().any(|t| !strip.contains(t)) {
            note(|| {
                format!(
                    "kept edge {}-{} crosses plane {} at {}",
                    mesh.position(ea),
                    mesh.position(eb),
                    planes[f].normal,
                    point
                )
            });
            return Ok(-1);
        }

        let before = mesh.triangle_count();
        let vertices = mesh.vertex_count();
        // Exact only: a clip needs a real vertex ON its plane, not a neighbour a float step off it.
        let vertex = mesh.split_edge(ea, eb, point, false)?;
        if vertex < vertices && vertex != ea && vertex != eb {
            // A corner already stands at the crossing: refused by name.
            note(|| format!("corner {} at the crossing {}", mesh.position(vertex), point));
            return Ok(-1);
        }

        made += 1;
        for child in before..mesh.triangle_count() {
            strip.insert(child);
        }

        for plane in planes.iter_mut() {
            if plane.on.contains(&ea) && plane.on.contains(&eb) {
                plane.on.insert(vertex);
            }
        }

        planes[f].on.insert(vertex);
    }
}
