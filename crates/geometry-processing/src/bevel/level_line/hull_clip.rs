//! The HULL mode of the strip clip: where a concave bead's rows are run on past the part itself, no
//! neighbour face stands there to grow, so the part's own SUPPORTING planes bound it instead - every
//! plane of a kept face with the whole kept surface on its inner side - and every piece of the
//! strip OUTSIDE any of them is dropped. Ports the nested type of `Bevel/LevelLine/HullClip.cs` (a
//! C# static class): one supporting plane ([`HullClipPlane`]), the kept surface's supporting
//! planes the strip crosses ([`crossed_supporting_planes`]), the face-directed edge ([`directed`])
//! and the thread's last outcome ([`note`], [`last_outcome`]). The clip itself lands with later
//! packages.

use std::cell::RefCell;
use std::collections::HashSet;

use indexmap::IndexSet;
use vector_math::{cs_math, Vector3};

use super::{FieldCounters, LevelLineMesh, LevelLineTriangle};

thread_local! {
    /// Where the last hull end on this thread stopped - "done" or the stage that refused - for a
    /// test to name (C# `[ThreadStatic] LastOutcome`). Written only while the meter is on.
    static LAST_OUTCOME: RefCell<Option<String>> = const { RefCell::new(None) };
}

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

/// C# `HullClip.Plane`: a supporting plane - a point on it, its outward normal, and the vertices
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
    // C# `Distinct()`: first-appearance order.
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
            cs_math::min_f64(low.x, p.x),
            cs_math::min_f64(low.y, p.y),
            cs_math::min_f64(low.z, p.z),
        );
        high = Vector3::new(
            cs_math::max_f64(high.x, p.x),
            cs_math::max_f64(high.y, p.y),
            cs_math::max_f64(high.z, p.z),
        );
    }

    let tolerance = 1e-9 * cs_math::max_f64(1.0, (high - low).length());
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
