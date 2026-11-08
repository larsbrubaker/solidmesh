//! The flat-ends switch of the corner cap: an END FACE end closed flat by the flat hole filler
//! instead of the zips and fans, on by default (the hole-fill switch, and the scope a build holds
//! both in, are in `corner_cap_hole_fill_switch.rs`), and its two closes, the flat fill
//! ([`closes_flat`]) and the fan from an end vertex on the run
//! ([`fans_from_the_end_vertex_on_the_run`]).

use std::cell::{Cell, RefCell};
use std::rc::Rc;

use solidmesh_math::Vector3;

use super::{corner_fill, flat_hole_fill};
use super::{HoleFillTriangle, LevelLineMesh, LevelLineMeshError, LevelLineWork};

/// The flat fill's snap on a mesh with no `SamePlace` of its own.
const DEFAULT_SNAP: f64 = 1e-9;

/// A watcher of each end hole closed flat: the mesh, the hole's loop (as laid) and the fill's
/// triangles over it.
pub type WatchFlatEnds = Rc<dyn Fn(&LevelLineMesh, &[i32], &[HoleFillTriangle])>;

thread_local! {
    /// A read-only observer of each end hole closed
    /// flat. Per thread; `None` by default.
    pub static WATCH_FLAT_ENDS: RefCell<Option<WatchFlatEnds>> = const { RefCell::new(None) };
}

thread_local! {
    /// This thread's flat-ends setting - `None` leaves it on. Held for a build by
    /// `corner_cap_hole_fill_switch::hold_corner_cap_switches`, from the request's routing.
    static USE_FLAT_ENDS_OVERRIDE: Cell<Option<bool>> = const { Cell::new(None) };
}

/// Sets (or, with `None`, clears back to on) this thread's [`use_flat_ends`].
pub fn set_use_flat_ends_override(value: Option<bool>) {
    USE_FLAT_ENDS_OVERRIDE.with(|o| o.set(value));
}

/// This thread's setting of [`use_flat_ends`], `None` where it has none.
pub fn use_flat_ends_override() -> Option<bool> {
    USE_FLAT_ENDS_OVERRIDE.with(|o| o.get())
}

/// Whether an end-face hole is closed by the flat hole filler (and its
/// end ray bounded at the end plane).
pub fn use_flat_ends() -> bool {
    use_flat_ends_override().unwrap_or(true)
}

/// The end hole's loop as the cap lays its edges: the row forward, then the run back from its
/// last-but-one vertex to its second.
fn end_hole_loop(row: &[i32], run: &[i32]) -> Vec<i32> {
    let inner = if run.len() > 2 {
        &run[1..run.len() - 1]
    } else {
        &[][..]
    };
    row.iter()
        .copied()
        .chain(inner.iter().rev().copied())
        .collect()
}

/// The end hole between `row` (walked the way the cap lays it) and
/// `run` (walked the way the kept surface does, sharing the row's two ends) closed flat, or false
/// with nothing laid where the hole is not planar within the cut's snap - the caller keeps its own
/// close for that.
pub fn closes_flat(
    mesh: &mut LevelLineMesh,
    row: &[i32],
    run: &[i32],
    added: &mut Vec<i32>,
) -> Result<bool, LevelLineMeshError> {
    let loop_ = end_hole_loop(row, run);
    let snap = if mesh.same_place() > 0.0 {
        mesh.same_place()
    } else {
        DEFAULT_SNAP
    };
    let points: Vec<Vector3> = loop_.iter().map(|&i| mesh.position(i)).collect();
    let Some((triangles, _)) = flat_hole_fill::fill(&points, snap, None) else {
        return Ok(false);
    };

    for triangle in &triangles {
        LevelLineWork::charge(1)?;
        corner_fill::lay(
            mesh,
            loop_[triangle.a as usize],
            loop_[triangle.b as usize],
            loop_[triangle.c as usize],
            added,
        )?;
    }

    let watch = WATCH_FLAT_ENDS.with(|w| w.borrow().clone());
    if let Some(watch) = watch {
        watch(mesh, &loop_, &triangles);
    }

    Ok(true)
}

/// The end hole fanned from the end vertex `v` where
/// `v` is itself a vertex of the run, inside a straight stretch of the rim. Every loop edge not at
/// `v` is fanned to it, and only where each such triangle turns the loop's way (Newell's normal) is
/// anything laid; false with nothing laid otherwise.
pub fn fans_from_the_end_vertex_on_the_run(
    mesh: &mut LevelLineMesh,
    row: &[i32],
    run: &[i32],
    v: Vector3,
    added: &mut Vec<i32>,
) -> Result<bool, LevelLineMeshError> {
    let loop_ = end_hole_loop(row, run);
    let at = loop_.iter().position(|&i| mesh.position(i) == v);
    let Some(at) = at else {
        return Ok(false);
    };
    if loop_.len() < 4 {
        return Ok(false);
    }

    let count = loop_.len();
    let before = mesh.position(loop_[(at + count - 1) % count]) - v;
    let after = mesh.position(loop_[(at + 1) % count]) - v;
    #[allow(clippy::neg_cmp_op_on_partial_ord)]
    let turns = !(before.dot(after) < 0.0)
        || before.cross(after).length() > 1e-6 * before.length() * after.length();
    if turns {
        return Ok(false);
    }

    let mut newell = Vector3::ZERO;
    for i in 0..count {
        let a = mesh.position(loop_[i]);
        let b = mesh.position(loop_[(i + 1) % count]);
        newell += Vector3::new(
            (a.y - b.y) * (a.z + b.z),
            (a.z - b.z) * (a.x + b.x),
            (a.x - b.x) * (a.y + b.y),
        );
    }

    let mut fan = Vec::new();
    for i in 0..count {
        let next = (i + 1) % count;
        if i == at || next == at {
            continue;
        }

        let a = mesh.position(loop_[i]);
        let b = mesh.position(loop_[next]);
        #[allow(clippy::neg_cmp_op_on_partial_ord)]
        let away = !((b - a).cross(v - a).dot(newell) > 0.0);
        if away {
            return Ok(false);
        }

        fan.push((loop_[i], loop_[next]));
    }

    for (a, b) in fan {
        LevelLineWork::charge(1)?;
        corner_fill::lay(mesh, a, b, loop_[at], added)?;
    }

    Ok(true)
}

/// A flat hole `hole_loop` whose corner is fanned by
/// end-face triangles lying in the hole's plane and holding the whole hole - those triangles taken
/// out of the mesh and the loop rerouted along the fan's far path, so the end face's piece at the
/// chain's end is closed with the hole. Only where the fan borders no other hole and the rerouted
/// hole closes flat; otherwise the loop is handed back unchanged. Each kept-boundary change is
/// recorded in `boundary_changes` as `(a, b, joins)`.
pub fn with_the_end_face_triangle_taken(
    mesh: &mut LevelLineMesh,
    hole_loop: &[i32],
    mut boundary_changes: Option<&mut Vec<(i32, i32, bool)>>,
) -> Vec<i32> {
    let n = hole_loop.len();
    if n < 4 {
        return hole_loop.to_vec();
    }

    // The loop's plane, by Newell's sum: the hole must lie flat for an end face to hold it.
    let mut normal = Vector3::default();
    for i in 0..n {
        let u = mesh.position(hole_loop[i]);
        let w = mesh.position(hole_loop[(i + 1) % n]);
        normal += Vector3::new(
            (u.y - w.y) * (u.z + w.z),
            (u.z - w.z) * (u.x + w.x),
            (u.x - w.x) * (u.y + w.y),
        );
    }

    if normal.length() <= 0.0 {
        return hole_loop.to_vec();
    }

    let normal = normal.get_normal();
    let origin = mesh.position(hole_loop[0]);
    let snap = solidmesh_math::ieee_math::max_f64(mesh.same_place, DEFAULT_SNAP) * 1000.0;
    let on_the_plane =
        |mesh: &LevelLineMesh, v: i32| (mesh.position(v) - origin).dot(normal).abs() <= snap;
    if !hole_loop.iter().all(|&v| on_the_plane(mesh, v)) {
        return hole_loop.to_vec();
    }

    for k in 0..n {
        let (before, corner, after) = (
            hole_loop[(k + n - 1) % n],
            hole_loop[k],
            hole_loop[(k + 1) % n],
        );

        // The kept triangles round the corner from its edge to `before` to its edge to `after`,
        // all in the hole's plane.
        let mut fan: Vec<i32> = Vec::new();
        let mut far = vec![before];
        let mut at = before;
        let mut fanned = false;
        for _ in 0..64 {
            let next: Vec<i32> = mesh
                .triangles_on(at, corner)
                .iter()
                .copied()
                .filter(|&t| !mesh.is_removed(t) && !fan.contains(&t))
                .collect();
            if next.len() != 1 {
                break;
            }

            let face = mesh.triangles[next[0] as usize];
            let third = if face.a != at && face.a != corner {
                face.a
            } else if face.b != at && face.b != corner {
                face.b
            } else {
                face.c
            };
            if !on_the_plane(mesh, third) {
                break;
            }

            fan.push(next[0]);
            far.push(third);
            at = third;
            if third == after {
                fanned = true;
                break;
            }
        }

        if !fanned || far.len() < 2 {
            continue;
        }

        // The hole must lie inside that piece, read in the plane.
        let axis_u = (if normal.x.abs() < 0.9 {
            Vector3::new(1.0, 0.0, 0.0)
        } else {
            Vector3::new(0.0, 1.0, 0.0)
        })
        .cross(normal)
        .get_normal();
        let axis_v = normal.cross(axis_u);
        let polygon: Vec<(f64, f64)> = far
            .iter()
            .chain(std::iter::once(&corner))
            .map(|&v| {
                let x = mesh.position(v);
                (x.dot(axis_u), x.dot(axis_v))
            })
            .collect();
        let inside = |x: Vector3| {
            let (px, py) = (x.dot(axis_u), x.dot(axis_v));
            let mut winding = 0.0;
            for i in 0..polygon.len() {
                let (ax, ay) = (polygon[i].0 - px, polygon[i].1 - py);
                let next = polygon[(i + 1) % polygon.len()];
                let (bx, by) = (next.0 - px, next.1 - py);
                if (ax * ax) + (ay * ay) <= snap * snap || (bx * bx) + (by * by) <= snap * snap {
                    return true;
                }

                winding += libm::atan2((ax * by) - (ay * bx), (ax * bx) + (ay * by));
            }

            // On the polygon's boundary the winding is half a turn; inside it is a whole one.
            winding.abs() > std::f64::consts::PI - 1e-6
        };
        if !hole_loop.iter().all(|&v| inside(mesh.position(v))) {
            continue;
        }

        let mut taken = Vec::with_capacity(n + far.len());
        for (i, &v) in hole_loop.iter().enumerate() {
            if i == k {
                taken.extend_from_slice(&far[1..far.len() - 1]);
            } else {
                taken.push(v);
            }
        }

        // Taken only where the fan borders no other hole.
        let walled = fan.iter().all(|&t| {
            let face = mesh.triangles[t as usize];
            [(face.a, face.b), (face.b, face.c), (face.c, face.a)]
                .into_iter()
                .all(|(p, q)| {
                    (p == corner && (q == before || q == after))
                        || (q == corner && (p == before || p == after))
                        || mesh
                            .triangles_on(p, q)
                            .iter()
                            .filter(|&&x| !mesh.is_removed(x))
                            .count()
                            == 2
                })
        });
        if !walled {
            continue;
        }

        // Taken only where the hole it leaves closes flat.
        let points: Vec<Vector3> = taken.iter().map(|&v| mesh.position(v)).collect();
        let fill_snap = if mesh.same_place > 0.0 {
            mesh.same_place
        } else {
            DEFAULT_SNAP
        };
        if flat_hole_fill::fill(&points, fill_snap, None).is_none() {
            continue;
        }

        for &t in &fan {
            mesh.remove_triangle(t);
        }

        // The kept boundary moves from the corner's two edges to the fan's far path.
        if let Some(changes) = boundary_changes.as_mut() {
            changes.push((after, corner, false));
            changes.push((corner, before, false));
            for i in 0..far.len() - 1 {
                changes.push((far[i + 1], far[i], true));
            }
        }

        return taken;
    }

    hole_loop.to_vec()
}

/// A hole, walked the way the cap lays its edges, closed flat by the
/// flat hole fill; false with nothing laid where it is not planar within the cut's snap.
pub fn closes_flat_hole(
    mesh: &mut LevelLineMesh,
    hole_loop: &[i32],
    added: &mut Vec<i32>,
) -> Result<bool, LevelLineMeshError> {
    let snap = if mesh.same_place() > 0.0 {
        mesh.same_place()
    } else {
        DEFAULT_SNAP
    };
    let points: Vec<Vector3> = hole_loop.iter().map(|&i| mesh.position(i)).collect();
    let Some((triangles, _)) = flat_hole_fill::fill(&points, snap, None) else {
        return Ok(false);
    };

    for triangle in &triangles {
        LevelLineWork::charge(1)?;
        corner_fill::lay(
            mesh,
            hole_loop[triangle.a as usize],
            hole_loop[triangle.b as usize],
            hole_loop[triangle.c as usize],
            added,
        )?;
    }

    Ok(true)
}
