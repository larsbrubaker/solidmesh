//! The flat-ends switch of the corner cap: an END FACE end closed flat by the flat hole filler
//! instead of the zips and fans, on by default. Ports the switch of
//! `Bevel/LevelLine/CornerCap.FlatEnds.cs` (a partial of the C# static class `CornerCap`; its
//! hole-fill switch is in `corner_cap_hole_fill_switch.rs`) and its two closes, the flat fill
//! ([`closes_flat`]) and the fan from an end vertex on the run
//! ([`fans_from_the_end_vertex_on_the_run`]).

use std::cell::{Cell, RefCell};
use std::rc::Rc;
use std::sync::OnceLock;

use vector_math::Vector3;

use super::level_line_routing::reads_as_off;
use super::{corner_fill, flat_hole_fill};
use super::{HoleFillTriangle, LevelLineMesh, LevelLineMeshError, LevelLineWork};

/// C# `CornerCap.DefaultSnap`: the flat fill's snap on a mesh with no `SamePlace` of its own.
const DEFAULT_SNAP: f64 = 1e-9;

/// A watcher of each end hole closed flat: the mesh, the hole's loop (as laid) and the fill's
/// triangles over it.
pub type WatchFlatEnds = Rc<dyn Fn(&LevelLineMesh, &[i32], &[HoleFillTriangle])>;

thread_local! {
    /// C# `[ThreadStatic] CornerCap.WatchFlatEnds`: a read-only observer of each end hole closed
    /// flat. Per thread; `None` by default.
    pub static WATCH_FLAT_ENDS: RefCell<Option<WatchFlatEnds>> = const { RefCell::new(None) };
}

/// C# `CornerCap.UseFlatEndsVariable`: the environment variable that turns [`use_flat_ends`] off
/// ("0" or "false"); unset or anything else leaves it on.
pub const USE_FLAT_ENDS_VARIABLE: &str = "MATTERCAD_BEVEL_FLATENDS";

/// C# `CornerCap.FlatEndsFromTheEnvironment`: read once, when first asked - the developer opt-out.
/// On by default since the owner files built no worse with it and the kernel suite is green on
/// it. The browser has no process environment, so there it is always on.
fn flat_ends_from_the_environment() -> bool {
    static FROM_THE_ENVIRONMENT: OnceLock<bool> = OnceLock::new();
    *FROM_THE_ENVIRONMENT
        .get_or_init(|| !reads_as_off(std::env::var(USE_FLAT_ENDS_VARIABLE).ok().as_deref()))
}

thread_local! {
    /// C# `[ThreadStatic] CornerCap.UseFlatEndsOverride`: a per-thread override of the
    /// environment's answer - `None` leaves it to the variable. Per thread so a test that turns
    /// flat ends off never changes what a parallel test builds.
    static USE_FLAT_ENDS_OVERRIDE: Cell<Option<bool>> = const { Cell::new(None) };
}

/// Sets (or, with `None`, clears) this thread's override of [`use_flat_ends`] (C#'s
/// `UseFlatEndsOverride` setter).
pub fn set_use_flat_ends_override(value: Option<bool>) {
    USE_FLAT_ENDS_OVERRIDE.with(|o| o.set(value));
}

/// This thread's override of [`use_flat_ends`] (C#'s `UseFlatEndsOverride` getter).
pub fn use_flat_ends_override() -> Option<bool> {
    USE_FLAT_ENDS_OVERRIDE.with(|o| o.get())
}

/// C# `CornerCap.UseFlatEnds`: whether an end-face hole is closed by the flat hole filler (and its
/// end ray bounded at the end plane).
pub fn use_flat_ends() -> bool {
    use_flat_ends_override().unwrap_or_else(flat_ends_from_the_environment)
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

/// C# `CornerCap.ClosesFlat`: the end hole between `row` (walked the way the cap lays it) and
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

/// C# `CornerCap.FansFromTheEndVertexOnTheRun`: the end hole fanned from the end vertex `v` where
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
