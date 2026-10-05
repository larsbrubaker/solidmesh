//! Flow rung 2: the strip clipped on a GROWING neighbour's planes, and the neighbour's caps. Rows
//! laid whole across a virtual rail overlap the neighbour that stands past the reached crease;
//! each of its faces on that crease clips the strip, the strip where EVERY face's distance is below
//! zero is dropped, and the neighbour grows a cap in each face's plane. Ports
//! `Bevel/LevelLine/StripClip.cs` (a C# static class, so a module of functions): so far the
//! directed edge, the opposite corner, the cap snap and the last outcome note ([`note`],
//! [`last_outcome`]); the clip itself lands with its method packages.

use std::cell::RefCell;

use super::LevelLineTriangle;

/// C# `StripClip.CapSnap`: the snap a cap's outline is read flat by: the build's own (BandGap's
/// 1e-9), so a cap whose outline is off its plane by more keeps the fan it always had.
pub const CAP_SNAP: f64 = 1e-9;

thread_local! {
    /// C# `[ThreadStatic] StripClip.LastOutcome`.
    static LAST_OUTCOME: RefCell<Option<String>> = const { RefCell::new(None) };
}

/// C# `StripClip.LastOutcome`: what the last clip on this thread did, for the sweep's no-result
/// rows (`None` before any clip).
pub fn last_outcome() -> Option<String> {
    LAST_OUTCOME.with(|o| o.borrow().clone())
}

/// C# `StripClip.Note`: records the clip's outcome. Written always, not only under the meter: the
/// sweep classifies no-result rows by the refusing site, and a clip runs a handful of times per
/// fill, so the string costs nothing beside the mesh work.
pub fn note(outcome: impl FnOnce() -> String) {
    let text = outcome();
    LAST_OUTCOME.with(|o| *o.borrow_mut() = Some(text));
}

/// C# `StripClip.OppositeCorner`: the corner of `face` that is neither `a` nor `b`.
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
