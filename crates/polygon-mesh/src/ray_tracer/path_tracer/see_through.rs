//! Part opacity: how a ray goes past a part that is only partly there.
//!
//! A part's opacity `a` says what fraction of it is present. A ray meeting the part finds it
//! there with probability `a`, and otherwise goes on along the same line as if the part did not
//! exist. "The part" is the whole part: a closed solid has two surfaces along a line, and the ray
//! that went through its first surface goes through its second without a new decision, so a
//! closed part at opacity `a` covers exactly `a` of what is behind it, the same as a single sheet
//! does. [`StraightLine`] carries what one straight line has already gone through.
//!
//! A pass-through changes nothing but the ray's origin: no throughput, no bounce, the same
//! direction, and the same record of how the path got here, so a sun reached past the part is
//! weighted by the same density as one reached with nothing in the way. See `integrator` for how
//! that keeps the two ways of reaching the sun consistent.
//!
//! **The cap.** One straight line goes through at most [`MAX_CROSSINGS_PER_LINE`] surfaces of
//! see-through parts, which ends a line that can't otherwise end: a stack of see-through parts of
//! any depth, or a broken face whose continuing ray starts where it was. A line past the cap ends
//! there, as a path past the bounce limit does: a path's estimate loses what it still carries and
//! a shadow ray counts as blocked. Opaque surfaces don't count against the cap; a pass straight
//! through a seam between two glass parts (`glass_seam`) counts as one crossing.
//!
//! Decisions are drawn only for a partly-there part, and never for one at opacity 0 or 1, so a
//! scene of opaque and invisible parts draws no random number to decide.

use solidmesh_math::{IntersectionType, Ray, Vector3};

use super::pixel_random::PixelRandom;
use super::render_scene::RenderScene;
use crate::ray_tracer::SurfacePoint;

/// The most surfaces of see-through parts one straight line goes through before it ends. A line
/// through a closed part counts its two surfaces. Far above what a scene shows: 512 surfaces
/// of parts at opacity 0.05 still leave a line almost nothing to carry through them.
pub const MAX_CROSSINGS_PER_LINE: u32 = 512;

/// What a line does at a hit on a part.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum AtHit {
    /// The part is there: the line ends on it.
    Stops,
    /// The part is not there: the line goes on past it.
    PassesThrough,
    /// The line has gone through as many surfaces as it may.
    Ended,
}

/// The parts one straight line has gone through so far. Reuses one list per path, and the list
/// stays unallocated in a scene with no see-through part.
#[derive(Default)]
pub(super) struct StraightLine {
    passed: Vec<usize>,
    crossings: u32,
}

impl StraightLine {
    /// Starts a new straight line.
    pub(super) fn restart(&mut self) {
        self.passed.clear();
        self.crossings = 0;
    }

    /// Counts a crossing of a see-through part's surface; the crossing's verdict is whether this
    /// line has been through that part before.
    fn count_crossing(&mut self, part: usize) -> Crossing {
        if self.crossings >= MAX_CROSSINGS_PER_LINE {
            return Crossing::Capped;
        }
        self.crossings += 1;
        if self.passed.contains(&part) {
            Crossing::AlreadyPassed
        } else {
            self.passed.push(part);
            Crossing::First
        }
    }

    /// Records that this line starts inside `part`, which the line before found not there, so
    /// the part's far surface is passed as the rest of it was.
    pub(super) fn starts_past(&mut self, part: usize) {
        self.passed.push(part);
    }

    /// Counts a pass straight through a seam between two glass parts, which crosses two
    /// surfaces at once as one; false once the line has gone through as many as it may.
    pub(super) fn count_pass(&mut self) -> bool {
        if self.crossings >= MAX_CROSSINGS_PER_LINE {
            return false;
        }
        self.crossings += 1;
        true
    }

    /// Decides what the line does at a hit on part `part_index`, drawing from `random`.
    pub(super) fn at_hit(
        &mut self,
        scene: &RenderScene,
        part_index: usize,
        random: &mut PixelRandom,
    ) -> AtHit {
        let opacity = scene.part_opacity(part_index).unwrap_or(1.0);
        if opacity >= 1.0 {
            return AtHit::Stops;
        }
        match self.count_crossing(part_index) {
            Crossing::Capped => AtHit::Ended,
            Crossing::AlreadyPassed => AtHit::PassesThrough,
            Crossing::First => {
                // A draw in [0, 1) is below `opacity` with probability `opacity`. At opacity 0
                // nothing is drawn: the part isn't there for anyone.
                if opacity > 0.0 && random.next_unit() < opacity {
                    self.passed.pop();
                    AtHit::Stops
                } else {
                    AtHit::PassesThrough
                }
            }
        }
    }

    /// The fraction of a line a part leaves uncovered, or `None` once the line has gone through
    /// as many surfaces as it may. A part the line has been through already covers nothing more.
    pub(super) fn uncovered_by(&mut self, part: usize, opacity: f64) -> Option<f64> {
        match self.count_crossing(part) {
            Crossing::Capped => None,
            Crossing::AlreadyPassed => Some(1.0),
            Crossing::First => Some(1.0 - opacity),
        }
    }
}

/// What the line had done before at a surface of a see-through part.
enum Crossing {
    First,
    AlreadyPassed,
    Capped,
}

/// The ray that continues `direction` from the far side of `point`: the same line, started just
/// past the surface it went through.
pub(super) fn continued_ray(point: &SurfacePoint, direction: Vector3) -> Ray {
    Ray::with_limits(
        point.next_ray_origin(direction),
        direction,
        0.0,
        f64::INFINITY,
        IntersectionType::Both,
    )
}
