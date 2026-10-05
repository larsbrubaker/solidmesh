//! Redrawing a run's SPLIT stations - vertices on the straight chain between two unsplit
//! neighbours - from those neighbours rather than from their own fans
//! (`Bevel/Sections/SectionSplitBlend.cs`, a C# `internal static class`, so a module here): what
//! the blend did at one station ([`SplitPlacement`]) and whether it split it, the switch that turns
//! absorbing split vertices on and off, and the blend's small readings (the nearest unsplit
//! neighbour, a point or number part way between two). The blend itself (`Apply`, `TryBlend`)
//! lands with a later run of this package as free functions in this module.

use std::sync::atomic::{AtomicBool, Ordering};

use vector_math::Vector3;

/// C# `SectionSplitBlend.SplitPlacement`: what the blend did at one station, kept so the passes
/// AFTER it do not have to read the split a second time. A later pass that treats a split vertex
/// differently - the surface walk, which may not walk one from its own station - asks this rather
/// than re-deriving the reading. A station is split exactly where both neighbours are set.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SplitPlacement {
    /// The nearest unsplit station before this one, or -1 where this station is not split.
    pub before: i32,
    /// The nearest unsplit station after this one, or -1 where this station is not split.
    pub after: i32,
    /// Where along the chord from `before`'s station to `after`'s this station stands - the very
    /// parameter the blend drew the whole section at.
    pub parameter: f64,
}

impl SplitPlacement {
    /// C# `IsSplit`: whether the blend redrew this station as a split vertex.
    pub fn is_split(&self) -> bool {
        self.before >= 0 && self.after >= 0
    }
}

/// C# `AbsorbSplitVertices` (a static property, initializer `true`): whether a run's split vertices
/// are absorbed into their neighbours' strips. A process-wide switch, as C#'s static is, kept
/// thread-safe because the kernel runs on worker threads (bevel README, "Decided").
static ABSORB_SPLIT_VERTICES: AtomicBool = AtomicBool::new(true);

/// Reads [`ABSORB_SPLIT_VERTICES`].
pub fn absorb_split_vertices() -> bool {
    ABSORB_SPLIT_VERTICES.load(Ordering::Relaxed)
}

/// Sets [`ABSORB_SPLIT_VERTICES`].
pub fn set_absorb_split_vertices(absorb: bool) {
    ABSORB_SPLIT_VERTICES.store(absorb, Ordering::Relaxed);
}

impl SplitPlacement {
    /// C#'s `SplitPlacement` constructor.
    pub fn new(before: i32, after: i32, parameter: f64) -> Self {
        Self {
            before,
            after,
            parameter,
        }
    }
}

/// C# `NearestThatIsNotSplit`: the nearest station `step` (±1) round the run from `from` that is not
/// split, or -1 where every other station is.
pub fn nearest_that_is_not_split(is_split: &[bool], from: i32, step: i32) -> i32 {
    let count = is_split.len() as i32;
    let mut at = from;

    for _ in 0..count {
        at = ((at + step) % count + count) % count;

        if at == from {
            return -1;
        }

        if !is_split[at as usize] {
            return at;
        }
    }

    -1
}

/// C# `HowFarAlong`: how far along the chain's `tangent` the `wanted` point stands from the `drawn`
/// one.
pub fn how_far_along(wanted: Vector3, drawn: Vector3, tangent: Vector3) -> f64 {
    (wanted - drawn).dot(tangent)
}

/// C# `Between` (points): the point `parameter` of the way from `from` to `to`.
pub fn between(from: Vector3, to: Vector3, parameter: f64) -> Vector3 {
    from + ((to - from) * parameter)
}

/// C# `Between` (numbers): the number `parameter` of the way from `from` to `to`.
pub fn between_values(from: f64, to: f64, parameter: f64) -> f64 {
    from + ((to - from) * parameter)
}
