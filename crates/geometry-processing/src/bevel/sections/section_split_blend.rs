//! Redrawing a run's SPLIT stations - vertices on the straight chain between two unsplit
//! neighbours - from those neighbours rather than from their own fans
//! (`Bevel/Sections/SectionSplitBlend.cs`, a C# `internal static class`, so a module here): what
//! the blend did at one station ([`SplitPlacement`]) and whether it split it. The blend and its switch land with the
//! core's section packages as free functions in this module.

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
