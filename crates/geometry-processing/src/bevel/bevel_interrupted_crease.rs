//! Whether a bead's end is where its crease ENDED or only where something INTERRUPTED it - the
//! difference between stopping square there and running on through
//! (`Bevel/BevelInterruptedCrease.cs`, a C# `internal static class`, so a module here): the
//! running reading of how the faces bounding one end lean against the run ([`Leaning`]) and what
//! one end's reading came to ([`Interruption`]). The readings land with the core's packages as
//! `impl` blocks and free functions in this module.

/// C# `BevelInterruptedCrease.Leaning` (a mutable struct): which way the faces bounding one end
/// lean against the run, accumulated as the cap plane scan meets them, and the reading of whether
/// that end is an interruption.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Leaning {
    /// How squarely the steepest face standing OVER the run's own continuation crosses it, as the
    /// magnitude of its normal's agreement with the run's direction. Zero where none does.
    pub closes_over_the_run: f64,
    /// Whether any face here leans the way the run leaves instead.
    pub falls_away_from_the_run: bool,
}

/// C# `BevelInterruptedCrease.Interruption`: what one end's reading came to - whether it is an
/// interruption to run on through, and how far along the run the crease picks up again on the
/// far side of it.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Interruption {
    /// How far past this end the selection's own crease resumes. Zero where it does not, and the
    /// end runs on exactly where this is positive.
    pub gap: f64,
}
