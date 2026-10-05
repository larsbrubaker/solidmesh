//! The end-face clip at the unrayed end of an open convex chain that stops on an unpicked concave
//! crease against one flat end face square to it. Ports the nested type of
//! `Bevel/LevelLine/SquareCreaseEndClip.cs` (a C# static class): the scope that turns the clips off
//! for a flow ([`SquareCreaseEndClipSquareCreaseClipsOffScope`]). The clip functions and the
//! per-flow gate land with later packages.

/// C# `SquareCreaseEndClip.SquareCreaseClipsOffScope`: turns the square-crease clips off for this
/// flow until it is dropped, which puts back what was there before - so a test can read the
/// unclipped ball end. Per async flow, as `BandGroupFields.RunOutsOn`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct SquareCreaseEndClipSquareCreaseClipsOffScope {
    /// Whether the clips were off when the scope was opened - restored when it closes.
    pub was: bool,
}
