//! The section a band's run-out end gives up, SPLIT by a feature standing in it
//! (`Bevel/Surface/SurfaceEndCapAroundAFeature.cs`, a static class): the upper piece the corner
//! still sees, the sliver beyond the feature's far trace, and nothing at all between them - the
//! extended feature is material there, so that stretch of the end plane is interior and carries no
//! face. Only its nested record lands here, where one piece meets the feature
//! ([`SurfaceEndCapAroundAFeatureMeeting`]); the capping lands with the standing-feature package.

/// C# `SurfaceEndCapAroundAFeature.Meeting` (a private readonly struct): where one of the two
/// pieces meets the feature - its Q on the strip, and the crease vertex standing in the end plane.
///
/// Every vertex of both pieces is one the notch or the trim already minted - the two Qs, the two
/// crease ends, the strip's own end ring - so the pieces, the extension and the notched strip close
/// on shared indices rather than on points a rounding apart.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct SurfaceEndCapAroundAFeatureMeeting {
    /// The feature's Q on the strip's end ring.
    pub q: i32,
    /// The crease vertex standing in the end plane.
    pub crease: i32,
}
