//! The mirror of `SurfaceEndGivesUpTheSection` (`Bevel/Surface/SurfaceEndGainsTheSection.cs`, a
//! static class): a CONVEX end whose face beyond the corner GAINS the section rather than giving it
//! up, because the corner is a REENTRANT vertex of that face and the section stands in the bite
//! beside it rather than in the face at all. Only its nested reading lands here
//! ([`SurfaceEndGainsTheSectionReading`]); the reading's measure and the cap land with the end-cut
//! package.

/// C# `SurfaceEndGainsTheSection.Reading`: which of the two the end face is - it holds some of the
/// section and is CUT, or it holds none of it and GAINS it.
///
/// Read as the area of the section the face covers, and not the corner's angle: a reentrant vertex
/// is one the face turns through more than half a turn at, which no one facet can be asked about. A
/// face that covers NONE of the section is one the section stands beside, and is gained; a face
/// covering only part of it is the ordinary case and not a fault. And covered means covered OF THE
/// SECTION, not of the wedge, whose two sides are infinite half spaces.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum SurfaceEndGainsTheSectionReading {
    /// The section stands in the end face, which gives it up - `SurfaceEndCut`'s own path.
    #[default]
    TheFaceHoldsIt = 0,

    /// The corner is reentrant and the section stands beside the face, which gains it as a cap.
    TheFaceIsBesideIt,
}
