//! Where a strip leaves the PART at one of its ends ([`SurfaceStripEndsOnTheHull`]): which of the
//! part's own hull planes ([`SurfaceStripEndsOnTheHullSupportingPlane`]) each rail goes out
//! through, at what parameter along the edge, and the one point the end turns at where the rails
//! change from one plane to the other. Ports the fields of
//! `Bevel/Surface/SurfaceStripEndsOnTheHull.cs`; the reading (`TryRead`) and the closing
//! (`TryClose`) land with the strip-ends package as further `impl` blocks.

use vector_math::Vector3;

/// C# `SurfaceStripEndsOnTheHull`: where a strip leaves the part at one of its ends.
///
/// A strip whose band ran off the face it landed on (`SurfaceBandRunsOffOverAStretch`) has no face
/// left to end on at that end of the edge: the bead carries on past the last of the wall and out of
/// the solid altogether. What ends it there is the part's own OUTSIDE, so the end is read against
/// the planes the part is bounded by rather than against any face the band lands on.
///
/// **The furthest cut back wins.** Walking toward the end, a rail may cross several of those
/// planes; the one that really ends it is the one that cuts it back FURTHEST from the end, because
/// past that crossing the rail is outside the part whatever the others say. Where two planes cut
/// one rail back by the same amount the tie is settled by which plane its NEIGHBOURS leave through,
/// because a run of rails leaving through one plane is what the end cap is cut from.
///
/// **Two planes at most, in two runs.** The cap this end leaves is one piece per plane, so an end
/// over three planes, or one whose runs interleave, is named rather than built
/// (`SurfaceHullEndOnThreePlanes`, `SurfaceHullEndCrossesTheCorner`). Where there ARE two
/// contiguous runs, the end turns at one point on the CHORD between the two rails either side of
/// the change, because the band is chorded and the cap has to be bounded by the surface that is
/// really built.
/// **And the apex is looked up** in the pool rather than minted, so that the cap, the faces beside
/// it and the part's own triangles all name one vertex.
///
/// **A rung, not a bug.** The hull is read off the mesh the WALKBACK left, so a plane that was not
/// supporting on the part the user picked can become supporting once a band has been cut away from
/// in front of it. Neither fixture reaches it, and the honest reading is of the ORIGINAL mesh,
/// handed down from the caller. Named here so the next rung has it.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct SurfaceStripEndsOnTheHull {
    /// Which end of the edge was read.
    pub at_the_start: bool,
    /// Where each rail leaves the part, as a parameter along the edge from its start.
    pub exit_at: Vec<f64>,
    /// The plane each rail leaves through.
    pub plane_of: Vec<SurfaceStripEndsOnTheHullSupportingPlane>,
    /// The rail the end changes plane after - so rails up to and including it leave through one
    /// plane and the rest through the other - or -1 where every rail leaves through the same one.
    pub changes_after: i32,
    /// Where the end turns from the one plane to the other, on the chord between the two rails
    /// either side of [`SurfaceStripEndsOnTheHull::changes_after`]. Meaningless where that is -1.
    pub between: Vector3,
    /// The part's own vertex where the two planes meet on the knife edge the band ran off, or -1
    /// where no knife edge was handed over to look for one on.
    pub apex: i32,
}

/// C# `SurfaceStripEndsOnTheHull.SupportingPlane` (a readonly struct): one plane the part is
/// bounded by, with every one of its vertices on the inner side - the six faces of the cube the
/// recess is cut into, and nothing of the recess itself. That is the definition a hull has, read on
/// the mesh rather than built as one, and it is what makes "where does this rail leave the part" a
/// question with a closed-form answer - a rail is a line, so each crossing is one division. The
/// zero value (no normal) is the plane that does not exist.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct SurfaceStripEndsOnTheHullSupportingPlane {
    /// A point on the plane.
    pub at: Vector3,
    /// Its outward normal, which is the part's own.
    pub normal: Vector3,
}
