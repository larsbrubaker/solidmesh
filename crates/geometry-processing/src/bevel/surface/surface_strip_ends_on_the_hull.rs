//! Where a strip leaves the PART at one of its ends ([`SurfaceStripEndsOnTheHull`]): which of the
//! part's own hull planes ([`SurfaceStripEndsOnTheHullSupportingPlane`]) each rail goes out
//! through, at what parameter along the edge, and the one point the end turns at where the rails
//! change from one plane to the other. Ports the fields of
//! `Bevel/Surface/SurfaceStripEndsOnTheHull.cs` and the turning point between two planes
//! ([`SurfaceStripEndsOnTheHull::try_between`]); the reading (`TryRead`) and the closing
//! (`TryClose`) land with a later package as further `impl` blocks.

use vector_math::Vector3;

use super::SurfaceStrip;

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

/// The end's turning point between two hull planes (C# `TryBetween`, `TryThreePlanes`).
impl SurfaceStripEndsOnTheHull {
    /// C# `TryBetween`: where the end turns from one plane to the other - the line the two planes
    /// meet in, crossed with the CHORD between the two rails either side of the change. `None` where
    /// the two planes do not meet in a line at all, or where they meet the band outside that chord -
    /// which is the corner being crossed somewhere the rails do not rule. `exit_at` is C#'s unused
    /// rail parameters, kept out of the signature.
    #[allow(clippy::neg_cmp_op_on_partial_ord)]
    pub fn try_between(
        strip: &SurfaceStrip,
        plane_of: &[SurfaceStripEndsOnTheHullSupportingPlane],
        changes_after: i32,
        length: f64,
        near: f64,
    ) -> Option<Vector3> {
        // The band between two rails is a ruled quad and so a PLANE - both rails run along the same
        // direction - and the chord the section is really bounded by is where one hull plane cuts that
        // quad. So where the section turns from the one plane to the other is where all THREE meet:
        // one point, by Cramer, with nothing sampled and nothing searched for.
        let foot = strip.point_on(changes_after, 0.0);
        let across = strip.point_on(changes_after + 1, 0.0) - foot;
        let column = strip.along().cross(across);

        if !(column.length_squared() > 0.0) {
            return None;
        }

        let between = Self::try_three_planes(
            plane_of[changes_after as usize],
            plane_of[changes_after as usize + 1],
            foot,
            column,
        )?;

        // And it has to stand ON the band: between the two rails, and inside the edge's own span. A
        // corner the two planes meet at somewhere else is a corner this end does not turn at.
        let sideways = (between - foot).dot(across) / across.length_squared();
        let along = strip.param_of(between) - strip.param_of(foot);

        (sideways >= -near && sideways <= 1.0 + near && along >= -near && along <= length + near)
            .then_some(between)
    }

    /// C# `TryThreePlanes`: where three planes meet, by Cramer's rule - `None` where they do not meet
    /// at a point.
    #[allow(clippy::neg_cmp_op_on_partial_ord)]
    pub fn try_three_planes(
        a: SurfaceStripEndsOnTheHullSupportingPlane,
        b: SurfaceStripEndsOnTheHullSupportingPlane,
        point_c: Vector3,
        normal_c: Vector3,
    ) -> Option<Vector3> {
        let volume = a.normal.dot(b.normal.cross(normal_c));

        if !(volume.abs() > 1e-9) {
            return None;
        }

        Some(
            ((b.normal.cross(normal_c) * a.at.dot(a.normal))
                + (normal_c.cross(a.normal) * b.at.dot(b.normal))
                + (a.normal.cross(b.normal) * point_c.dot(normal_c)))
                / volume,
        )
    }
}
