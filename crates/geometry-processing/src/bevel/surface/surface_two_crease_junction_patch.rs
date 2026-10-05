//! Closes a TWO-crease junction - one cut and two beads - with the one sweep of the canal between
//! the two beads' own end rings (`Bevel/Surface/SurfaceTwoCreaseJunctionPatch.cs`, a static class;
//! `SurfaceTwoCreaseJunction` reads it, `SurfaceTwoCreaseJunctionRings` lays its grid). ONE sweep,
//! not two half patches: its columns run from the upper bead's end ring round to the lower's, the
//! cut's ring is the last row, and every seam of it is a curve somebody else already minted. Only
//! its nested record lands here, one strip's end ring ([`SurfaceTwoCreaseJunctionPatchEndRing`]);
//! the patch lands with the two-crease junction package.

/// C# `SurfaceTwoCreaseJunctionPatch.EndRing` (a private readonly struct): one strip's end ring at
/// the corner, in BOTH of the orders it is wanted in.
///
/// The two are kept apart because they are answers to different questions and the answer is not
/// always the same. The GRID wants every ring running the way it runs - the stations from the first
/// arm's tangency round, the rows from the wall out - and turns a strip's rails over to get it. The
/// STRIP wants its own end ring back in ITS rail order, because that is the order its seams are in
/// and the stitch checks the two rail for rail. Handing the grid's order back to the strip sews it
/// to the seams reversed, which is a strip whose wall side is stitched to its shared face's tangent
/// run - and it only shows where the turning-over actually turns something, which is why a crease
/// chorded beside the corner found it and the whole crease did not.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct SurfaceTwoCreaseJunctionPatchEndRing {
    /// The ring's rails in the STRIP's own order, as vertices of the build's own pool.
    pub ring: Vec<i32>,
    /// Where along its own edge each rail of it meets the plane it was cut on, in that same order.
    pub meets_at: Vec<f64>,
    /// The same rails in the GRID's order.
    pub as_the_grid_wants_it: Vec<i32>,
}
