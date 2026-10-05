//! The wall nobody picked, as a MIXED CORNER sees it ([`SurfaceMixedCornerWall`], with its
//! [`SurfaceMixedCornerWallFacet`]s). Ports the records of
//! `Bevel/Surface/SurfaceMixedCornerWall.cs`; its reading, its turn bookkeeping and the strip
//! under the blend land with the mixed-corner package's methods.

use std::rc::Rc;

use vector_math::Vector3;

use super::SurfaceMixedCorner;

/// C# `SurfaceMixedCornerWall.Facet` (a private readonly struct): one facet of the wall - where it
/// cuts the cap's plane, and the plane it stands in.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct SurfaceMixedCornerWallFacet {
    /// The end nearer the picked edge.
    pub from: Vector3,
    /// The far end, which is where the next facet starts.
    pub to: Vector3,
    /// The facet's outward normal.
    pub normal: Vector3,
}

/// C# `SurfaceMixedCornerWall` (a sealed class): the wall nobody picked, as a mixed corner sees
/// it: the run of facets it is made of, each one a straight segment in the cap face's own plane with
/// the plane it stands in. Held through `Rc`, because the rolled face and the unpicked face are
/// often the one wall ([`SurfaceMixedCornerFaces`](super::SurfaceMixedCornerFaces)).
///
/// **Why the cap's plane.** Everything the corner draws turns about the bead's axis, and that axis
/// is square to the cap. So the whole construction is flat: the outer rail is an arc about the
/// point the axis meets the cap, the ball centres are a radius under it, and the wall - wherever it
/// runs along the axis, which is checked here - shows in that plane as the polyline its facets
/// cut. Read once, every question the footprint asks is a ray or a segment in one plane.
///
/// **A flat wall is one facet.** Where nothing was rolled the plan carries no section, and this
/// reads the wall as the single segment from the corner vertex to the corner's cap corner in the
/// wall's own plane - which is the very square the flat construction always took, so an L prism
/// gets the same points from the general statement as it did from the special one.
#[derive(Debug)]
pub struct SurfaceMixedCornerWall {
    /// The corner this wall was read for.
    pub corner: Rc<SurfaceMixedCorner>,
    /// The facets of the wall, from the picked edge outward.
    pub facets: Vec<SurfaceMixedCornerWallFacet>,
    /// The bead's own foot on THIS face, in the cap's plane - the far end of the strip under the
    /// blend.
    pub foot: Vector3,
    /// Where each facet's own stretch of the wall begins, as a TURN about the axis.
    pub entry_turn: Vec<f64>,
    /// Where each facet's own stretch of the wall ends, as a TURN about the axis. A wall read as
    /// ONE PLANE has no far end, so its last facet's exit is `+inf`: the plane itself goes on to
    /// trim the whole runout.
    pub exit_turn: Vec<f64>,
    /// Which facet the bead's own profile is tangent to - `k`, counted from the picked edge.
    pub touched_facet: i32,
}
