//! The walk across a curved wall a rolled edge was planned from ([`SurfaceWallSection`]). Ports
//! `Bevel/Surface/SurfaceWallSection.cs`; it is carried on a [`SurfaceEdgePlan`] and read by the
//! mixed corner, so the corner stays pure - it reads plans and asks the mesh nothing.
//!
//! [`SurfaceEdgePlan`]: super::SurfaceEdgePlan

use vector_math::Vector3;

use super::{SurfaceLine, SurfaceRunAcrossSectionStep};

/// C# `SurfaceWallSection` (a sealed class): the WALK across a curved wall that a rolled edge was
/// planned from - every facet the section crossed, in order from the chain edge outward, with
/// where the section entered and left each of them.
///
/// `SurfaceRollAcrossTheWall` takes this walk to find the facet `k` its profile is tangent to and
/// then keeps only that facet's answers. What is past `k` matters too: the mixed corner's blend
/// runs off the end of the bead's rim and its footprint spreads across the facets beyond it
/// (`SurfaceMixedCornerRunOut`), and its footprint BEFORE `k` is a band across the facets the bead
/// already crossed rather than one square in one plane. So the whole walk is carried on the plan,
/// which is what lets the corner stay PURE - it reads plans and asks the mesh nothing - while still
/// knowing a wall that is not one plane.
///
/// The section is taken square to the chain edge at the MIDDLE of it (`SurfaceRunAcross`'s own
/// choice, away from both corners), so its points are that one section's. A facet's PLANE is what
/// the corner reads off them, which is the same plane anywhere along the edge exactly while the
/// facet runs along it - so the corner checks that against [`SurfaceWallSection::along`] rather
/// than assuming it. Shared by the plan and everything that copies it, hence held through `Rc`.
#[derive(Debug, Clone, PartialEq)]
pub struct SurfaceWallSection {
    /// The chain edge's own direction, which the section plane is square to.
    pub along: Vector3,
    /// Which facet of the walk the profile is tangent to - `k`, counted from the chain edge.
    pub touched_facet: i32,
    /// The facets the section crossed, first to last, the first being the chain edge's own.
    pub facets: Vec<SurfaceRunAcrossSectionStep>,
    /// Whether the profile rests ON THE CREASE at the near end of
    /// [`SurfaceWallSection::touched_facet`] - tangent to neither facet's plane, touching the wall
    /// along the line the two of them share (`SurfaceRollAcrossTheWall`).
    ///
    /// Carried here rather than re-derived, because everything downstream of the roll assumes
    /// tangency to a facet PLANE and has to be told otherwise: the corner's cut edge stands no chord
    /// exactly `2r` from the bead's axis (`SurfaceMixedCornerCutEdge`) and the point the two
    /// cylinders touch at is on this line rather than on the facet the cut stands in
    /// (`SurfaceMixedCorner`). Asking the foot whether it lands on a facet's end would be a second
    /// reading of the roll's own quadratic, which is the one thing the corner never does.
    pub rests_on_the_crease: bool,
    /// The crease the profile rests on where it does: the vertex the two facets share, as a line
    /// along the wall's ruling - which is the line the bead touches the wall along, and so the line
    /// the corner's own tangency stands on. Meaningless where
    /// [`SurfaceWallSection::rests_on_the_crease`] is false.
    pub crease_line: SurfaceLine,
}

impl SurfaceWallSection {
    /// C#'s constructor.
    pub fn new(
        along: Vector3,
        touched_facet: i32,
        facets: Vec<SurfaceRunAcrossSectionStep>,
        rests_on_the_crease: bool,
        crease_line: SurfaceLine,
    ) -> Self {
        SurfaceWallSection {
            along,
            touched_facet,
            facets,
            rests_on_the_crease,
            crease_line,
        }
    }
}
