//! A bevel ROLLED along a polygonal wall (`Bevel/Surface/SurfaceRollAcrossTheWall.cs`, a static
//! class): the profile is tangent to the flat face and to the one facet of the wall it actually
//! touches, so the centre moves with the curvature and the setback on the curved side is measured
//! ALONG the surface rather than across a chord of it. What lands here is the file's types - what a
//! chain edge's two landing surfaces turned out to be ([`SurfaceRollReading`]), which side is the
//! wall and what the roll reads of the two ([`SurfaceRollAcrossTheWallSides`]), and where one
//! facet's solve put the wall foot ([`SurfaceRollAcrossTheWallLanding`]); the roll itself lands
//! with the plan package.
//!
//! A fillet flows across a smooth mating surface staying tangent to it, and on a tessellated wall
//! "tangent to it" has to mean tangent to ONE facet: the facet k of the section walk where the
//! circle of radius `size` tangent to the flat face's plane and to plane(k) has its wall foot
//! inside facet k's own span. On a convex wall that k is unique, because the foot runs forward
//! along the section as the facets turn away and each facet claims one stretch of it. The section
//! walk is `SurfaceRunAcross`'s own - the very walk the ROOM across the same wall is measured by -
//! so a size the room allows and the facet this lands on cannot disagree about how far along the
//! surface the bevel reaches. Everything is drawn in the section plane, square to the chain edge,
//! which is why a wall whose section crosses only facet 0 comes back with exactly the unrolled
//! plan's numbers. One curved side only: where BOTH sides curve it is refused by name
//! (`SurfaceLandsOnTwoCurvedFaces`) rather than approximated, once each side has been asked whether
//! the plain ball already stands clear of it (`SurfaceWallStandsClear`).

use std::sync::Arc;

use vector_math::Vector3;

use super::SurfaceRunAcrossSectionStep;
use crate::topology::{ChainEdge, FaceGroup};

/// C# `SurfaceRollReading`: what a chain edge's two landing surfaces turned out to be, and so how
/// it was planned.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum SurfaceRollReading {
    /// Neither side curves along its section, so the plan's own two-plane construction is exact.
    #[default]
    NotCurved = 0,

    /// One side is a curved wall, and the profile was rolled along it to the facet it touches.
    Rolled,

    /// Nothing was planned, and the advisory handed back says why.
    Refused,
}

/// C# `SurfaceRollAcrossTheWall.Sides` (a readonly struct): which side of the edge is the wall, and
/// everything about the two of them the roll reads.
#[derive(Debug, Clone, PartialEq)]
pub struct SurfaceRollAcrossTheWallSides {
    /// The chain edge being rolled along.
    pub chain_edge: ChainEdge,
    /// The unit direction along the chain edge.
    pub along: Vector3,
    /// Whether the wall is the edge's LEFT face (the flat face is then the right).
    pub wall_on_the_left: bool,
    /// The facets the middle section crossed, which the roll walks.
    pub steps: Vec<SurfaceRunAcrossSectionStep>,
    /// The wall's region, shared with the cache it was flooded into.
    pub wall_region: Arc<FaceGroup>,
    /// The wall's inward direction at the chain edge, square to the edge in facet 0's plane.
    pub wall_inward: Vector3,
    /// The flat face's outward normal.
    pub flat_normal: Vector3,
    /// The flat face's inward direction, square to the edge and in its plane.
    pub flat_inward: Vector3,
}

/// C# `SurfaceRollAcrossTheWall.Landing` (private): where one facet's own solve put the profile's
/// wall foot, against that facet's two ends.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum SurfaceRollAcrossTheWallLanding {
    /// There is no circle to place at all - the two traces are parallel, or the flat foot is behind
    /// the edge.
    #[default]
    Nowhere = 0,

    /// Inside the facet, which is the facet that takes the roll.
    OnTheFacet,

    /// Short of its near end, so the profile touches the wall before this facet begins.
    BeforeIt,

    /// Past its far end, so the profile touches the wall after this facet ends.
    PastIt,
}
