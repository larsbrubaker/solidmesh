//! One chain edge as the roll across a curved wall read it ([`SurfaceRolledEdge`]). Ports the
//! record of `Bevel/Surface/SurfaceRolledEdge.cs`; its `AsPlan` - the reading turned into a
//! [`SurfaceEdgePlan`] - lands with its methods.
//!
//! [`SurfaceEdgePlan`]: super::SurfaceEdgePlan

use std::collections::HashSet;
use std::rc::Rc;

use vector_math::Vector3;

use super::{SurfaceLine, SurfaceWallSection};

/// C# `SurfaceRolledEdge` (a readonly struct): one chain edge as `SurfaceRollAcrossTheWall` read
/// it - which side was the wall, which facet of it the profile touches, and the lines and setbacks
/// that follow - the whole of what a rolled edge tells a plan that the unrolled construction would
/// have said differently.
#[derive(Debug, Clone, PartialEq)]
pub struct SurfaceRolledEdge {
    /// Whether the curved side is the left of the chain's walk.
    pub wall_on_the_left: bool,
    /// Which facet of the section walk the profile touches, counted from the chain edge. Zero is
    /// the facet the edge itself lies in, where the roll reduces to the unrolled construction.
    pub facet: i32,
    /// How far the bevel reaches into the wall, measured ALONG the surface from the edge.
    pub wall_setback: f64,
    /// How far it reaches into the flat face, measured in that face from the edge.
    pub flat_setback: f64,
    /// The in-facet direction away from the edge in the facet that is touched.
    pub wall_inward: Vector3,
    /// The line in that facet the wall is walked back to.
    pub wall_tangent_line: SurfaceLine,
    /// The line in the flat face it is walked back to.
    pub flat_tangent_line: SurfaceLine,
    /// Whether there is a [`SurfaceRolledEdge::center_line`] - true for a round, false for a
    /// chamfer.
    pub has_center_line: bool,
    /// The line the rolling ball's centre runs along: a radius from the flat plane and from
    /// plane(k).
    pub center_line: SurfaceLine,
    /// How far that line stands off the chain edge's own line.
    pub center_distance: f64,
    /// The facets of the wall the cut may take material off - the stretch of surface the sections
    /// paced out, and nothing beyond it (`SurfaceRollAcrossTheWall`). Handed on, shared, to the
    /// plan's band on the wall's side.
    pub band: Rc<HashSet<i32>>,
    /// The whole section walk across the wall, which is more than this edge's own bevel needs: a
    /// mixed corner at either end of it runs off the end of the bead and takes its footprint off
    /// the facets past `k` as well ([`SurfaceWallSection`]).
    pub section: Rc<SurfaceWallSection>,
}
