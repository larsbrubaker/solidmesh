//! One chain edge as the roll across a curved wall read it ([`SurfaceRolledEdge`]). Ports
//! `Bevel/Surface/SurfaceRolledEdge.cs`: the record and its `AsPlan`, the reading turned into a
//! [`SurfaceEdgePlan`].
//!
//! [`SurfaceEdgePlan`]: super::SurfaceEdgePlan

use std::collections::HashSet;
use std::rc::Rc;

use vector_math::Vector3;

use std::sync::Arc;

use super::{SurfaceEdgePlan, SurfaceLine, SurfaceWallSection};
use crate::topology::{ChainEdge, FaceGroup};

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

impl SurfaceRolledEdge {
    /// C#'s constructor.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        wall_on_the_left: bool,
        facet: i32,
        wall_setback: f64,
        flat_setback: f64,
        wall_inward: Vector3,
        wall_tangent_line: SurfaceLine,
        flat_tangent_line: SurfaceLine,
        has_center_line: bool,
        center_line: SurfaceLine,
        center_distance: f64,
        band: Rc<HashSet<i32>>,
        section: Rc<SurfaceWallSection>,
    ) -> Self {
        SurfaceRolledEdge {
            wall_on_the_left,
            facet,
            wall_setback,
            flat_setback,
            wall_inward,
            wall_tangent_line,
            flat_tangent_line,
            has_center_line,
            center_line,
            center_distance,
            band,
            section,
        }
    }

    /// This reading as the plan for its edge, with the flat side's own numbers left exactly as
    /// the unrolled construction drew them apart from its setback and its foot. `left_inward` and
    /// `right_inward` are the faces' in-face directions away from the edge as the plan found them;
    /// `wedge_angle_degrees` is the edge's own dihedral wedge, which is what a room check reads.
    pub fn as_plan(
        &self,
        chain_edge: ChainEdge,
        left_region: Arc<FaceGroup>,
        right_region: Arc<FaceGroup>,
        wedge_angle_degrees: f64,
        left_inward: Vector3,
        right_inward: Vector3,
    ) -> SurfaceEdgePlan {
        let left = self.wall_on_the_left;

        SurfaceEdgePlan::new(
            chain_edge,
            left_region,
            right_region,
            wedge_angle_degrees,
            if left {
                self.wall_setback
            } else {
                self.flat_setback
            },
            if left {
                self.flat_setback
            } else {
                self.wall_setback
            },
            if left { self.wall_inward } else { left_inward },
            if left { right_inward } else { self.wall_inward },
            left_inward,
            right_inward,
            if left {
                self.wall_tangent_line
            } else {
                self.flat_tangent_line
            },
            if left {
                self.flat_tangent_line
            } else {
                self.wall_tangent_line
            },
            self.has_center_line,
            self.center_line,
            self.center_distance,
            if left { Some(self.band.clone()) } else { None },
            if left { None } else { Some(self.band.clone()) },
            Some(self.section.clone()),
        )
    }
}
