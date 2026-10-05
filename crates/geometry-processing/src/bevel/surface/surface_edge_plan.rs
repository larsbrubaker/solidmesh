//! What the surface construction has to know about ONE edge of a chain ([`SurfaceEdgePlan`]).
//! Ports the record of `Bevel/Surface/SurfaceEdgePlan.cs`, its constructor and its
//! read-throughs (convexity, dihedral, the region keys, the two ends).

use std::collections::HashSet;
use std::rc::Rc;
use std::sync::Arc;

use vector_math::Vector3;

use super::{SurfaceBevelPlan, SurfaceLine, SurfaceWallSection};
use crate::topology::{ChainEdge, EdgeConvexity, FaceGroup};

/// C# `SurfaceEdgePlan` (a readonly struct): which two face regions meet at one chain edge, how
/// far back into each of them the bevel reaches, the two lines it reaches back TO, and - for a
/// round - the line the rolling ball's center runs along.
///
/// This is the 3D reading of the 2D bevel's corner: `PathBevelArcs` finds a center a radius from
/// both runs and drops the feet of that center onto them, and here the center is a LINE parallel to
/// the edge and the feet are the two tangent lines. The formulas are the 2D ones with the corner's
/// interior angle read as the dihedral wedge, which is why a chamfer (no center, setback = size)
/// and a fillet (setback = r/tan(wedge/2), center at r/sin(wedge/2)) are the same two numbers here
/// as there.
///
/// Convex and concave edges are NOT two cases. The wedge the profile is drawn in is the material
/// wedge on a convex edge and the void wedge on a concave one
/// (`BevelEdgeToolBuilder.ProfileWedgeAngleDegrees`, `180 - |turn|` on both), and with that one
/// substitution every line below comes out right without a branch: the bisector points into the
/// material one way round and into the void the other, which is exactly the difference between
/// cutting the strip in and bulging it out. The convexity is recorded (on the edge) so a later
/// phase can say which it was; nothing here reads it.
#[derive(Debug, Clone, PartialEq)]
pub struct SurfaceEdgePlan {
    /// The chain edge this plans, oriented the way the chain walks it.
    pub edge: ChainEdge,
    /// The face region on the left of the walk direction, seen from outside the solid - the region
    /// [`SurfaceEdgePlan::left_tangent_line`] lies in and that the walkback cuts. Shared with the
    /// region cache, as every region is.
    pub left_region: Arc<FaceGroup>,
    /// The face region on the right of the walk direction.
    pub right_region: Arc<FaceGroup>,
    /// The wedge the profile is drawn in, in degrees - the dihedral on a convex edge and the void
    /// wedge on a concave one. It is also the angle between the two in-face inward directions,
    /// which is what makes the setback and center formulas blind to convexity.
    pub wedge_angle_degrees: f64,
    /// How far back into the LEFT face the bevel reaches, measured in that face from the edge - and
    /// on a face rolled along, measured ALONG its surface (`SurfaceRollAcrossTheWall`).
    ///
    /// Per side, because the two sides no longer always agree. Between two planes the profile is
    /// inscribed symmetrically in the wedge and both faces give up the same width; rolled along a
    /// polygonal wall the circle touches the wall at a facet the edge is not on, so the wedge it is
    /// inscribed in has its apex somewhere off the edge and the two setbacks are different numbers.
    pub left_setback: f64,
    /// The same for the right face.
    pub right_setback: f64,
    /// The facets of the left region the cut may take material off, or `None` - which is nearly
    /// always - to let the band spread through whatever material it can reach. Only a roll along a
    /// curved wall names them (`SurfaceRollAcrossTheWall` says why it has to). Shared with the cut
    /// the walkback makes from this plan.
    pub left_band: Option<Rc<HashSet<i32>>>,
    /// The same for the right region.
    pub right_band: Option<Rc<HashSet<i32>>>,
    /// The direction that lies IN the left face, square to the edge, pointing away from the edge
    /// into that face - the side of the tangent line the face is KEPT on. Seen from outside the
    /// solid the left face's body is to the left of the walk, which is its normal crossed with the
    /// direction.
    ///
    /// Worked out once here, where the tangent lines are drawn from it, and read by the walkback
    /// rather than derived a second time: the same formula written twice is a sign flip waiting to
    /// happen, and a sign flip here keeps the sliver and throws the face away.
    pub left_inward: Vector3,
    /// The same for the right face, which is the direction crossed with its normal.
    pub right_inward: Vector3,
    /// The left face's in-face inward direction AT THE EDGE - facet 0's - which is the same vector
    /// as [`SurfaceEdgePlan::left_inward`] wherever nothing was rolled, and the direction the roll
    /// started from where something was.
    ///
    /// Two directions because two readings want different ones. The CUT is made in the plane of the
    /// facet the profile touches, so it takes facet k's; a reading that measures ACROSS the region
    /// from the edge - `SurfaceRoom`'s walk, and the pairing of two cuts that face each other - has
    /// to start where the edge is, which is facet 0's. Handing the walk facet k's direction would
    /// start it part way round the wall and pointing the wrong way off it.
    pub left_inward_at_the_edge: Vector3,
    /// The same for the right face.
    pub right_inward_at_the_edge: Vector3,
    /// The line in the left face the walkback cuts to: the edge line moved into that face by the
    /// setback.
    pub left_tangent_line: SurfaceLine,
    /// The same line in the right face.
    pub right_tangent_line: SurfaceLine,
    /// Whether there is a [`SurfaceEdgePlan::center_line`] - true for a round, false for a
    /// chamfer, whose straight profile has no center to swing about.
    pub has_center_line: bool,
    /// The line the rolling ball's center runs along: parallel to the edge, at the radius from both
    /// face planes, with the two tangent lines as its perpendicular feet. Only meaningful when
    /// [`SurfaceEdgePlan::has_center_line`].
    pub center_line: SurfaceLine,
    /// How far the center line stands from the edge along the wedge bisector,
    /// `radius / sin(wedge / 2)`. Zero for a chamfer.
    pub center_distance: f64,
    /// The walk across the curved wall this edge was ROLLED along, or `None` where neither side
    /// curves ([`SurfaceWallSection`]). Which side is the wall is the band's own answer: it is the
    /// side [`SurfaceEdgePlan::left_band`] or [`SurfaceEdgePlan::right_band`] is not `None` on.
    pub wall_section: Option<Rc<SurfaceWallSection>>,
}

impl SurfaceEdgePlan {
    /// C#'s constructor, every field named in C#'s order; `wall_section` is C#'s optional last
    /// argument (pass `None` for its default).
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        edge: ChainEdge,
        left_region: Arc<FaceGroup>,
        right_region: Arc<FaceGroup>,
        wedge_angle_degrees: f64,
        left_setback: f64,
        right_setback: f64,
        left_inward: Vector3,
        right_inward: Vector3,
        left_inward_at_the_edge: Vector3,
        right_inward_at_the_edge: Vector3,
        left_tangent_line: SurfaceLine,
        right_tangent_line: SurfaceLine,
        has_center_line: bool,
        center_line: SurfaceLine,
        center_distance: f64,
        left_band: Option<Rc<HashSet<i32>>>,
        right_band: Option<Rc<HashSet<i32>>>,
        wall_section: Option<Rc<SurfaceWallSection>>,
    ) -> Self {
        SurfaceEdgePlan {
            edge,
            left_region,
            right_region,
            wedge_angle_degrees,
            left_setback,
            right_setback,
            left_band,
            right_band,
            left_inward,
            right_inward,
            left_inward_at_the_edge,
            right_inward_at_the_edge,
            left_tangent_line,
            right_tangent_line,
            has_center_line,
            center_line,
            center_distance,
            wall_section,
        }
    }

    /// The angle between the two faces measured through the material, in degrees: 90 on a box
    /// edge.
    pub fn dihedral_angle_degrees(&self) -> f64 {
        self.edge.edge.dihedral_angle_degrees()
    }

    /// Which way the solid folds here. Recorded, not branched on: the construction is the same
    /// either way and only the side the strip bulges differs.
    pub fn convexity(&self) -> EdgeConvexity {
        self.edge.edge.convexity
    }

    /// What names the left face's region, whichever of its faces it was flooded from.
    pub fn left_region_key(&self) -> i32 {
        SurfaceBevelPlan::key_of(&self.left_region)
    }

    /// The same for the right face's region.
    pub fn right_region_key(&self) -> i32 {
        SurfaceBevelPlan::key_of(&self.right_region)
    }

    /// Where the chain enters this edge.
    pub fn from(&self) -> Vector3 {
        self.edge.from()
    }

    /// Where the chain leaves it.
    pub fn to(&self) -> Vector3 {
        self.edge.to()
    }
}
