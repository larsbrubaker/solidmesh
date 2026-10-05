//! The corner where THREE ROUND strips meet at one vertex of the part
//! (`Bevel/Surface/SurfaceCornerPatch.cs`, a static class): each one ends on the plane through the
//! ball centre square to its own edge, and the spherical triangle between the three end rings is
//! stitched in as the patch - the rung 3 reading of `BevelCornerPatches`, built as surface and
//! shared by vertex rather than built as a solid and subtracted. Only its nested record lands here,
//! one strip arriving at the corner ([`SurfaceCornerPatchArrival`]); the patch's construction lands
//! with the corner package.

use std::rc::Rc;

use super::{SurfaceEdgePlan, SurfaceStrip};

/// C# `SurfaceCornerPatch.Arrival` (a readonly struct): one strip arriving at the corner - which
/// strip, what it was planned as, and the two vertices the walkback left where its own two tangent
/// lines ended.
#[derive(Debug, Clone)]
pub struct SurfaceCornerPatchArrival {
    /// The strip arriving.
    pub strip: Rc<SurfaceStrip>,
    /// The edge plan the strip was built from (a C# struct, so held by value).
    pub edge: SurfaceEdgePlan,
    /// The last vertex of the left face's tangent run - rail 0 of the end ring.
    pub on_the_left: i32,
    /// The same for the right face, which is the ring's last rail.
    pub on_the_right: i32,
}
