//! One end of the band a surface cut takes off, as the half space the band is kept INSIDE: a plane
//! through one corner of the chain edge, with the side the edge itself lies on named.
//!
//! Two planes stand at a corner, and which one it is says what the chain is doing there. Where the
//! chain turns and goes on, it is the MITRE plane the two strips are cut against; where the chain
//! stops in the middle of a run, it is the plane square to the edge the run-out wall stands in.
//!
//! A cut with no ends removes its band from rim to rim of the region, which is what one picked
//! edge of a cube's top wants. A cut with ends removes it only between them - on a cylinder's cap
//! the unbounded half space would take a whole circular segment off the disc, and an interior edge
//! of a partial rim run would carry its band past the far end of the chain.

use vector_math::Vector3;

/// One end of a band: the plane through the corner and the direction that points into the band.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct SurfaceBandEnd {
    /// The corner of the chain the end plane stands at.
    pub point: Vector3,
    /// The unit direction from that corner into the chain - the side the band is on.
    pub inward: Vector3,
}

impl SurfaceBandEnd {
    /// The end through `point`, keeping the side `inward` points to.
    pub fn new(point: Vector3, inward: Vector3) -> Self {
        SurfaceBandEnd { point, inward }
    }

    /// How far a point stands on the band's side of this end; negative beyond it.
    pub fn signed_distance(&self, point: Vector3) -> f64 {
        (point - self.point).dot(self.inward)
    }
}
