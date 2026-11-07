//! Where one free end of an open swept run has to stop: a plane through the corner vertex that
//! the tool must not cross, or nothing at all, which means run past the end as phase 2 did.
//!
//! The normal is oriented the way the sweep LEAVES this end - out past the last vertex - so the
//! tool is the half space `normal . x <= distance` and both ends read the same way whichever end
//! they are.
//!
//! It sits apart from `BevelChainEnds`, which is what DECIDES an end, because it is read far more
//! widely than that decision is: the sweep places rings with it, `BevelCapClip` turns one into a
//! half space, `BevelMixedBlends` publishes one as a handover. A plane and a slide are all of it.

use vector_math::Vector3;

/// How one free end of a swept run stops. `Default` is an uncapped end.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct BevelEndCap {
    /// The points a corner PATCH has already put on this plane, which the sweep's own ring takes
    /// in place of its arc - see `BevelSeamRing` for why the seam is carried rather than computed
    /// twice. `None` on every end that is not handing over to a patch, which is most of them.
    pub seam_points: Option<Vec<Vector3>>,

    /// The outward unit normal of the capping plane, pointing the way the sweep leaves this end.
    /// Zero when this end is not capped at all.
    pub plane_normal: Vector3,

    /// The plane's offset: the plane is every point whose dot with `plane_normal` is this.
    pub plane_distance: f64,

    /// Whether this end drives THROUGH a corner patch rather than stopping on the face that
    /// bounds it - see `BevelCornerSeam`. The sweep reads it to taper the tool's two outer walls
    /// over the overshoot, which is what keeps two runs sharing a face plane from meeting
    /// coplanarly there.
    pub drives_through: bool,

    /// The plane the overshoot's own cap sits on, where `drives_through`. The run itself still
    /// sweeps to `plane_distance` - the ball centre's plane, unchanged - and the sweep adds ONE
    /// more ring out here, so the span between them is the only part that changes shape.
    pub drive_through_distance: f64,
}

impl BevelEndCap {
    /// A cap on a plane, with the optional parts at their defaults (no drive-through, no seam
    /// points).
    pub fn new(plane_normal: Vector3, plane_distance: f64) -> Self {
        Self::with_options(plane_normal, plane_distance, false, 0.0, None)
    }

    /// A cap with every part given.
    pub fn with_options(
        plane_normal: Vector3,
        plane_distance: f64,
        drives_through: bool,
        drive_through_distance: f64,
        seam_points: Option<Vec<Vector3>>,
    ) -> Self {
        Self {
            seam_points,
            plane_normal,
            plane_distance,
            drives_through,
            drive_through_distance,
        }
    }

    /// Whether this end stops on a plane rather than overshooting.
    pub fn is_capped(&self) -> bool {
        self.plane_normal.length_squared() > 0.0
    }

    /// How far along its own segment one profile point has to slide to land on the capping
    /// plane. `anchor` is the chain vertex this end's ring is built around, `offset` that profile
    /// point as an offset from the chain in part space, `direction` the segment's sweep direction
    /// which the ring slides along.
    ///
    /// The whole ring is the profile projected ALONG the segment onto the plane, an invertible
    /// linear map of the profile's own plane whenever the denominator is non zero - so the capped
    /// ring is still the same simple polygon, wound the same way. `BevelChainEnds` is what
    /// guarantees the denominator: it refuses a plane too nearly edge on to the run before one of
    /// these is ever made.
    pub fn slide_to_plane(&self, anchor: Vector3, offset: Vector3, direction: Vector3) -> f64 {
        (self.plane_distance - self.plane_normal.dot(anchor + offset))
            / self.plane_normal.dot(direction)
    }
}
