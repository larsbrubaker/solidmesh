//! The END RAY's bound ([`EndRayBound`]): an open chain's line carried on past an end whose end
//! face is oblique to the chain. The point carried on along the ray is here
//! ([`super::ChainField::point_on_the_ray`]).
//!
//! The plain field clamps its parameter at an open chain's end, and the clamp is the end vertex's
//! ball, so a rail that runs past the end vertex curls round it on its side face - the rounded
//! run-out decision 3 forbids. Where the end face is square to the chain the rail meets the end
//! face before the ball begins, so nothing shows; where it leans, one side runs on past the end
//! vertex and curls. With an end ray the end segment also answers for its line run on past the
//! end, read only on the far side of the plane through the end vertex square to the chain: there
//! the ray is never farther than the vertex, and on that plane the two agree, so the field stays
//! exactly 1-Lipschitz and continuous, and every rail runs straight on.
//!
//! The parameter runs on with the ray - beyond the chain's length past the last point, below zero
//! before the first - continuous with the end. The setback is the end point's. The reach is the
//! chain's box diagonal plus four setbacks: enough for a rail at the setback to meet an end face
//! leaning up to ~75 degrees. Only ends named in `ChainField::end_rays` carry one, so a field
//! built without them is bit-identical.

use vector_math::Vector3;

/// How far an end ray must run - until every point of the round's row at the end
/// vertex, carried along the chain, has met the end face's plane.
///
/// [`EndRayBound::same_sense`]: the end face folds the chain's own way at the side creases, so the
/// fill trims it to the round's section (`EndPlaneTrim`) rather than growing it.
/// [`EndRayBound::crease_room`]: the largest size a square crease end's section stays on its crease
/// (`square_crease_end_clip`). [`EndRayBound::stopped`]: a stopped end's clip, and
/// [`EndRayBound::plane_tilt`] how far its stored plane may lean off the true one. `Default` is
/// written out because not every default is zero (`crease_room` is +infinity: no bound).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EndRayBound {
    /// How far the ray runs, as a multiple of the end's setback (the `EndPlaneTrim.FarthestOut`
    /// figure over the plane's slant, read with the fillet's row, which reaches at least as far as
    /// a chamfer's chord).
    pub reach_per_setback: f64,
    /// The end face's plane through the end vertex, faced on along the ray.
    pub plane_normal: Vector3,
    /// True when the end face folds the chain's own way at the side creases (default false).
    pub same_sense: bool,
    /// The largest size a square crease end's section stays on its crease (default +infinity).
    pub crease_room: f64,
    /// True when this is a STOPPED end's clip (`stopped_end_clip.rs`, default false): no ray, a
    /// plane that does not scale with the size.
    pub stopped: bool,
    /// How far a STOPPED end's plane normal may lean off the true mitre, in radians, from the
    /// float storage of the three vertices it is read from (default 0): each chord's unit
    /// direction leans at most sqrt(3) float steps (its two ends rounded half a step on each
    /// axis) over the chord's length, and the normal, their bisector, at most the two leans over
    /// the bisector's length. A seam in the true plane stands off the stored one by this times its
    /// distance from the end vertex ([`super::ChainField::on_a_stopped_plane`]).
    pub plane_tilt: f64,
}

impl Default for EndRayBound {
    fn default() -> Self {
        EndRayBound {
            reach_per_setback: 0.0,
            plane_normal: Vector3::default(),
            same_sense: false,
            crease_room: f64::INFINITY,
            stopped: false,
            plane_tilt: 0.0,
        }
    }
}

impl EndRayBound {
    /// Whether this is a square crease end's bound (same sense, with its crease room), whose plane
    /// holds the end face and is cut into the mesh as a known cut (`known_cut_trace.rs`).
    pub fn holds_its_end_face(&self) -> bool {
        self.same_sense && self.crease_room != f64::INFINITY
    }
}

impl super::ChainField {
    /// The chain's point at `parameter`, carried on along the END
    /// RAY past an end that has one; clamped to the chain as [`super::ChainField::point_at`] is
    /// past an end that has none. Returns the parameter the point stands at (the clamped one where
    /// there is no ray) and the point.
    pub fn point_on_the_ray(&self, parameter: f64) -> (f64, Vector3) {
        let n = self.points.len();
        if parameter < 0.0 && self.end_rays.0 {
            let point =
                self.points[0] + ((self.points[0] - self.points[1]).get_normal() * -parameter);
            return (parameter, point);
        }

        if parameter > self.length && self.end_rays.1 {
            let point = self.points[n - 1]
                + ((self.points[n - 1] - self.points[n - 2]).get_normal()
                    * (parameter - self.length));
            return (parameter, point);
        }

        let clamped = parameter.clamp(0.0, self.length);
        (clamped, self.point_at(clamped))
    }
}
