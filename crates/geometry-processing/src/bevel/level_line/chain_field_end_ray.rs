//! The END RAY's bound ([`EndRayBound`]), from `Bevel/LevelLine/ChainField.EndRay.cs`: an open
//! chain's line carried on past an end whose end face is oblique to the chain. The ray's reading
//! and the `ChainField` methods of that partial land with their method package.
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

/// C# `EndRayBound`: how far an end ray must run - until every point of the round's row at the end
/// vertex, carried along the chain, has met the end face's plane.
///
/// [`EndRayBound::same_sense`]: the end face folds the chain's own way at the side creases, so the
/// fill trims it to the round's section (`EndPlaneTrim`) rather than growing it.
/// [`EndRayBound::crease_room`]: the largest size a square crease end's section stays on its crease
/// (`SquareCreaseEndClip.SquareCreaseEndClips`). `Default` is written out because C#'s record
/// parameter defaults are not zero (`CreaseRoom = +infinity`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EndRayBound {
    /// How far the ray runs, as a multiple of the end's setback (the `EndPlaneTrim.FarthestOut`
    /// figure over the plane's slant, read with the fillet's row, which reaches at least as far as
    /// a chamfer's chord).
    pub reach_per_setback: f64,
    /// The end face's plane through the end vertex, faced on along the ray.
    pub plane_normal: Vector3,
    /// True when the end face folds the chain's own way at the side creases (C# default false).
    pub same_sense: bool,
    /// The largest size a square crease end's section stays on its crease (C# default
    /// +infinity).
    pub crease_room: f64,
}

impl Default for EndRayBound {
    fn default() -> Self {
        EndRayBound {
            reach_per_setback: 0.0,
            plane_normal: Vector3::default(),
            same_sense: false,
            crease_room: f64::INFINITY,
        }
    }
}
