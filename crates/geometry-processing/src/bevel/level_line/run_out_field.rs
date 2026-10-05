//! The field of one `CreaseRunOut` ([`RunOutField`]). Ports the type of
//! `Bevel/LevelLine/RunOutField.cs`; its constructors and field queries land with later packages.

use vector_math::Vector3;

/// C# `RunOutField`: the field of one `CreaseRunOut` - negative in a thin tapering wedge along the
/// unpicked crease, from the pick corner to P*, zero width at P* - the sliver the round's void ball
/// takes off both crease faces as it runs out. Read as the least of it and the pick's own field
/// (`BandGroupFields`), so the band cut carries the round on past the crease the side flood stopped
/// at.
///
/// **The shape.** The wedge is the solid of revolution about the crease's line under the wall
/// contact T(theta)'s profile: at distance s along the crease it reaches as far off the line as the
/// contact does (`Of` takes the upper concave hull of the sampled contact and P*). The hull's last
/// edge, falling to P*, is a CONE with its apex there ([`RunOutField::slope`], exact past the
/// apex); every earlier edge is one more line `off <= A + B s`; [`RunOutField::width`] caps the
/// peak; and the plane through the corner square to the crease closes it.
///
/// **What the seam needs.** Every part is convex and 1-Lipschitz (a hull line `off - A - B s` has
/// gradient sqrt(1 + B^2), since off and s change along orthogonal directions, so dividing by
/// exactly that makes it 1-Lipschitz - the normalisation is load-bearing), so the max is
/// 1-Lipschitz and CONVEX. Convex makes every query simple: the greatest over a segment or triangle
/// stands at a corner, and the least is found by golden-section search.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct RunOutField {
    /// The pick corner the wedge starts at.
    pub corner: Vector3,
    /// The unit direction along the crease, away from the corner.
    pub direction: Vector3,
    /// How far along the crease the wedge ends, P* (C# `Reach`).
    pub reach: f64,
    /// `sqrt(1 + slope^2)`: what the cone's reading is divided by to be 1-Lipschitz.
    pub norm: f64,
    /// The centre of the ball `NothingReachesBelow` tests against, halfway to P*; its radius
    /// depends on the margin.
    pub ball_centre: Vector3,
    /// The concave profile's rising and shoulder lines, each `off <= a + b s` with its own
    /// normalisation `sqrt(1 + b^2)`, before the cone's own falling generator; empty for a bare
    /// cone.
    pub lines: Vec<(f64, f64, f64)>,
    /// The taper: the wedge's radius per unit still to go to P*.
    pub slope: f64,
    /// The widest the wedge stands off the crease: the widest the wall contact does. Infinite for a
    /// bare cone.
    pub width: f64,
}
