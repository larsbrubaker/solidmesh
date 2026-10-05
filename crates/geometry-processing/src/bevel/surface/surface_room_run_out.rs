//! The footprints a mixed corner's run-out lays on the room - the skirt on the cap it runs across
//! and the bite out of the wall it turns against (`Bevel/Surface/SurfaceRoomRunOut.cs`, a static
//! class: free functions here). How far the run-out's arc reaches across a direction lands here
//! first; laying the footprints lands with a later package.

use vector_math::{cs_math, Vector3};

use super::SurfaceMixedCorner;

/// How far the run-out's arc reaches either side of `from` along `across`, as `(low, high)`, over
/// the turns the skirt covers (from the corner's own turn to `dies`) - exactly rather than
/// sampled: the projection of a circle's arc is `a cos(t) + b sin(t)`, whose extremes over a
/// stretch of turn are at its ends or at the one turn in it where the derivative vanishes.
pub fn span_across(
    corner: &SurfaceMixedCorner,
    dies: f64,
    from: Vector3,
    across: Vector3,
) -> (f64, f64) {
    // Seeded with the corner itself at zero: the footprint runs from there, so the span always
    // holds it however the arc leans.
    let mut nearest = 0.0;
    let mut furthest = 0.0;

    let centre = (corner.outer_rail_centre - from).dot(across);
    let along_zero = corner.towards_the_shared_face().dot(across) * corner.outer_rail_radius();
    let along_quarter = corner.towards_the_wall_square().dot(across) * corner.outer_rail_radius();

    let at = |turn: f64| centre + (along_zero * turn.cos()) + (along_quarter * turn.sin());

    let mut take = |turn: f64| {
        let here = at(turn);

        nearest = cs_math::min_f64(nearest, here);
        furthest = cs_math::max_f64(furthest, here);
    };

    take(corner.turn_radians());
    take(dies);

    let stationary = f64::atan2(along_quarter, along_zero);

    for half in -2..=2 {
        let turn = stationary + (f64::from(half) * std::f64::consts::PI);

        if turn > corner.turn_radians() && turn < dies {
            take(turn);
        }
    }

    (nearest, furthest)
}
