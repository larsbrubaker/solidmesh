//! The footprints a mixed corner's run-out lays on the room - the skirt on the cap it runs across
//! and the bite out of the wall it turns against: the skirt as a cut ([`try_cut`]), the wall bite
//! as a cut ([`try_wall_cut`]), and how far the run-out's arc reaches across a direction
//! ([`span_across`]).
//!
//! Every other cut `surface_room_measure::measure` makes belongs to a chain EDGE; a mixed corner's
//! run-out belongs to none, so it is read as a cut of its own: the corner is the origin, the way
//! the ball's touch point travels across the face is the inward direction, and the reach per unit
//! of size is its setback. Its ROOM is deliberately infinite - a run-out may carry on past the
//! cap's own rim; the only thing it may not do is land on face another chain has taken, which is
//! the pair rule.

use std::rc::Rc;

use solidmesh_math::{ieee_math, Vector3};

use super::{
    surface_mixed_corner_run_out, SurfaceMixedCorner, SurfaceMixedCornerWall, SurfaceRoomCut,
    SurfaceRoomFootprint,
};

/// The run-out's footprint on the cap face as a cut, told which corner laid it (`corner_key`,
/// [`SurfaceRoomCut::NO_CORNER`] where no corner is named), or `None` where this corner has no
/// readable run-out - a wall that cannot be read, or a rail that never meets it - or where the
/// skirt stands no width.
#[allow(clippy::neg_cmp_op_on_partial_ord)] // `!(x > y)` on purpose: it refuses a NaN as well.
pub fn try_cut(corner: &Rc<SurfaceMixedCorner>, corner_key: i32) -> Option<SurfaceRoomCut> {
    if !(corner.size > 0.0) {
        return None;
    }

    let wall = SurfaceMixedCornerWall::try_read(corner).ok()?;
    let meets = wall.try_where_the_rail_meets_the_wall()?;

    // Never past the flat wall's own sixty degrees: a facet answering later than that is answering
    // for a turn the run-out does not reach.
    let dies = ieee_math::min_f64(
        meets,
        corner.turn_radians() + surface_mixed_corner_run_out::clears_the_wall(),
    );

    if !(dies > corner.turn_radians()) {
        return None;
    }

    let cap_normal = corner.out_of_the_cap_face;
    let last = corner.outer_rail_point(dies);
    let mut away = last - corner.corner;

    // In the cap's own plane, which only takes the arithmetic's own drift out.
    away = away - (cap_normal * away.dot(cap_normal));

    let to_the_end = away.length();

    if !(to_the_end > 0.0) {
        return None;
    }

    let travel = away / to_the_end;
    let across = travel.cross(cap_normal).get_normal();

    // How far the skirt reaches ALONG that travel: the arc's own furthest point along it can be a
    // turn short of the one it dies at.
    let (_, reach) = span_across(corner, dies, corner.corner, travel);

    if !(reach > 0.0) {
        return None;
    }

    // How wide the skirt stands, square to the way it travels.
    let (low, high) = span_across(corner, dies, corner.corner, across);

    let region_key = if corner.cut.left_region_key() == corner.shared_region_key {
        corner.cut.right_region_key()
    } else {
        corner.cut.left_region_key()
    };

    let cut = SurfaceRoomCut {
        shape: SurfaceRoomFootprint::RunOutSkirt,
        corner_key,
        ..SurfaceRoomCut::new(
            region_key,
            corner.corner + (across * low),
            corner.corner + (across * high),
            across,
            travel,
            high - low,
            reach / corner.size,
            f64::INFINITY,
        )
    };

    (high - low > 0.0).then_some(cut)
}

/// The run-out's footprint on the WALL it turns against - the bite the scallop takes between the
/// sharp edge and the trim curve - as a cut told which corner laid it, or `None` where the wall
/// cannot be read. Read exactly: along one facet of the wall the trim point's reach is monotone in
/// the turn, so the furthest the bite gets stands at an end of the turn or at a crease between two
/// facets.
#[allow(clippy::neg_cmp_op_on_partial_ord)] // `!(x > y)` on purpose: it refuses a NaN as well.
pub fn try_wall_cut(corner: &Rc<SurfaceMixedCorner>, corner_key: i32) -> Option<SurfaceRoomCut> {
    if !(corner.size > 0.0) {
        return None;
    }

    let wall = SurfaceMixedCornerWall::try_read(corner).ok()?;
    let meets = wall.try_where_the_rail_meets_the_wall()?;

    let dies = ieee_math::min_f64(
        meets,
        corner.turn_radians() + surface_mixed_corner_run_out::clears_the_wall(),
    );

    if !(dies > corner.turn_radians()) {
        return None;
    }

    // Every turn the bite's own reach can be extreme at: the two ends, and every crease between.
    let mut asking = vec![corner.turn_radians(), dies];
    asking.extend(wall.creases_between(corner.turn_radians(), dies));

    let down = -corner.out_of_the_cap_face;

    // Along the wall, away from the corner, taken from where the bite ENDS.
    let last = surface_mixed_corner_run_out::try_scallop_point(
        corner,
        &wall,
        dies - corner.turn_radians(),
    )?;

    let mut away = last - corner.corner;

    away = away - (corner.out_of_the_wall * away.dot(corner.out_of_the_wall));
    away = away - (down * away.dot(down));

    if !(away.length() > 0.0) {
        return None;
    }

    let travel = away.get_normal();

    let mut reach = 0.0;
    let mut deepest = 0.0;

    for turn in asking {
        if turn < corner.turn_radians() || turn > dies {
            continue;
        }

        let Some(at) = surface_mixed_corner_run_out::try_scallop_point(
            corner,
            &wall,
            turn - corner.turn_radians(),
        ) else {
            continue;
        };

        reach = ieee_math::max_f64(reach, (at - corner.corner).dot(travel));
        deepest = ieee_math::max_f64(deepest, (at - corner.corner).dot(down));
    }

    if !(reach > 0.0) {
        // The bite travels the other way along the wall than the corner does.
        return None;
    }

    let region_key = if corner.bead.left_region_key() == corner.shared_region_key {
        corner.bead.right_region_key()
    } else {
        corner.bead.left_region_key()
    };

    let cut = SurfaceRoomCut {
        shape: SurfaceRoomFootprint::RunOutSkirt,
        corner_key,
        the_wall_bite: true,
        ..SurfaceRoomCut::new(
            region_key,
            corner.corner,
            corner.corner + (down * deepest),
            down,
            travel,
            deepest,
            reach / corner.size,
            f64::INFINITY,
        )
    };

    (deepest > 0.0).then_some(cut)
}

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

    let at =
        |turn: f64| centre + (along_zero * libm::cos(turn)) + (along_quarter * libm::sin(turn));

    let mut take = |turn: f64| {
        let here = at(turn);

        nearest = ieee_math::min_f64(nearest, here);
        furthest = ieee_math::max_f64(furthest, here);
    };

    take(corner.turn_radians());
    take(dies);

    let stationary = libm::atan2(along_quarter, along_zero);

    for half in -2..=2 {
        let turn = stationary + (f64::from(half) * std::f64::consts::PI);

        if turn > corner.turn_radians() && turn < dies {
            take(turn);
        }
    }

    (nearest, furthest)
}
