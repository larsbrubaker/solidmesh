//! A mixed corner's RUN-OUT: the blend carried on past the bead's own turn until the ball's
//! touch row dies into the sharp edge the part keeps (`Bevel/Surface/SurfaceMixedCornerRunOut.cs`,
//! a static class: free functions here): its constant, the one-place test, and where a station's
//! arc meets the wall ([`try_scallop_point`], [`try_scallop_on_the_ring`]), and the turns its
//! stations stand at ([`try_turns_past_the_bead`]); the run-out's columns land with this package's
//! later methods.

use std::cmp::Ordering;

use vector_math::{cs_math, cs_sort, Vector3};

use super::surface_run_out_column_ends;
use super::{
    SurfaceMixedCorner, SurfaceMixedCornerRows, SurfaceMixedCornerWall, SurfaceRunOutCarry,
};
use crate::bevel::{BevelAdvisory, BevelAdvisoryCode};

/// How far past the bead's own turn the run-out's touch row clears the wall on a flat wall:
/// sixty degrees, `acos(1/2)`, whatever the wedge.
pub fn clears_the_wall() -> f64 {
    0.5f64.acos()
}

/// Whether two columns stand at one place: their trim points, or their rail points, within
/// `near_enough`.
pub fn is_one_place_with(
    trim: Vector3,
    rail: Vector3,
    other_trim: Vector3,
    other_rail: Vector3,
    near_enough: f64,
) -> bool {
    (other_trim - trim).length() <= near_enough || (other_rail - rail).length() <= near_enough
}

/// C# `TryScallopPoint`: where station `past`'s arc crosses the wall's plane - out of the axis by
/// the wall's own radius there, and down from the cap by as much of the ball as is left over it.
/// Where the bead's ring leans it is the station's own ring met with the wall
/// ([`try_scallop_on_the_ring`]); the square corner keeps the closed form so none of its numbers
/// moves in the last bits.
pub fn try_scallop_point(
    corner: &SurfaceMixedCorner,
    wall: &SurfaceMixedCornerWall,
    past: f64,
) -> Option<Vector3> {
    if corner.the_beads_ring_leans() {
        return try_scallop_on_the_ring(corner, wall, past);
    }

    let (out_of_the_axis, _) = wall.try_out_of_the_axis(corner.turn_radians() + past)?;

    // C#'s `Math.Pow(x, 2)`, which is the correctly rounded square, as `x * x` is.
    let left_over = (2.0 * corner.size) - out_of_the_axis;
    let under_the_cap = corner.size
        - cs_math::max_f64(0.0, (corner.size * corner.size) - (left_over * left_over)).sqrt();

    Some(
        corner.outer_rail_centre
            + (corner.out_of_the_axis(corner.turn_radians() + past) * out_of_the_axis)
            - (corner.out_of_the_cap_face * under_the_cap),
    )
}

/// C# `TryScallopOnTheRing`: the same point read as the STATION'S OWN RING met with the wall's
/// facet. The ring is `C + r(u cos psi + across sin psi)` - the ball's centre, the way to the face
/// it rides, and the arm across it the rows lay their rails in - so met with the facet's plane it
/// is `A cos psi + B sin psi = K`, and the root taken is the FIRST at or past `psi = 0`, the cap
/// contact. On a square cap it is the closed form of [`try_scallop_point`] to a nanometre.
pub fn try_scallop_on_the_ring(
    corner: &SurfaceMixedCorner,
    wall: &SurfaceMixedCornerWall,
    past: f64,
) -> Option<Vector3> {
    let turn = corner.turn_radians() + past;
    let (stands_in, facing) = wall.try_plane_at(turn)?;

    let centre = corner.spine.centre_at(turn);
    let towards_the_plate = corner.out_of_the_cap_face;
    let across = SurfaceMixedCornerRows::across_at(towards_the_plate, corner.out_of_the_axis(turn));

    let psi = SurfaceMixedCornerWall::try_the_first_root(
        corner.size * towards_the_plate.dot(facing),
        corner.size * across.dot(facing),
        (stands_in - centre).dot(facing),
        0.0,
    )?;

    Some(
        centre
            + (towards_the_plate * (corner.size * psi.cos()))
            + (across * (corner.size * psi.sin())),
    )
}

/// C#'s `double.CompareTo`: a NaN before every number, and two NaNs equal.
fn compare_turns(first: f64, second: f64) -> Ordering {
    match (first.is_nan(), second.is_nan()) {
        (true, true) => Ordering::Equal,
        (true, false) => Ordering::Less,
        (false, true) => Ordering::Greater,
        _ => first.partial_cmp(&second).unwrap_or(Ordering::Equal),
    }
}

/// C# `TryTurnsPastTheBead`: the turns past the patch's last, in order - one on every crease of
/// the wall the curve runs over, the caller's own angular `step` wherever a crease has not already
/// stationed it, and one last station EXACTLY where the ball clears the wall (the point the trim
/// curve dies at, named rather than stepped to). `near_enough` is the part's ruler; nearer the end
/// than that a station is not worth standing on. `touches_the_cap_at` is where the ball touches the
/// cap at the patch's own last turn (`None` for the ideal rail's point), and `carry` the run-out's
/// carry where the column is laid on the spine.
///
/// A step that is not positive asks for no stations at all.
#[allow(clippy::neg_cmp_op_on_partial_ord)] // C#'s `!(x > 0)` / `!(x < y)`, which a NaN passes.
pub fn try_turns_past_the_bead(
    corner: &SurfaceMixedCorner,
    wall: &SurfaceMixedCornerWall,
    step: f64,
    near_enough: f64,
    touches_the_cap_at: Option<Vector3>,
    carry: Option<&SurfaceRunOutCarry>,
) -> Result<Vec<f64>, BevelAdvisory> {
    let not_supported = || {
        BevelAdvisory::new(
            BevelAdvisoryCode::SurfaceCornerNotSupported,
            vec![corner.corner.into()],
        )
    };
    let mut turns = Vec::new();

    if !(step > 0.0) {
        return Ok(turns);
    }

    // The wall ran out before the ball ever cleared it, so nothing trims the blend and the sharp
    // edge the runout dies into is not there to die into. A corner of another shape.
    let meets = wall
        .try_where_the_rail_meets_the_wall()
        .ok_or_else(not_supported)?;

    let mut clears = meets - corner.turn_radians();

    // And no station's touch point may stand OUTSIDE the wall: where the patch's own touch row
    // stands off the ideal rail it crosses the wall's trace first. Never on the SPINE, where the
    // wall reading already met the leaning contact ellipse itself.
    if let Some(touch) = touches_the_cap_at {
        if !corner.the_beads_ring_leans() {
            if let Some(its_own) = wall.try_where_the_touch_point_meets_the_wall(touch) {
                if its_own < clears {
                    clears = its_own;
                }
            }
        }
    }

    let hair = cs_math::max_f64(
        step * 1e-6,
        surface_run_out_column_ends::too_close_to_the_end(near_enough, corner.size),
    );

    // Everything that ASKS for a station, in order of turn: every crease of the wall, every whole
    // step of the caller's own chording, and the end. The creases and the end must stand.
    let mut asking: Vec<(f64, bool)> = Vec::new();

    for crease in wall.creases_between(corner.turn_radians(), meets) {
        asking.push((crease - corner.turn_radians(), true));
    }

    let mut turn = step;
    while turn < clears - hair {
        asking.push((turn, false));
        turn += step;
    }

    asking.push((clears, true));

    // C#'s `List.Sort` (an introsort) by `double.CompareTo`, so tied turns land where C# leaves them.
    cs_sort::introspective_sort(&mut asking, &mut |first, second| {
        compare_turns(first.0, second.0)
    });

    // And then the ones that would stand on top of each other are COLLAPSED - measured by where
    // their trim points and rail points STAND rather than by the angle between them, since the two
    // run at opposite speeds. Collapsed, not thinned: where two stations are one place ONE comes out
    // and the other is re-measured against the station before it, however many that takes. A step
    // always gives way; between two that must stand the END wins, otherwise the earlier one stays.
    let from_the_bead = try_scallop_point(corner, wall, 0.0).ok_or_else(not_supported)?;

    let mut at = vec![from_the_bead];
    let mut rails = vec![corner.outer_rail_point(corner.turn_radians())];
    let mut standing = vec![true];

    for (turn, must_stand) in asking {
        if !(turn > 0.0) || turn > clears {
            continue;
        }
        let Some(trim) = try_scallop_point(corner, wall, turn) else {
            continue;
        };
        let Some((stands_off, _)) = wall.try_out_of_the_axis(corner.turn_radians() + turn) else {
            continue;
        };

        // A station the wall has already SWALLOWED is no station either: its column would hold
        // nothing but the trim point. The end is never swallowed - it IS where the wall takes the
        // last of it.
        let touch_row = carry.map_or(corner.outer_rail_radius(), |carry| {
            carry.the_touch_row_stands_off_at(turn)
        });
        if turn < clears && stands_off > touch_row - near_enough {
            continue;
        }

        let rail = corner.outer_rail_point(corner.turn_radians() + turn);
        let is_the_end = !(turn < clears);
        let mut stands = true;

        // What this candidate has taken out, put back if it is then refused against a station that
        // must stand.
        let mut popped: Vec<(Vector3, Vector3, bool, f64)> = Vec::new();

        while stands
            && is_one_place_with(
                at[at.len() - 1],
                rails[rails.len() - 1],
                trim,
                rail,
                near_enough,
            )
        {
            if standing[standing.len() - 1] && !is_the_end {
                // The one already there must stand and this one has no better claim.
                stands = false;
                continue;
            }

            if at.len() == 1 {
                // All the way back to the SEED - the bead's own last rail, the row the runout
                // starts from. The end standing there says there is no runout here to draw.
                return Err(not_supported());
            }

            // It gives way, and the candidate is measured again against what is behind it.
            // (`at`, `rails`, `standing` and `turns` move together; `at` has one more, the seed.)
            let (Some(a), Some(r), Some(s), Some(t)) =
                (at.pop(), rails.pop(), standing.pop(), turns.pop())
            else {
                return Err(not_supported());
            };
            popped.push((a, r, s, t));
        }

        if !stands {
            for (a, r, s, t) in popped.into_iter().rev() {
                at.push(a);
                rails.push(r);
                standing.push(s);
                turns.push(t);
            }

            continue;
        }

        turns.push(turn);
        at.push(trim);
        rails.push(rail);
        standing.push(must_stand);
    }

    // The end is the point the trim curve dies at, and the runout is that curve: a turn whose last
    // station is anything else is a different surface.
    match turns.last() {
        Some(&last) if !(last < clears) => Ok(turns),
        _ => Err(not_supported()),
    }
}
