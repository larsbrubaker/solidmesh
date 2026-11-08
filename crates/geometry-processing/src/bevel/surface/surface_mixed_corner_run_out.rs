//! A mixed corner's RUN-OUT: the blend carried on past the bead's own turn until the ball's touch
//! row dies into the sharp edge the part keeps: its constant, the one-place test, and where a
//! station's arc meets the wall ([`try_scallop_point`], [`try_scallop_on_the_ring`]), and the turns
//! its stations stand at ([`try_turns_past_the_bead`]), its columns ([`try_columns`]), and the
//! run-out drawn station by station ([`try_emit`]).

use std::cmp::Ordering;
use std::rc::Rc;

use solidmesh_math::{fixed_tie_sort, ieee_math, Vector3};

use super::surface_run_out_column_ends;
use super::{surface_mixed_corner_patch, surface_mixed_corner_run_out_strip};
use super::{
    SurfaceBandEnd, SurfaceCrossingAtTheMitre, SurfaceMixedCorner, SurfaceMixedCornerRows,
    SurfaceMixedCornerWall, SurfaceRunOutCarry, SurfaceRunOutColumnWalk, SurfaceRunOutColumnWindow,
    SurfaceRunOutMeetsAWallBand, SurfaceRunOutMeetsTheBand, SurfaceVertexPool,
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

/// Where station `past`'s arc crosses the wall's plane - out of the axis by the wall's own radius
/// there, and down from the cap by as much of the ball as is left over it. Where the bead's ring
/// leans it is the station's own ring met with the wall ([`try_scallop_on_the_ring`]); the square
/// corner keeps the closed form so none of its numbers moves in the last bits.
pub fn try_scallop_point(
    corner: &SurfaceMixedCorner,
    wall: &SurfaceMixedCornerWall,
    past: f64,
) -> Option<Vector3> {
    if corner.the_beads_ring_leans() {
        return try_scallop_on_the_ring(corner, wall, past);
    }

    let (out_of_the_axis, _) = wall.try_out_of_the_axis(corner.turn_radians() + past)?;

    // Squared as `x * x`, which is the correctly rounded square.
    let left_over = (2.0 * corner.size) - out_of_the_axis;
    let under_the_cap = corner.size
        - ieee_math::max_f64(0.0, (corner.size * corner.size) - (left_over * left_over)).sqrt();

    Some(
        corner.outer_rail_centre
            + (corner.out_of_the_axis(corner.turn_radians() + past) * out_of_the_axis)
            - (corner.out_of_the_cap_face * under_the_cap),
    )
}

/// The same point read as the STATION'S OWN RING met with the wall's facet. The ring is `C + r(u
/// cos psi + across sin psi)` - the ball's centre, the way to the face it rides, and the arm across
/// it the rows lay their rails in - so met with the facet's plane it is `A cos psi + B sin psi =
/// K`, and the root taken is the FIRST at or past `psi = 0`, the cap contact. On a square cap it is
/// the closed form of [`try_scallop_point`] to a nanometre.
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

/// A NaN before every number, and two NaNs equal.
fn compare_turns(first: f64, second: f64) -> Ordering {
    match (first.is_nan(), second.is_nan()) {
        (true, true) => Ordering::Equal,
        (true, false) => Ordering::Less,
        (false, true) => Ordering::Greater,
        _ => first.partial_cmp(&second).unwrap_or(Ordering::Equal),
    }
}

/// The turns past the patch's last, in order - one on every crease of the wall the curve runs over,
/// the caller's own angular `step` wherever a crease has not already stationed it, and one last
/// station EXACTLY where the ball clears the wall (the point the trim curve dies at, named rather
/// than stepped to). `near_enough` is the part's ruler; nearer the end than that a station is not
/// worth standing on. `touches_the_cap_at` is where the ball touches the cap at the patch's own
/// last turn (`None` for the ideal rail's point), and `carry` the run-out's carry where the column
/// is laid on the spine.
///
/// A step that is not positive asks for no stations at all.
#[allow(clippy::neg_cmp_op_on_partial_ord)] // `!(x > 0)` / `!(x < y)` on purpose: a NaN passes.
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

    let hair = ieee_math::max_f64(
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

    // An introsort by `f64` order: the one deterministic order every run lands tied turns in.
    fixed_tie_sort::introspective_sort(&mut asking, &mut |first, second| {
        compare_turns(first.0, second.0)
    });

    // And then the ones that would stand on top of each other are COLLAPSED - measured by where
    // their trim points and rail points STAND rather than by the angle between them, since the two
    // run at opposite speeds. Collapsed, not thinned: where two stations are one place ONE comes
    // out and the other is re-measured against the station before it, however many that takes. A
    // step always gives way; between two that must stand the END wins, otherwise the earlier one
    // stays.
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

/// What [`try_columns`] draws, by name.
#[derive(Debug, Clone, PartialEq)]
pub struct SurfaceMixedCornerRunOutColumns {
    /// The columns, in station order, the patch's own last column first.
    pub columns: Vec<Vec<i32>>,
    /// The turn past the bead each column is drawn at, entry for entry with the columns.
    pub turns: Vec<f64>,
    /// The first row of the profile each column holds, entry for entry with the columns.
    pub first_rows: Vec<i32>,
    /// The curve the CAP band (or the plane across the wall's end) is cut back by, or `None`.
    pub section: Option<Vec<i32>>,
    /// The curve the WALL band is cut back by, where both bands of a mitred pair are crossed.
    pub section_on_the_wall: Option<Vec<i32>>,
    /// Where the run-out would have died on the sharp edge had nothing crossed it, or -1.
    pub the_die_point: i32,
    /// The last column whose top the CAP's own sector may be drawn to (`i32::MAX` where there are
    /// no columns to name).
    pub the_sector_runs_to: i32,
    /// Where the crossing curve reaches the touch row and dies on the cap, or -1.
    pub the_cap_exit: i32,
    /// What the piece before each column carries on its top rim beyond the two columns' own tops.
    pub top_rims: Vec<Option<Vec<i32>>>,
}

/// The runout's columns, station by station, each one the rows of the patch's profile that still
/// stand in the material at that turn and then the point where the arc meets the wall. WHICH rows
/// is the window's to say ([`SurfaceRunOutColumnWindow::try_read`]) and the drawing the walk's
/// ([`SurfaceRunOutColumnWalk::try_walk`]); what is here is the carry they share, how far the cap's
/// sector may run, and the SECTION each crossed band (or the plane across the wall's end) is cut
/// back by. `rows` are the rows the patch's grid was laid with on the spine, or `None`.
#[allow(clippy::too_many_arguments)]
pub fn try_columns(
    pool: &mut SurfaceVertexPool,
    corner: &Rc<SurfaceMixedCorner>,
    wall: &SurfaceMixedCornerWall,
    last_column: &[i32],
    step: f64,
    ends_on: Option<SurfaceBandEnd>,
    meets_the_band: Option<&SurfaceRunOutMeetsTheBand>,
    meets_a_wall_band: Option<&SurfaceRunOutMeetsAWallBand>,
    at_the_mitre: Option<&SurfaceCrossingAtTheMitre>,
    rows: Option<Rc<SurfaceMixedCornerRows>>,
) -> Result<SurfaceMixedCornerRunOutColumns, BevelAdvisory> {
    let refused = |code: BevelAdvisoryCode| BevelAdvisory::new(code, vec![corner.corner.into()]);

    // How the patch's own last column is CARRIED past the bead's turn: rigidly about the axis, or
    // along the SPINE where the ring shrinks and each station has its own radii.
    let carry = SurfaceRunOutCarry::of(Rc::clone(corner), rows, pool, last_column);

    // And then the WINDOW: both cuts, the turns to draw them at, and the turn past which nothing is
    // left of any column.
    let window = SurfaceRunOutColumnWindow::try_read(
        pool,
        corner,
        wall,
        last_column,
        step,
        ends_on,
        meets_the_band,
        meets_a_wall_band,
        at_the_mitre,
        &carry,
    )?;

    let walk = SurfaceRunOutColumnWalk::try_walk(
        pool,
        corner,
        wall,
        &window,
        last_column,
        &carry,
        at_the_mitre,
    )?;

    // Only where the run-out really dies on the cap: with a band on the CAP as well the window's
    // exit is the hand-over on the two bands' mitre, which the sector has no business closing on.
    let the_cap_exit = if at_the_mitre.is_none() {
        window.the_cap_exit
    } else {
        -1
    };
    let SurfaceRunOutColumnWalk {
        columns,
        turns,
        first_rows,
        mut the_curve,
        top_rims,
        along_the_chord,
        tops_between_crossings,
        the_chord_starts_at,
        on_the_plane,
    } = walk;

    // How far the CAP's own sector may be drawn: to the last column standing at or before the cap
    // exit. With no crossing the exit's turn is infinite and this is every column there is.
    let mut the_sector_runs_to = i32::MAX;

    for (column, &turn) in turns.iter().enumerate() {
        if turn <= window.the_cap_exits_turn + surface_run_out_column_ends::HAIR {
            the_sector_runs_to = column as i32;
        }
    }

    let mut drawn = SurfaceMixedCornerRunOutColumns {
        columns,
        turns,
        first_rows,
        section: None,
        section_on_the_wall: None,
        the_die_point: window.the_die_point,
        the_sector_runs_to,
        the_cap_exit,
        top_rims,
    };

    if meets_a_wall_band.is_some() {
        if the_curve.len() < 2 {
            // One corner or none is no curve to cut either surface at.
            return Err(refused(BevelAdvisoryCode::SurfaceCrossingLeavesTheBand));
        }

        let Some(at_the_mitre) = at_the_mitre else {
            // The cap exit last, as the FOOT of the curve: what every column in the lens is topped
            // by, so the two surfaces meet along the one segment from the band's last rail to it.
            the_curve.push(the_cap_exit);
            drawn.section = Some(the_curve);

            return Ok(drawn);
        };

        // Two curves, one for each band, meeting at the hand-over: the WALL band's is the column
        // ends that band made; the CAP band's is read below and closes on the same vertex.
        drawn.section_on_the_wall = Some(the_curve);
        *at_the_mitre.along_the_chord.borrow_mut() = Some(along_the_chord);
        at_the_mitre.the_chord_starts_at.set(the_chord_starts_at);
    }

    if window.the_switch_turn != f64::INFINITY {
        // The SECTION the face across gives up: every row that reached the plane, in row order,
        // with whatever the boundary stepped through on the chord into the NEXT crossing, and the
        // foot the trim curve crossed at.
        let mut made = Vec::with_capacity(last_column.len() + 1);

        for (row, &on_it) in on_the_plane.iter().enumerate() {
            if on_it < 0 {
                continue;
            }

            made.push(on_it);
            made.extend_from_slice(&tops_between_crossings[row]);
        }

        if made.is_empty() || drawn.columns.len() < 2 {
            return Err(refused(BevelAdvisoryCode::SurfaceCornerNotSupported));
        }

        // The FOOT: the last column's own trim point with one cut; the hand-over with two (the
        // last column IS that one point, so the two read the same, and naming it says which).
        let foot = match at_the_mitre {
            Some(at_the_mitre) => at_the_mitre.vertex.get(),
            None => {
                let last = &drawn.columns[drawn.columns.len() - 1];
                last[last.len() - 1]
            }
        };
        made.push(foot);

        drawn.section = Some(made);
    }

    Ok(drawn)
}

/// The run-out drawn as a strip between each pair of neighbouring stations' columns (with each
/// station's top rim), facing the corner's outward sum; refused, naming the first piece that covers
/// a face already drawn, where the corner came in clean and a piece makes it fold.
pub fn try_emit(
    pool: &mut super::SurfaceVertexPool,
    faces: &mut Vec<(i32, i32, i32)>,
    corner: &super::SurfaceMixedCorner,
    columns: &[Vec<i32>],
    first_rows: &[i32],
    top_rims: Option<&[Vec<i32>]>,
) -> Result<(), BevelAdvisory> {
    let outward = corner.outward_sum();

    // Whether anything covers itself BEFORE the run-out has drawn a thing.
    let came_in_clean = surface_mixed_corner_patch::nothing_covers_nothing(pool, faces).is_ok();

    for station in 0..columns.len().saturating_sub(1) {
        let top_rim = top_rims
            .filter(|rims| station + 1 < rims.len())
            .map(|rims| rims[station + 1].as_slice());
        surface_mixed_corner_run_out_strip::emit(
            pool,
            faces,
            &columns[station],
            &columns[station + 1],
            first_rows[station],
            first_rows[station + 1],
            top_rim,
            outward,
        );

        // Read as each piece LANDS: the first station a fold shows at is the piece that made it.
        if came_in_clean {
            if let Err(covered) = surface_mixed_corner_patch::nothing_covers_nothing(pool, faces) {
                return Err(BevelAdvisory::new(
                    BevelAdvisoryCode::SurfaceRunOutLiesOnAFaceAlreadyDrawn,
                    vec![covered.args()[0].clone()],
                ));
            }
        }
    }

    Ok(())
}
