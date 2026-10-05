//! The WALK that draws a run-out's columns ([`SurfaceRunOutColumnWalk`]). Ports the fields of
//! `Bevel/Surface/SurfaceRunOutColumnWalk.cs`, its constructor, the test of a column topped by a
//! chord between two crossings, and the walk itself ([`SurfaceRunOutColumnWalk::try_walk`]). Kept
//! apart from `SurfaceMixedCornerRunOut`, which reads the turns and
//! emits the surface, and from [`super::SurfaceRunOutColumnWindow`], which reads what ends the
//! columns from above and below.

/// C# `SurfaceRunOutColumnWalk`: the walk that draws a run-out's columns - one column per station,
/// from the rows the window leaves standing there down to the point the surface ends on, and the
/// curve those ends make where a band is what ended them.
///
/// This is the drawing itself: which rows a column holds, what tops it, what it ends on, and what
/// each of those is worth to the surfaces the run-out meets. Every column is a run of the profile's
/// own rows turned to that station's meridian, so two neighbours pair by ROW NUMBER rather than by
/// position - which is why the first row each of them holds is carried alongside it
/// (`SurfaceMixedCornerRunOutStrip`).
use super::surface_run_out_column_ends::{self, HAIR};
use super::{
    SurfaceCrossingAtTheMitre, SurfaceMixedCorner, SurfaceMixedCornerWall, SurfaceRunOutCarry,
    SurfaceRunOutColumnWindow, SurfaceVertexPool,
};
use crate::bevel::{BevelAdvisory, BevelAdvisoryCode};

#[derive(Debug, Clone, PartialEq, Default)]
pub struct SurfaceRunOutColumnWalk {
    /// The column tops standing between one crossed row's crossing and the NEXT one's, entry per
    /// row: `tops_between_crossings[row]` is what the boundary steps through between `row`'s
    /// crossing and the crossing of the row below it, in turn order.
    ///
    /// Only a mitred pair has any: the wall band's own rails are stations too, so the run-out is
    /// drawn at turns BETWEEN two of the cap band's row crossings, and at such a turn the boundary
    /// stands neither on one row nor on the next. What it stands on is the cap band's own chord
    /// between those two crossings - which is the line the band's staircase draws there - so the
    /// top is read along that chord and handed back to be carried as one of its between points. A
    /// torus row turned to that station instead would be a vertex of the run-out that the strip has
    /// no point of.
    pub tops_between_crossings: Vec<Vec<i32>>,
    /// The columns, in station order, the patch's own last column first.
    pub columns: Vec<Vec<i32>>,
    /// The turn past the bead each column is drawn at, entry for entry with the columns.
    pub turns: Vec<f64>,
    /// The first row of the profile each column holds, entry for entry with the columns.
    pub first_rows: Vec<i32>,
    /// The curve the run-out and a band on the WALL meet in: the end of every column that band
    /// ended, in the order the turn runs.
    pub the_curve: Vec<i32>,
    /// What the piece before each column carries on its top rim beyond the two columns' own tops,
    /// entry for entry with the columns, or `None` where it carries nothing. One point where the
    /// touch row ends between two stations: the cap exit, which stands at no station of its own.
    pub top_rims: Vec<Option<Vec<i32>>>,
    /// The column tops carried along a mitred pair's own chord, in turn order.
    pub along_the_chord: Vec<i32>,
    /// The vertex that chord starts at, or -1.
    pub the_chord_starts_at: i32,
    /// The vertex each row left standing ON the plane across a wall's end, or -1 per row.
    pub on_the_plane: Vec<i32>,
}

impl SurfaceRunOutColumnWalk {
    /// C#'s private constructor, in C#'s argument order.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        columns: Vec<Vec<i32>>,
        turns: Vec<f64>,
        first_rows: Vec<i32>,
        the_curve: Vec<i32>,
        top_rims: Vec<Option<Vec<i32>>>,
        along_the_chord: Vec<i32>,
        tops_between_crossings: Vec<Vec<i32>>,
        the_chord_starts_at: i32,
        on_the_plane: Vec<i32>,
    ) -> Self {
        SurfaceRunOutColumnWalk {
            tops_between_crossings,
            columns,
            turns,
            first_rows,
            the_curve,
            top_rims,
            along_the_chord,
            the_chord_starts_at,
            on_the_plane,
        }
    }

    /// Whether the column at `past` is topped by the CHORD between the crossings of rows
    /// `first_row - 1` and `first_row` (it stands strictly between those two crossings, at or
    /// before the last row crossed), and - second - whether that chord is missing because the
    /// earlier row left no vertex on the plane. Returns `(tops_it, the_chord_is_missing)`.
    pub fn the_chord_between_two_crossings_tops_it(
        first_row: usize,
        the_last_crossed: usize,
        crosses_at: &[f64],
        on_the_plane: &[i32],
        past: f64,
    ) -> (bool, bool) {
        let between_two_crossings = first_row > 0
            && first_row < crosses_at.len()
            && first_row <= the_last_crossed
            && crosses_at[first_row] != f64::INFINITY
            && past > crosses_at[first_row - 1] + HAIR
            && past < crosses_at[first_row] - HAIR;

        let the_chord_is_missing = between_two_crossings && on_the_plane[first_row - 1] < 0;

        (
            between_two_crossings && !the_chord_is_missing,
            the_chord_is_missing,
        )
    }
}

impl SurfaceRunOutColumnWalk {
    /// C# `TryWalk`: walks the stations of `window` and draws the columns - from the patch's own
    /// `last_column`, which is the run-out's first, each station's rows the window leaves standing
    /// carried there by `carry`, down to the point the surface ends on. `at_the_mitre` is the
    /// hand-over where a mitred pair is crossed. A shape this construction refuses is named.
    #[allow(clippy::too_many_arguments)]
    pub fn try_walk(
        pool: &mut SurfaceVertexPool,
        corner: &SurfaceMixedCorner,
        wall: &SurfaceMixedCornerWall,
        window: &SurfaceRunOutColumnWindow,
        last_column: &[i32],
        carry: &SurfaceRunOutCarry,
        at_the_mitre: Option<&SurfaceCrossingAtTheMitre>,
    ) -> Result<SurfaceRunOutColumnWalk, BevelAdvisory> {
        let refused =
            |code: BevelAdvisoryCode| BevelAdvisory::new(code, vec![corner.corner.into()]);

        let mut columns: Vec<Vec<i32>> = vec![last_column.to_vec()];
        let mut first_rows: Vec<i32> = vec![0];

        // The patch's own last turn is the run-out's first station, so the list the caller reads is
        // one longer than the turns past the bead and lines up with the columns entry for entry.
        let mut station_turns = vec![0.0];

        let stations = &window.stations;
        let crosses_at: Vec<f64> = (0..last_column.len())
            .map(|row| window.where_the_row_crosses(row))
            .collect();

        let foot_crosses_at = window.the_switch_turn;
        let the_cap_exit = window.the_cap_exit;

        // The curve the run-out and a band on the wall meet in: the end of every column that band
        // ended, in the order the turn runs.
        let mut the_curve = Vec::new();
        let mut on_the_plane = vec![-1; last_column.len()];

        // The last row the upper cut takes before the two cuts meet (C#'s -1 as `None`).
        let mut the_last_crossed: Option<usize> = None;

        for (row, &crossing) in crosses_at.iter().enumerate() {
            if crossing != f64::INFINITY && crossing < foot_crosses_at - HAIR {
                the_last_crossed = Some(row);
            }
        }

        let mut along_the_chord = Vec::new();
        let mut the_chord_starts_at = -1;
        let mut tops_between_crossings: Vec<Vec<i32>> = vec![Vec::new(); crosses_at.len()];

        // How many columns the LENS holds - whether it is wide enough to hold a station at all.
        let mut the_lens_columns = 0;

        // What each piece carries on its TOP rim beyond the two columns' own tops, entry for entry
        // with the columns: the piece before column i is top_rims[i].
        let mut top_rims: Vec<Option<Vec<i32>>> = vec![None];

        for (station, &past) in stations.iter().enumerate() {
            let stands_off = window.stands_off_at(station);
            let scallop = window.scallop_at(station);

            // How far out of the axis each row stands AT THIS STATION.
            let out_of_the_axis = carry.out_of_the_axis_at(past);

            if past > foot_crosses_at + HAIR {
                // Past the turn the foot left the material there is nothing of the surface left.
                break;
            }

            // Rows the plane has already taken, from the TOP.
            let Some(mut first_row) =
                surface_run_out_column_ends::try_first_row_standing(&crosses_at, past)
            else {
                return Err(refused(BevelAdvisoryCode::SurfaceCornerNotSupported));
            };

            if window.the_band_ends_them(station) && station == stations.len() - 1 {
                // The HAIRPIN: at the band's own furthest turn its two sides meet, so the column there
                // is that one point and the run-out closes on it - built BEFORE any row is turned, so
                // no orphan vertex is left a hundredth off the rim. Where a band on the CAP closes the
                // window, the two sides meet AT the hand-over on the mitre.
                if let Some(at_the_mitre) = at_the_mitre {
                    the_curve.push(at_the_mitre.vertex.get());
                    columns.push(vec![at_the_mitre.vertex.get()]);
                    station_turns.push(past);
                    first_rows.push(0);

                    break;
                }

                let at_the_hairpin = pool.add(scallop);

                the_curve.push(at_the_hairpin);
                columns.push(vec![at_the_hairpin]);
                station_turns.push(past);

                // Where NOTHING stood in the lens, the touch row ends between the station before this
                // one and the hairpin itself, so this piece carries the exit.
                top_rims.push(if the_lens_columns == 0 && the_cap_exit >= 0 {
                    Some(vec![the_cap_exit])
                } else {
                    None
                });

                // A column of one point is read at the row the station before it started at.
                first_rows.push(first_rows[first_rows.len() - 1]);

                break;
            }

            // Where the WALL has reached the outer rail the column is the trim point and NOTHING
            // else: the turn the run-out dies at. Read off the wall's own radius rather than off the
            // rows, which disagree in the last thousandth.
            if stands_off >= carry.the_touch_row_stands_off_at(past) - pool.near_enough()
                && !window.the_band_ends_them(station)
            {
                let dies = pool.add(scallop);

                columns.push(vec![dies]);
                station_turns.push(past);
                first_rows.push(first_rows[first_rows.len() - 1]);
                top_rims.push(None);

                break;
            }

            let mut column: Vec<i32> = Vec::with_capacity(last_column.len());

            // Past the LAST row the upper cut takes, the column's top stands on the upper cut's own
            // chord between the last crossed row and the hand-over, read at this turn - a point of the
            // band's rim as much as of the run-out's.
            if let (Some(at_the_mitre), Some(last_crossed)) = (at_the_mitre, the_last_crossed) {
                let crosses = window.where_the_row_crosses(last_crossed);

                if first_row > last_crossed
                    && past > crosses + HAIR
                    && past < foot_crosses_at - HAIR
                {
                    if on_the_plane[last_crossed] < 0 {
                        // The last crossed row never put a vertex on the band (it stood inside the
                        // LOWER cut, or the lapping reading took it back), so there is no point to
                        // start the chord at.
                        return Err(refused(BevelAdvisoryCode::SurfaceCrossingMissesTheMitre));
                    }

                    let from = pool.position_of(on_the_plane[last_crossed]);
                    let how_far = (past - crosses) / (foot_crosses_at - crosses);

                    let top = pool.add(from + ((at_the_mitre.at - from) * how_far));

                    column.push(top);
                    along_the_chord.push(top);
                    the_chord_starts_at = on_the_plane[last_crossed];
                }
            }

            // And BETWEEN two crossings, the column's top is the point on the cap band's DIAGONAL
            // CHORD between the two crossings at this turn, standing in the slot of the row it
            // replaces so the columns either side still pair by row number.
            let (the_chord_tops_it, the_chord_is_missing) = match (at_the_mitre, the_last_crossed) {
                (Some(_), Some(last_crossed)) => Self::the_chord_between_two_crossings_tops_it(
                    first_row,
                    last_crossed,
                    &crosses_at,
                    &on_the_plane,
                    past,
                ),
                // C#'s `firstRow <= -1` is never true, so with no row crossed nothing tops it.
                _ => (false, false),
            };

            if the_chord_is_missing {
                // Nothing to start the chord at, and no row of the profile may stand in for it.
                return Err(refused(BevelAdvisoryCode::SurfaceCrossingMissesTheChord));
            }

            if the_chord_tops_it {
                let from = pool.position_of(on_the_plane[first_row - 1]);

                // The far end is the crossing the row below has not reached yet, read where it will
                // stand: the pool welds the two when its own station comes.
                let to = carry.at(first_row, crosses_at[first_row]);
                let how_far = (past - crosses_at[first_row - 1])
                    / (crosses_at[first_row] - crosses_at[first_row - 1]);

                let on_the_chord = pool.add(from + ((to - from) * how_far));

                column.push(on_the_chord);
                tops_between_crossings[first_row - 1].push(on_the_chord);
            }

            // Inside the LENS the rows above the band's far side are dropped and the CAP EXIT tops
            // the column, in the SLOT of the last row it swallowed.
            let mut the_lens_tops_it = false;

            if let Some(the_lens_stands_off) = window.try_the_lens_top(station) {
                if let Some(slot) = surface_run_out_column_ends::try_the_slot_the_top_takes(
                    &out_of_the_axis,
                    first_row,
                    the_lens_stands_off,
                    pool.near_enough(),
                ) {
                    first_row = slot;
                    the_lens_tops_it = true;
                }
            }

            if the_lens_tops_it {
                column.push(the_cap_exit);
                the_lens_columns += 1;
            }

            let skip = usize::from(the_lens_tops_it || the_chord_tops_it);

            for row in (first_row + skip)..last_column.len() {
                if out_of_the_axis[row] < stands_off + pool.near_enough() {
                    break;
                }

                let at = pool.add(carry.at(row, past));

                column.push(at);

                if (crosses_at[row] - past).abs() <= HAIR {
                    // This station IS that row's crossing, so this vertex stands on the plane and is
                    // a corner of the section the face across gives up.
                    on_the_plane[row] = at;
                }
            }

            // And then the rows that LAP: a row whose own shadow on the wall falls past the trim
            // chord, on the material the wall keeps - a fin of a few hundredths if drawn. Never the
            // first row the column has left: the cap's own sector is drawn from it.
            while column.len() > 1
                && surface_run_out_column_ends::it_laps_over_the_walls_own_leftover(
                    pool.position_of(column[column.len() - 1]),
                    window.scallops(),
                    stations,
                    station,
                    corner,
                    wall,
                )
            {
                let popped = first_row + column.len() - 1;

                if popped < on_the_plane.len() && on_the_plane[popped] == column[column.len() - 1] {
                    // It was this station's own crossing as well, so it has just left the strip.
                    on_the_plane[popped] = -1;
                }

                column.pop();
            }

            let trim = pool.add(scallop);

            if window.the_band_ends_them(station) {
                the_curve.push(trim);
            }

            // The trim point can weld onto the last row it left standing - and then the column
            // simply ends on that row.
            if column.last() != Some(&trim) {
                column.push(trim);
            }

            // Where the plane across the wall's end has taken every row a column had, the piece
            // carries the section's own rim on its top - from the last top the plane left.
            let count = columns.len();
            let top_rim = if count >= 2
                && column.len() == 1
                && columns[count - 1].len() == 1
                && columns[count - 2].len() > 1
            {
                Some(vec![columns[count - 2][0]])
            } else {
                None
            };

            columns.push(column);
            station_turns.push(past);
            first_rows.push(first_row as i32);
            top_rims.push(top_rim);
        }

        Ok(Self::new(
            columns,
            station_turns,
            first_rows,
            the_curve,
            top_rims,
            along_the_chord,
            tops_between_crossings,
            the_chord_starts_at,
            on_the_plane,
        ))
    }
}
