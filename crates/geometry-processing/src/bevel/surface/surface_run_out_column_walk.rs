//! The WALK that draws a run-out's columns ([`SurfaceRunOutColumnWalk`]). Ports the fields of
//! `Bevel/Surface/SurfaceRunOutColumnWalk.cs`, its constructor and the test of a column topped by a
//! chord between two crossings; the walk itself lands with a later package as a further `impl`
//! block. Kept apart from `SurfaceMixedCornerRunOut`, which reads the turns and
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
use super::surface_run_out_column_ends::HAIR;

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
