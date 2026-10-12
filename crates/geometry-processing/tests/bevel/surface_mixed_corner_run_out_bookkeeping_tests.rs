//! Port of `SurfaceMixedCornerRunOutBookkeepingTests.cs`: the runout's own BOOKKEEPING, asked of the
//! readings themselves rather than of a part - which rows the plane across the wall's end has
//! already taken at a station, which crossings may name the turn the stations stop chording at, and
//! how far along its own chord a row is the lap test's business.
//!
//! Each of these is a place the columns could be built wrong without any fixture noticing: the
//! answer is still a closed surface, it just stands somewhere the part does not carry material.
//! Asked here of the arrays the readings are made of, so that the wrong answer FAILS.

use geometry_processing::bevel::surface::{
    surface_mixed_corner_run_out, surface_run_out_column_ends, SurfaceMixedCorner,
    SurfaceMixedCornerWall,
};
use vector_math::Vector3;

use crate::surface_mixed_corner_fixture as fixture;

/// A row that never reaches the plane does not stop the count: the rows the plane has taken are
/// read from ALL of them, so a row below one the wall swallowed is still past its own crossing.
#[test]
fn a_row_below_one_that_never_crosses_is_still_read_as_taken() {
    let crosses_at = [0.1, 0.2, 0.3];

    let first_row = surface_run_out_column_ends::try_first_row_standing(&crosses_at, 0.25)
        .expect("the rows read");
    assert_eq!(
        first_row, 2,
        "the two rows above 0.25 are gone - instead {first_row}"
    );

    // The third row crosses before this station and the second never crosses at all, so the rows
    // the plane has taken are no longer a run from the top and the column cannot be a prefix of
    // anything.
    let out_of_order =
        surface_run_out_column_ends::try_first_row_standing(&[0.1, f64::INFINITY, 0.3], 0.4);
    assert!(
        out_of_order.is_none(),
        "instead it read {out_of_order:?} and left the row that crossed standing"
    );
}

/// The turn the stations stop chording at is named only by a row the WALL still has material
/// under: a row the wall swallowed earlier crosses a plane that is not yet cutting anything of the
/// surface, and letting it name that turn throws the chording away for nothing.
#[test]
fn a_row_the_wall_has_already_taken_does_not_name_where_the_cutting_starts() {
    let stations = [0.1, 0.2, 0.3, 0.4];
    let crosses_at = [0.05, 0.35];
    let foot_crosses_at = 0.5;

    // The top row's crossing at 0.05 is a row the wall took before the plane could have cut it, so
    // the chording stands until the row that still has material under it crosses at 0.35.
    let stationed = surface_run_out_column_ends::with_the_crossings_stationed(
        &stations,
        &crosses_at,
        &[true, false],
        foot_crosses_at,
        None,
    );

    assert!(
        stationed.contains(&0.2),
        "the chording has to survive - instead {stationed:?}"
    );

    assert!(
        stationed.contains(&0.3),
        "right up to the first crossing that cuts - instead {stationed:?}"
    );

    // And once a row the wall has NOT taken names it, the chording past that turn goes as before.
    let cutting = surface_run_out_column_ends::with_the_crossings_stationed(
        &stations,
        &crosses_at,
        &[false, false],
        foot_crosses_at,
        None,
    );

    assert!(
        !cutting.contains(&0.2),
        "past the first real crossing the stations are the crossings - instead {cutting:?}"
    );
}

/// A row PAST the far end of the trim chord is the next piece's business, and one BEHIND its start
/// is still this piece's: the chord advances in turn while the rows asked of it stand at the
/// station's own turn, so a row high on the profile projects behind the trim point beside it.
#[test]
fn a_row_past_the_chords_end_is_the_next_pieces_and_one_behind_its_start_is_still_this() {
    let (plan, _, _) = fixture::the_plan();
    let corner = fixture::the_corner(&plan);

    let wall = SurfaceMixedCornerWall::try_read(&corner)
        .unwrap_or_else(|why| panic!("the wall has to read: {:?}", why.code()));

    let stations = [0.0, 0.2];
    let trims: Vec<Vector3> = stations
        .iter()
        .map(|&station| {
            surface_mixed_corner_run_out::try_scallop_point(&corner, &wall, station)
                .expect("the trim point has to read")
        })
        .collect();

    let chord = trims[1] - trims[0];

    // The same standoff onto the side of the chord the wall keeps, read at three places along it:
    // behind the chord's start, in the middle of it, and past its end.
    let stand_off =
        the_way_the_wall_keeps(&corner, &wall, &stations, &trims) * (chord.length() / 20.0);

    assert!(
        surface_run_out_column_ends::it_laps_over_the_walls_own_leftover(
            trims[0] + (chord * 0.5) + stand_off,
            &trims,
            &stations,
            0,
            &corner,
            &wall
        ),
        "over the chord's own span a row past it really is lapping"
    );

    assert!(
        surface_run_out_column_ends::it_laps_over_the_walls_own_leftover(
            trims[0] - (chord * 0.2) + stand_off,
            &trims,
            &stations,
            0,
            &corner,
            &wall
        ),
        "and behind its start it is still this piece's - that is the station's own turn"
    );

    assert!(
        !surface_run_out_column_ends::it_laps_over_the_walls_own_leftover(
            trims[0] + (chord * 1.2) + stand_off,
            &trims,
            &stations,
            0,
            &corner,
            &wall
        ),
        "and past its end it is the piece after this one's"
    );
}

/// The unit way across the trim chord that the wall KEEPS - the side away from the cap, which is
/// the side a lapping row stands on.
fn the_way_the_wall_keeps(
    corner: &SurfaceMixedCorner,
    wall: &SurfaceMixedCornerWall,
    stations: &[f64],
    trims: &[Vector3],
) -> Vector3 {
    let (_, facing) = wall
        .try_plane_at(corner.turn_radians() + ((stations[0] + stations[1]) / 2.0))
        .expect("the wall's facet has to read");

    let across = facing.cross(trims[1] - trims[0]).get_normal();

    if across.dot(corner.out_of_the_cap_face) < 0.0 {
        across
    } else {
        -across
    }
}
