//! What tops a run-out's column at a station standing BETWEEN two of the cap band's row crossings
//! (`SurfaceRunOutColumnWalk::the_chord_between_two_crossings_tops_it`), and what it does when
//! everything about the station says the band's chord tops it but the chord has no start. Driven on
//! its own numbers rather than through a build.

use solidmesh::bevel::surface::SurfaceRunOutColumnWalk;

/// Row 0 crosses at 0.1 radians past the bead and row 1 at 0.3, with row 2 never.
const CROSSES_AT: [f64; 3] = [0.1, 0.3, f64::INFINITY];

/// The ordinary station: its turn stands between the two crossings, the row below carries its own
/// vertex on the band, and the chord from it tops the column.
#[test]
fn a_station_between_two_crossings_is_topped_by_the_bands_chord() {
    let on_the_plane = [7, -1, -1];

    let (tops_it, the_chord_is_missing) =
        SurfaceRunOutColumnWalk::the_chord_between_two_crossings_tops_it(
            1,
            1,
            &CROSSES_AT,
            &on_the_plane,
            0.2,
        );

    assert!(
        tops_it,
        "row 0 has crossed and row 1 has not, so the top stands on the band's chord between the two"
    );
    assert!(
        !the_chord_is_missing,
        "there is a chord and it is read, so there is nothing to name"
    );
}

/// And the same station with the row below NOT on the band: the reading says no chord tops it,
/// and says WHY.
#[test]
fn a_chord_whose_row_below_never_reached_the_band_is_named() {
    let on_the_plane = [-1, -1, -1];

    let (tops_it, the_chord_is_missing) =
        SurfaceRunOutColumnWalk::the_chord_between_two_crossings_tops_it(
            1,
            1,
            &CROSSES_AT,
            &on_the_plane,
            0.2,
        );

    assert!(
        !tops_it,
        "there is no vertex on the band to start the chord at"
    );
    assert!(
        the_chord_is_missing,
        "and that is the refusal, not an ordinary station: the column would otherwise be topped by the \
         row the chord was to have replaced, which stands over material the band removed"
    );
}

/// A station OUTSIDE the two crossings is an ordinary one however the rows below stand.
#[test]
fn a_station_past_the_second_crossing_is_no_chords_business() {
    let on_the_plane = [-1, -1, -1];

    for past in [0.05, 0.4] {
        let (tops_it, the_chord_is_missing) =
            SurfaceRunOutColumnWalk::the_chord_between_two_crossings_tops_it(
                1,
                1,
                &CROSSES_AT,
                &on_the_plane,
                past,
            );

        assert!(!tops_it, "{past} does not stand between the two crossings");
        assert!(
            !the_chord_is_missing,
            "so there was never a chord to be missing"
        );
    }
}

/// And the row the top would stand on has to be CROSSED: past the last crossed row the top is read
/// along the chord to the hand-over instead.
#[test]
fn a_station_past_the_last_crossed_row_is_no_chords_business() {
    let on_the_plane = [-1, -1, -1];

    let (tops_it, the_chord_is_missing) =
        SurfaceRunOutColumnWalk::the_chord_between_two_crossings_tops_it(
            1,
            0,
            &CROSSES_AT,
            &on_the_plane,
            0.2,
        );

    assert!(!tops_it, "row 1 is past the last row the upper cut takes");
    assert!(
        !the_chord_is_missing,
        "the chord between two crossings is not what tops a column out there"
    );
}
