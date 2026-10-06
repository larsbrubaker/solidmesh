//! Port of `SurfaceRunOutLensReadingsTests.cs`: the two arithmetics the LENS is read by, each on
//! its own - which row slot a column's own top takes (`surface_run_out_column_ends`), and where
//! the touch row leaves the face it runs across (`surface_mixed_corner_bites`). The second reads
//! the L prism's mixed corner from `surface_mixed_corner_reading_tests::the_mixed_corner`.

use geometry_processing::bevel::surface::surface_mixed_corner_bites::try_where_the_touch_row_leaves_the_face;
use geometry_processing::bevel::surface::surface_run_out_column_ends::try_the_slot_the_top_takes;
use geometry_processing::bevel::surface::SurfaceBandEnd;

use crate::surface_mixed_corner_reading_tests::the_mixed_corner;

/// A profile's rows, furthest out of the axis first, as a run-out always holds them.
const ROWS: [f64; 6] = [7.5, 6.8, 6.1, 5.4, 4.7, 4.0];

/// A top standing among the rows takes the slot of the LAST row it swallowed, so the column
/// either side of it still pairs by row number.
#[test]
fn the_top_takes_the_slot_of_the_last_row_it_swallowed() {
    let slot = try_the_slot_the_top_takes(&ROWS, 0, 6.5, 1e-6).expect("6.5 stands among the rows");

    assert_eq!(
        slot, 1,
        "7.5 and 6.8 stand outside 6.5, so the top takes the second of them and the rows below it stand"
    );

    let slot = try_the_slot_the_top_takes(&ROWS, 0, 4.3, 1e-6).expect("4.3 stands among the rows");

    assert_eq!(
        slot, 4,
        "everything down to 4.7 is swallowed, and only the last row stands under the top"
    );
}

/// And a top standing OUTSIDE every row swallows nothing, so there is no slot for it: the column
/// is the ordinary prefix it always was.
#[test]
fn a_top_that_swallows_nothing_has_no_slot() {
    assert!(
        try_the_slot_the_top_takes(&ROWS, 0, 9.0, 1e-6).is_none(),
        "nine is outside every row, so the top stands above the column rather than among it"
    );

    assert!(
        try_the_slot_the_top_takes(&ROWS, 2, 6.9, 1e-6).is_none(),
        "read from row two, 6.9 is outside every row that is left"
    );
}

/// The touch row crosses the plane that ends a face TWICE, and the one it meets is the one it is
/// running towards - the nearer of the two to the point it has reached.
#[test]
fn the_touch_row_leaves_the_face_on_the_side_it_is_running_towards() {
    // Read in the corner's OWN frame rather than against the axes: the plane is put one radius
    // along one of the cap's two directions, so the two roots stand a known half chord either
    // side of it.
    let corner = the_mixed_corner();
    let centre = corner.outer_rail_centre;
    let up = corner.towards_the_shared_face();
    let across = up.cross(corner.out_of_the_cap_face).get_normal();
    let size = corner.size;
    let ends_on = SurfaceBandEnd::new(centre + (up * size), -up);
    let half = ((4.0 * size * size) - (size * size)).sqrt();
    let one = centre + (up * size) + (across * half);
    let other = centre + (up * size) - (across * half);

    let leaves_at = try_where_the_touch_row_leaves_the_face(&corner, ends_on, one + (across * 2.0))
        .expect("the row reaches the plane");

    assert!(
        (leaves_at - one).length() < 1e-9,
        "the row has reached that side, so it is that root it leaves on - instead {leaves_at:?}"
    );

    let the_other =
        try_where_the_touch_row_leaves_the_face(&corner, ends_on, other - (across * 2.0))
            .expect("the row reaches the plane");

    assert!(
        (the_other - other).length() < 1e-9,
        "reached from the other side, the row leaves on the other root - instead {the_other:?}"
    );

    assert!(
        ((leaves_at - centre).length() - 2.0 * size).abs() <= 1e-9,
        "both roots stand on the circle the touch point traces"
    );
}

/// And a face ending beyond the ball's own reach is never left at all.
#[test]
fn a_face_ending_past_the_reach_is_never_left() {
    let corner = the_mixed_corner();
    let up = corner.towards_the_shared_face();
    let far_off = SurfaceBandEnd::new(corner.outer_rail_centre + (up * 3.0 * corner.size), -up);

    assert!(
        try_where_the_touch_row_leaves_the_face(&corner, far_off, corner.outer_rail_centre)
            .is_none(),
        "the touch row never reaches a plane three radii out"
    );
}
