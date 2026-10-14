//! The SAME oblique corner of the nose-in-notch
//! part read the other way round - the ball is the BEAD's void ball, rolling along the WALL and
//! pressed against the CUT's tube, and the canal it sweeps is the whole of the corner the Onshape
//! control draws. Read through production's own `SurfaceMixedCorner::try_read_the_dual_canal`.

use std::f64::consts::{PI, TAU};
use std::rc::Rc;

use geometry_processing::bevel::surface::{
    SurfaceLine, SurfaceMixedCorner, SurfaceMixedCornerSpine,
};
use vector_math::Vector3;

use crate::nose_in_notch_mixed_corners;
use crate::nose_in_notch_test_meshes;

const RADIUS: f64 = 1.0;

/// Five decimals, which is what the design's own measurements are quoted to.
const AS_MEASURED: f64 = 5e-5;

const EXACT: f64 = 1e-9;

/// Cos 45 rounded to five places: the tolerances below were set against this value.
#[allow(clippy::approx_constant)]
const CS_COS_45: f64 = 0.70711;

/// The ball at turn zero is a ball of the BEAD: its centre stands on the bead's own axis, so the
/// canal leaves the bead's SQUARE end section tangentially.
#[test]
fn turn_zero_is_a_ball_of_the_bead_itself() {
    let (spine, corner) = the_dual_spine();
    let at = spine.centre_at(0.0);

    assert!(
        at.equals_within(Vector3::new(7.10759, 5.69337, 11.66410), AS_MEASURED),
        "instead {at:?}"
    );

    let off = corner.axis().distance_to(at);
    assert!(
        off < EXACT,
        "which is a point of the BEAD's own axis - instead {at:?} stands {off} off it"
    );

    let step = 1e-6;
    let tangent = (spine.centre_at(step) - spine.centre_at(-step)).get_normal();
    let direction = corner.axis().direction;

    assert!(
        tangent.equals_within(-direction, 1e-6),
        "and the spine leaves it ALONG that axis, back down the bead - instead {tangent:?} against {direction:?}"
    );

    // And the ball touches the cut's tube where the two cylinders touch each other.
    assert!(
        spine
            .bead_contact_at(0.0)
            .equals_within(corner.tangent_point, EXACT),
        "instead {:?} against {:?}",
        spine.bead_contact_at(0.0),
        corner.tangent_point
    );
}

/// The centre path is the ELLIPSE the plane a radius OUTSIDE the wall cuts the cylinder of radius
/// `2r` about the CUT's centre line in: semi-axes `2r` and `2r / 0.70711`.
#[test]
fn the_centre_path_is_the_ellipse_the_wall_cuts_the_cuts_cylinder_in() {
    let (spine, corner) = the_dual_spine();
    let wall = corner.out_of_the_wall;

    let centre = (spine.centre_at(0.0) + spine.centre_at(PI)) / 2.0;

    let mut longest = 0.0f64;
    let mut shortest = f64::MAX;

    for station in 0..3600 {
        let at = spine.centre_at(f64::from(station) * TAU / 3600.0);
        let out_of_the_centre = (at - centre).length();

        longest = longest.max(out_of_the_centre);
        shortest = shortest.min(out_of_the_centre);
    }

    assert!(
        (longest - 2.0 * RADIUS / CS_COS_45).abs() <= 1e-4,
        "the long semi-axis is 2r over how square the cut stands to the wall - instead {longest}"
    );
    assert!(
        (shortest - 2.0 * RADIUS).abs() <= 1e-4,
        "and the short one is the cylinder's own 2r - instead {shortest}"
    );

    // The two conditions themselves, over the arc the blend uses.
    for station in 0..=32 {
        let at = spine.centre_at(f64::from(station) * spine.turn_radians() / 32.0);

        assert!(
            ((at - corner.corner).dot(wall) - RADIUS).abs() <= EXACT,
            "every centre of it stands a radius OUTSIDE the wall - {at:?} does not"
        );
        assert!(
            (corner.cut_center_line().distance_to(at) - 2.0 * RADIUS).abs() <= EXACT,
            "and 2r from the cut's centre line - {at:?} does not"
        );
    }
}

/// The rail the ball leaves on the WALL crosses BOTH creases of the nose while it turns - the upper
/// one at 45.88 degrees and the lower one at 66.74.
#[test]
fn the_wall_rail_crosses_both_creases_of_the_nose() {
    let (spine, corner) = the_dual_spine();

    let upper = the_turn_where_the_wall_rail_crosses(
        &spine,
        nose_in_notch_test_meshes::upper_nose_normal(),
        40.0,
        50.0,
    );
    let lower = the_turn_where_the_wall_rail_crosses(
        &spine,
        nose_in_notch_test_meshes::lower_nose_normal(),
        60.0,
        70.0,
    );

    assert!(
        (upper.to_degrees() - 45.88377).abs() <= 1e-4,
        "instead {}",
        upper.to_degrees()
    );
    assert!(
        (lower.to_degrees() - 66.73609).abs() <= 1e-4,
        "instead {}",
        lower.to_degrees()
    );

    assert!(
        spine
            .t_contact_at(upper)
            .equals_within(Vector3::new(5.54290, 5.54290, 10.36194), AS_MEASURED),
        "instead {:?}",
        spine.t_contact_at(upper)
    );
    assert!(
        spine
            .t_contact_at(lower)
            .equals_within(Vector3::new(5.54290, 5.54290, 9.63806), AS_MEASURED),
        "instead {:?}",
        spine.t_contact_at(lower)
    );

    for station in 0..=32 {
        let at = spine.t_contact_at(f64::from(station) * spine.turn_radians() / 32.0);

        assert!(
            (at - corner.corner).dot(corner.out_of_the_wall).abs() <= EXACT,
            "and every station of that rail stands ON the wall - {at:?} does not"
        );
    }
}

/// The turn ends where the CUT's own strip does, 112.62 degrees on, with the ball touching the
/// cut's tube at the point the cut's end ring is sprung from on the cap.
#[test]
fn the_turn_ends_on_the_cuts_own_tube_contact() {
    let (spine, corner) = the_dual_spine();

    assert!(
        (spine.turn_radians().to_degrees() - 112.6199).abs() <= 1e-3,
        "the dual turns the cut strip's own sweep - instead {}",
        spine.turn_radians().to_degrees()
    );

    let end = spine.bead_contact_at(spine.turn_radians());
    assert!(
        end.equals_within(Vector3::new(7.10759, 6.24808, 9.16795), AS_MEASURED),
        "instead {end:?}"
    );

    for station in 0..=32 {
        let turn = f64::from(station) * spine.turn_radians() / 32.0;
        let at = spine.bead_contact_at(turn);

        assert!(
            (corner.cut_center_line().distance_to(at) - RADIUS).abs() <= EXACT,
            "every station of that contact lies on the cut's own cylinder - {at:?} does not"
        );
        assert!(
            ((at - spine.centre_at(turn)).length() - RADIUS).abs() <= EXACT,
            "and on the ball at its own station - {at:?} does not"
        );
    }
}

/// The ring's SWEEP is V shaped rather than closing: it narrows from the bead's own arc at either
/// end of the turn to 45 degrees at the middle.
#[test]
fn the_rings_sweep_narrows_to_forty_five_degrees_mid_turn() {
    let (spine, _) = the_dual_spine();

    assert!(
        (swept_at(&spine, 0.0) - 66.90653).abs() <= 1e-4,
        "instead {}",
        swept_at(&spine, 0.0)
    );
    assert!(
        (swept_at(&spine, spine.turn_radians()) - 66.90653).abs() <= 1e-4,
        "instead {}",
        swept_at(&spine, spine.turn_radians())
    );

    let mut narrowest = 180.0;
    let mut narrowest_at = 0.0;

    for station in 0..=1000 {
        let turn = f64::from(station) * spine.turn_radians() / 1000.0;

        if swept_at(&spine, turn) < narrowest {
            narrowest = swept_at(&spine, turn);
            narrowest_at = turn;
        }
    }

    assert!(
        (narrowest - 45.0).abs() <= 1e-4,
        "instead the narrowest ring sweeps {narrowest}"
    );
    assert!(
        (f64::to_degrees(narrowest_at) - 56.30993).abs() <= 1e-2,
        "and it is the mid station of the turn - instead {}",
        f64::to_degrees(narrowest_at)
    );
}

/// The curve the ball traces on the CUT's tube is PLANAR, leaning 63.43 degrees across the cut,
/// dipping to `x = 6.21699` at mid turn and back to `x = 7.10759` at both ends.
#[test]
fn the_cuts_end_ring_is_planar_and_leans_across_the_cut() {
    let (spine, corner) = the_dual_spine();
    let plane = spine.tube_ring_plane();

    // The design quotes the plane's normal as (-1, 2, 0) / sqrt(5); a band end faces the way its
    // own edge runs away from the corner, which on this corner is the other sign of the same plane.
    assert!(
        plane
            .inward
            .equals_within(Vector3::new(1.0, -2.0, 0.0).get_normal(), AS_MEASURED),
        "instead {:?}",
        plane.inward
    );

    let leaning = plane
        .inward
        .dot(corner.cut_center_line().direction)
        .abs()
        .clamp(-1.0, 1.0)
        .acos()
        .to_degrees();

    assert!(
        (leaning - 63.43495).abs() <= 1e-4,
        "the cut's square section faces along the cut - instead the ring leans {leaning}"
    );

    assert!(
        plane.point.equals_within(corner.tangent_point, EXACT),
        "the ring stands through the point the two cylinders touch at - instead {:?}",
        plane.point
    );

    let mut lowest = f64::MAX;
    let mut lowest_at = 0.0;

    for station in 0..=1000 {
        let turn = f64::from(station) * spine.turn_radians() / 1000.0;
        let at = spine.bead_contact_at(turn);

        assert!(
            plane.signed_distance(at).abs() < 1e-12,
            "the cut's contact curve is planar, and {at:?} stands off it"
        );

        if at.x < lowest {
            lowest = at.x;
            lowest_at = turn;
        }
    }

    assert!(
        (lowest - 6.21699).abs() <= AS_MEASURED,
        "instead the ring reaches back to {lowest}"
    );
    assert!(
        (f64::to_degrees(lowest_at) - 56.30993).abs() <= 1e-2,
        "at mid turn - instead {}",
        f64::to_degrees(lowest_at)
    );

    assert!(
        (spine.bead_contact_at(0.0).x - 7.10759).abs() <= AS_MEASURED,
        "instead {}",
        spine.bead_contact_at(0.0).x
    );
    assert!(
        (spine.bead_contact_at(spine.turn_radians()).x - 7.10759).abs() <= AS_MEASURED,
        "instead {}",
        spine.bead_contact_at(spine.turn_radians()).x
    );
}

/// The dual of a cut running IN the wall has no spine at all. Driven straight at the spine.
#[test]
fn a_cut_lying_in_the_wall_is_refused() {
    let read = SurfaceMixedCornerSpine::try_create(
        SurfaceLine::new(Vector3::ZERO, Vector3::new(1.0, 0.0, 0.0)),
        Vector3::ZERO,
        Vector3::new(0.0, 0.0, 1.0),
        0.0,
        true,
        Vector3::new(0.0, 1.0, 0.0),
        Vector3::new(0.0, 0.0, 1.0),
        RADIUS,
        PI / 2.0,
        Vector3::new(1.0, 0.0, 0.0),
        1e-6,
    );

    assert!(
        read.is_none(),
        "a cut lying in the wall names no ellipse, and a spine that divided by that would be all infinities"
    );
}

/// How wide the ball's own ring is at `turn`, in degrees: the angle at the centre between the
/// wall's contact and the cut tube's.
fn swept_at(spine: &SurfaceMixedCornerSpine, turn: f64) -> f64 {
    let centre = spine.centre_at(turn);
    let to_the_wall = (spine.t_contact_at(turn) - centre).get_normal();
    let to_the_tube = (spine.bead_contact_at(turn) - centre).get_normal();

    to_the_wall
        .dot(to_the_tube)
        .clamp(-1.0, 1.0)
        .acos()
        .to_degrees()
}

/// Where in the turn the ball's rail on the wall crosses the plane of `face`, by bisection between
/// the two bounds in degrees.
fn the_turn_where_the_wall_rail_crosses(
    spine: &SurfaceMixedCornerSpine,
    face: Vector3,
    from_degrees: f64,
    to_degrees: f64,
) -> f64 {
    let off = |turn: f64| {
        (spine.t_contact_at(turn) - nose_in_notch_test_meshes::left_ridge_end()).dot(face)
    };

    let mut from = from_degrees.to_radians();
    let mut to = to_degrees.to_radians();
    let above = off(from) > 0.0;

    for _ in 0..200 {
        let middle = (from + to) / 2.0;

        if (off(middle) > 0.0) == above {
            from = middle;
        } else {
            to = middle;
        }
    }

    (from + to) / 2.0
}

/// The same corner `TheObliqueCorner` reads, with the spine's roles SWAPPED - through production's
/// own `try_read_the_dual_canal`.
fn the_dual_spine() -> (SurfaceMixedCornerSpine, Rc<SurfaceMixedCorner>) {
    let corner = nose_in_notch_mixed_corners::at_the_ridge_end(
        nose_in_notch_test_meshes::left_ridge_end(),
        RADIUS,
    );

    let dual = corner.try_read_the_dual_canal().unwrap_or_else(|failure| {
        panic!("the dual canal of the oblique corner would not read: {failure:?}")
    });

    (dual.spine, corner)
}
