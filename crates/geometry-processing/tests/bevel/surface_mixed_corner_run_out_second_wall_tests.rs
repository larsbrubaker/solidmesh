//! Port of `SurfaceMixedCornerRunOutSecondWallTests`: what the leaning mixed corner's run-out does
//! when it reaches the end of the crease it was going to die on - measured, because the design
//! question is whether the ball can be CARRIED across the notch's own centre crease onto the second
//! slope and run out there. It cannot: the two ellipses cross where the ball is pinched in the
//! valley between the slopes.

use std::rc::Rc;

use geometry_processing::bevel::surface::{
    SurfaceMixedCorner, SurfaceMixedCornerDualCanal, SurfaceMixedCornerSpine,
};
use vector_math::Vector3;

use crate::nose_in_notch_mixed_corners;
use crate::nose_in_notch_test_meshes;

/// How far the crease the run-out dies on actually runs, corner to (10, 10, 20/3).
const THE_CREASES_OWN_LENGTH: f64 = 7.81736;

/// The run-out's sheet stands clear of the second slope up to exactly the size the crease runs out
/// at, and no further.
fn the_sheet_enters_the_second_slope_exactly_when_the_foot_runs_past_the_crease(
    radius: f64,
    deepest: f64,
) {
    let canal = the_canal_at(radius);
    let measured = deepest_into_the_second_slope(&canal);

    assert!(
        (measured - deepest).abs() <= 5e-5,
        "at r {radius} the sheet stands {measured:.5} inside the second slope"
    );

    let overshoot = (canal.reach_along_the_crease() - THE_CREASES_OWN_LENGTH) * 0.904534;
    assert!(
        (measured - overshoot).abs() <= 1e-4,
        "and that depth is the reach's own overshoot of the crease, read across the slope - {measured} against {overshoot}"
    );

    assert_eq!(
        measured > 0.0,
        canal.reach_along_the_crease() > THE_CREASES_OWN_LENGTH,
        "so the two questions are one question, and the refusal is measuring the gouge"
    );
}

#[test]
fn the_sheet_enters_the_second_slope_exactly_when_the_foot_runs_past_the_crease_1_0() {
    the_sheet_enters_the_second_slope_exactly_when_the_foot_runs_past_the_crease(1.0, -2.92414);
}

#[test]
fn the_sheet_enters_the_second_slope_exactly_when_the_foot_runs_past_the_crease_1_6() {
    the_sheet_enters_the_second_slope_exactly_when_the_foot_runs_past_the_crease(1.6, -0.43599);
}

#[test]
fn the_sheet_enters_the_second_slope_exactly_when_the_foot_runs_past_the_crease_1_7() {
    the_sheet_enters_the_second_slope_exactly_when_the_foot_runs_past_the_crease(1.7, -0.02130);
}

#[test]
fn the_sheet_enters_the_second_slope_exactly_when_the_foot_runs_past_the_crease_1_705() {
    the_sheet_enters_the_second_slope_exactly_when_the_foot_runs_past_the_crease(1.705, -0.00056);
}

#[test]
fn the_sheet_enters_the_second_slope_exactly_when_the_foot_runs_past_the_crease_1_75() {
    the_sheet_enters_the_second_slope_exactly_when_the_foot_runs_past_the_crease(1.75, 0.18605);
}

#[test]
fn the_sheet_enters_the_second_slope_exactly_when_the_foot_runs_past_the_crease_2_0() {
    the_sheet_enters_the_second_slope_exactly_when_the_foot_runs_past_the_crease(2.0, 1.22278);
}

/// The ball cannot be carried onto the second slope: at `r = 2` it first touches that slope at
/// 125.30216 degrees of turn, wedged in the valley, and the second slope's own branch of the canal
/// leaves that crossing back along the cut.
#[test]
fn the_ball_is_pinched_in_the_valley_before_it_can_be_carried_across() {
    let radius = 2.0;

    let corner = nose_in_notch_mixed_corners::at_the_ridge_end(
        nose_in_notch_test_meshes::left_ridge_end(),
        radius,
    );
    let canal = the_canal_at(radius);
    let second = the_second_slopes_branch(&corner, &canal);

    let slope2 = nose_in_notch_test_meshes::right_slope_normal();
    let on_slope2 = Vector3::new(20.0, 0.0, 0.0);

    let spine = &canal.dual.spine;
    let clear = |turn: f64| (spine.centre_at(turn) - on_slope2).dot(slope2) - radius;

    let graze = bisect(&clear, canal.the_cut_strip_ends_at(), canal.dies_at);

    assert!(
        (graze.to_degrees() - 125.30216).abs() <= 1e-4,
        "instead {:.5}",
        graze.to_degrees()
    );

    assert!(
        (spine.centre_at(graze) - Vector3::new(10.0, 7.17157, 6.26590)).length() < 1e-4,
        "the pinched ball's centre stands at {:?}",
        spine.centre_at(graze)
    );

    assert!(
        (canal.row_runs_to(graze).to_degrees() - 51.099).abs() <= 1e-2,
        "and the row it is drawing there is nowhere near closed - {}",
        canal.row_runs_to(graze).to_degrees()
    );

    // The second slope's own branch passes through that same centre - and only that one.
    assert!(
        (second.centre_at(graze) - spine.centre_at(graze)).length() < 1e-6,
        "the two ellipses cross where the ball touches both slopes at once"
    );

    let foot_runs_off = bisect(
        &|turn: f64| 10.0 - spine.t_contact_at(turn).x,
        graze,
        canal.dies_at,
    );

    assert!(
        (foot_runs_off.to_degrees() - 146.0268).abs() <= 1e-3,
        "the first branch's foot only reaches the centre crease 20.7 degrees later - {}",
        foot_runs_off.to_degrees()
    );

    let apart = (second.centre_at(foot_runs_off) - spine.centre_at(foot_runs_off)).length();
    assert!(
        (apart - 2.82843).abs() <= 1e-4,
        "by which turn the second branch's ball stands a whole diameter away along the cut - {apart}"
    );

    // And where the second branch's own foot returns to a crease, its row is still open.
    let dies_on_the_centre_crease = second.t_contact_at(foot_runs_off);

    assert!(
        (dies_on_the_centre_crease - Vector3::new(10.0, 10.0, 6.0)).length() < 1e-4,
        "instead {dies_on_the_centre_crease:?}"
    );

    let off_the_cap = (dies_on_the_centre_crease - corner.corner).dot(corner.out_of_the_cap_face);
    assert!(
        (off_the_cap - 0.55470).abs() <= 1e-4,
        "and that point is not in the cap's plane, so nothing closes the second sheet there - {off_the_cap}"
    );
}

/// The corner's canal read with no crease to die on, so the geometry can be asked for whole.
fn the_canal_at(radius: f64) -> SurfaceMixedCornerDualCanal {
    let corner = nose_in_notch_mixed_corners::at_the_ridge_end(
        nose_in_notch_test_meshes::left_ridge_end(),
        radius,
    );

    SurfaceMixedCornerDualCanal::try_read(&corner, f64::INFINITY)
        .unwrap_or_else(|why| panic!("the canal would not read at r {radius}: {why:?}"))
}

/// The same canal's ball read against the SECOND slope - `2r` from the cut's axis still, but a
/// radius outside that slope instead of this one.
fn the_second_slopes_branch(
    corner: &Rc<SurfaceMixedCorner>,
    canal: &SurfaceMixedCornerDualCanal,
) -> SurfaceMixedCornerSpine {
    let axis = corner.cut_center_line();
    let slope2 = nose_in_notch_test_meshes::right_slope_normal();
    let on_slope2 = Vector3::new(20.0, 0.0, 0.0);
    let along_the_cut_out_of_it = axis.direction.dot(slope2);
    let rail_centre = axis.point
        + (axis.direction * ((on_slope2 - axis.point).dot(slope2) / along_the_cut_out_of_it));

    let towards_the_shared_face = corner.out_of_the_shared_face();
    let towards_the_cap = corner.out_of_the_cap_face;
    let square =
        towards_the_cap - (towards_the_shared_face * towards_the_cap.dot(towards_the_shared_face));

    SurfaceMixedCornerSpine::try_create(
        axis,
        rail_centre,
        slope2,
        along_the_cut_out_of_it,
        true,
        towards_the_shared_face,
        square.get_normal(),
        corner.size,
        canal.dual.turn_radians,
        SurfaceMixedCorner::away_from_the_corner(&corner.cut, corner.corner),
        1e-6,
    )
    .expect("the second slope's branch would not read")
}

/// How far the deepest point of the whole published run-out sheet stands INSIDE the second slope's
/// plane - negative while the sheet is clear of it.
fn deepest_into_the_second_slope(canal: &SurfaceMixedCornerDualCanal) -> f64 {
    let mut deepest = f64::MIN;

    for station in 0..=1000 {
        let turn = canal.the_cut_strip_ends_at()
            + ((canal.dies_at - canal.the_cut_strip_ends_at()) * f64::from(station) / 1000.0);
        let runs_to = canal.row_runs_to(turn);

        for along in 0..=200 {
            let point = canal.on_the_ring(turn, runs_to * f64::from(along) / 200.0);

            deepest = deepest.max((point.x + point.y - 20.0) / 2f64.sqrt());
        }
    }

    deepest
}

/// The one sign change of `of` between the two turns, to the bit.
fn bisect(of: &dyn Fn(f64) -> f64, from: f64, to: f64) -> f64 {
    let sign = of(from) > 0.0;
    let (mut from, mut to) = (from, to);

    for _ in 0..200 {
        let mid = (from + to) / 2.0;

        if (of(mid) > 0.0) == sign {
            from = mid;
        } else {
            to = mid;
        }
    }

    (from + to) / 2.0
}
