//! The one rule that says how finely an arc is chorded (`bevel_arc_chording::segments_for`). Each
//! test's rows run as a loop, each row's reason as the assertion message.

use solidmesh::bevel::bevel_arc_chording::segments_for;

/// The count a fresh fillet carries, and the one every number below is stated against.
const EIGHT: i32 = 8;

#[test]
fn an_arc_is_chorded_by_its_angle_rather_than_by_a_fixed_count() {
    let rows: [(f64, i32, i32, &str); 6] = [
        (90.0, EIGHT, 8, "a quarter turn IS the unit, so it comes back unchanged - which is what keeps every 90 degree fixture byte for byte where it was"),
        (135.0, EIGHT, 12, "a 45 degree knife tip turns its fillet through 135, and gets half again as many segments"),
        (45.0, EIGHT, 4, "a 135 degree obtuse edge turns through only 45, and does not need eight"),
        (180.0, EIGHT, 16, "a half turn is two quarters"),
        (360.0, EIGHT, 32, "a full circle is four quarters - which is exactly the 4 * segments the ball tube already used"),
        (30.0, EIGHT, 3, "a third of a quarter turn"),
    ];

    for (angle_degrees, per_quarter, expected, why) in rows {
        assert_eq!(segments_for(angle_degrees, per_quarter), expected, "{why}");
    }
}

/// The step an arc is left with, which is the point of the rule: the same everywhere, so the
/// sagitta is too.
#[test]
fn the_step_comes_out_the_same_whatever_the_arc() {
    for angle_degrees in [90.0, 135.0, 45.0] {
        let step = angle_degrees / f64::from(segments_for(angle_degrees, EIGHT));

        assert!(
            (step - 90.0 / f64::from(EIGHT)).abs() <= 1e-9,
            "chording by angle means every arc is stepped at the same number of degrees, so a \
             sharp corner is no longer sampled more coarsely than a blunt one ({angle_degrees}: {step})"
        );
    }
}

/// A sliver of an arc still has to be crossed, and a nonsense count still has to answer
/// something a sweep can use.
#[test]
fn degenerate_inputs_still_answer_a_usable_count() {
    let rows: [(f64, i32, i32); 4] = [(0.1, EIGHT, 1), (1.0, 1, 1), (90.0, 0, 1), (-5.0, EIGHT, 8)];

    for (angle_degrees, per_quarter, expected) in rows {
        assert_eq!(
            segments_for(angle_degrees, per_quarter),
            expected,
            "nothing downstream can sweep a profile of zero segments, so the floor is one \
             ({angle_degrees}, {per_quarter})"
        );
    }
}
