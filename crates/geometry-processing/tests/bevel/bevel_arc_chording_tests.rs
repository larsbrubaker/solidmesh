//! The one rule that says how finely an arc is chorded, which a run's profile and the corner
//! patch's boundary both have to read the same way or the seam between them does not meet. Each
//! test's rows run as a loop, each row's reason as the assertion message.

use geometry_processing::bevel::bevel_arc_chording::{effective_segments, segments_for};
use geometry_processing::topology::{FeatureEdge, FeatureEdgeSet};
use polygon_mesh::solids::platonic_solids;

use crate::topology_test_meshes::create_isosceles_prism;

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

/// The count one whole OPERATION is built at: the largest any of its arcs asks for, floored at
/// the operation's own number, read off real parts. A prism whose APEX is obtuse still answers
/// 14, because the maximum is over every edge and that prism's back corners are sharper than its
/// apex.
#[test]
fn an_operation_is_built_at_the_count_its_sharpest_arc_asks_for() {
    let rows: [(f64, i32, &str); 3] = [
        (90.0, 8, "a cube: every edge turns a quarter, so nothing moves"),
        (45.0, 12, "a 45 degree knife tip turns through 135, which is a segment and a half per quarter"),
        (135.0, 14, "the SHARPEST arc anywhere in the part decides, and on a 135 degree prism that is not the apex at all - its back corners are 22.5 degrees interior, turning through 157.5, which asks for 14"),
    ];

    for (apex_degrees, expected, why) in rows {
        let part = if apex_degrees == 90.0 {
            platonic_solids::create_cube(20.0, 20.0, 20.0)
        } else {
            create_isosceles_prism(apex_degrees, 20.0, 20.0)
        };

        let features = FeatureEdgeSet::create(&part);
        let edges: Vec<FeatureEdge> = features.edges().to_vec();

        assert_eq!(effective_segments(Some(&edges), EIGHT), expected, "{why}");
    }
}

/// Nothing to read is the operation's own count, not zero.
#[test]
fn no_edges_answer_the_operations_own_count() {
    assert_eq!(effective_segments(None::<&[FeatureEdge]>, EIGHT), EIGHT);
    assert_eq!(
        effective_segments(Some(&[] as &[FeatureEdge]), EIGHT),
        EIGHT
    );
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
