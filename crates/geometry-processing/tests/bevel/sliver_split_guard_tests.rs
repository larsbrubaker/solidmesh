//! Port of `LevelLine/SliverSplitGuardTests.cs`: the refinement rules' sliver guard
//! (`refinement_splits::would_thin_a_child*`, `refinement_splits_enqueue::bisect_instead_of_thinning*`):
//! a split that would leave a child far thinner than its parent bisects that child's longest-edge
//! path instead.
//!
//! `TheGuardWouldFireOnTheLandAndNowhereOnThePinnedFixtures` reads its builds through
//! `PointBevelTests.BuildForMeter` (the point cone and cube rows) and lands with that class's port.

use std::collections::{HashMap, VecDeque};
use std::rc::Rc;

use vector_math::Vector3;

use geometry_processing::bevel::level_line::refinement_splits::{
    would_thin_a_child, would_thin_a_child_on_edge,
};
use geometry_processing::bevel::level_line::refinement_splits_enqueue::{
    bisect_instead_of_thinning_on_edge, SLIVER_ASPECT_LIMIT,
};
use geometry_processing::bevel::level_line::{FieldCounters, LevelLineMesh, LevelLineRule};

const LIMIT: f64 = SLIVER_ASPECT_LIMIT;

#[test]
fn splitting_a_strip_well_inside_its_long_edge_thins_a_child() {
    // The child (P, 1, 2) keeps the 16.008 edge 1-2 with 0.3 of the area: aspect 107 against the
    // parent's 32.
    assert!(would_thin_a_child_on_edge(
        &strip(),
        0,
        1,
        Vector3::new(11.2, 0.0, 0.0),
        LIMIT,
        0.0
    ));
}

#[test]
fn splitting_a_well_shaped_triangle_at_a_midpoint_does_not() {
    assert!(!would_thin_a_child_on_edge(
        &well_shaped(),
        0,
        1,
        Vector3::new(2.0, 0.0, 0.0),
        LIMIT,
        0.0
    ));
}

#[test]
fn a_nick_within_one_over_the_limit_of_an_end_is_left_to_the_rules() {
    // Half of 1/64 of the way in from vertex 1: the child (P, 1, 2) is a real sliver, but that is
    // what any crossing near a corner makes, and it is not the regress.
    let near_the_end = Vector3::new(16.0 * (1.0 - (0.5 / LIMIT)), 0.0, 0.0);
    assert!(!would_thin_a_child_on_edge(
        &strip(),
        0,
        1,
        near_the_end,
        LIMIT,
        0.0
    ));
}

#[test]
fn a_triangle_no_longer_than_the_gate_is_not_guarded() {
    assert!(!would_thin_a_child_on_edge(
        &strip(),
        0,
        1,
        Vector3::new(11.2, 0.0, 0.0),
        LIMIT,
        17.0
    ));
}

#[test]
fn every_triangle_on_the_edge_is_asked() {
    // The strip's long edge 0-1 is shared with a well-shaped triangle below it, whose children at
    // 0.7 are fine; the strip's are not, so the split is guarded.
    let mesh = LevelLineMesh::from_triangles(
        &[
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(16.0, 0.0, 0.0),
            Vector3::new(0.0, 0.5, 0.0),
            Vector3::new(8.0, -14.0, 0.0),
        ],
        &[(0, 1, 2), (1, 0, 3)],
        None,
    )
    .expect("the mesh builds");

    assert!(would_thin_a_child_on_edge(
        &mesh,
        0,
        1,
        Vector3::new(11.2, 0.0, 0.0),
        LIMIT,
        0.0
    ));
}

#[test]
fn an_interior_point_is_asked_of_its_three_star_children() {
    // 0.01 over the strip's long edge: star child (0, 1, P) is 16 long and 0.01 high. That is
    // farther than the triangle's own millionth from the edge, so it is an interior insertion.
    assert!(would_thin_a_child(
        &strip(),
        0,
        Vector3::new(4.0, 0.01, 0.0),
        LIMIT,
        0.0
    ));

    // The centroid of a well-shaped triangle makes three well-shaped children.
    assert!(!would_thin_a_child(
        &well_shaped(),
        0,
        Vector3::new(2.0, 3.5 / 3.0, 0.0),
        LIMIT,
        0.0
    ));

    // A point on an edge is asked as that edge's split, and at a corner nothing is minted.
    assert!(would_thin_a_child(
        &strip(),
        0,
        Vector3::new(11.2, 0.0, 0.0),
        LIMIT,
        0.0
    ));
    assert!(!would_thin_a_child(
        &strip(),
        0,
        Vector3::new(16.0, 0.0, 0.0),
        LIMIT,
        0.0
    ));
}

#[test]
fn the_guard_leaves_a_triangle_of_another_tag_alone() {
    // Edge 0-1 is a fold: the strip above it is tag 1, the well-shaped triangle below is tag 0. A
    // split at 0.7 thins only the strip, so it is guarded when the strip asks and not when the
    // tag-0 triangle does.
    let fold = || {
        LevelLineMesh::from_triangles(
            &[
                Vector3::new(0.0, 0.0, 0.0),
                Vector3::new(16.0, 0.0, 0.0),
                Vector3::new(0.0, 0.5, 0.0),
                Vector3::new(8.0, -14.0, 0.0),
            ],
            &[(0, 1, 2), (1, 0, 3)],
            Some(&[1, 0]),
        )
        .expect("the mesh builds")
    };
    let split = Vector3::new(11.2, 0.0, 0.0);

    let mut from_below = fold();
    assert!(!bisect_instead_of_thinning_on_edge(
        &mut from_below,
        1,
        0,
        1,
        split,
        0.0,
        &mut VecDeque::new(),
        &mut HashMap::new()
    )
    .expect("the guard reads"));
    assert_eq!(from_below.vertex_count(), 4);

    let mut from_above = fold();
    assert!(bisect_instead_of_thinning_on_edge(
        &mut from_above,
        0,
        0,
        1,
        split,
        0.0,
        &mut VecDeque::new(),
        &mut HashMap::new()
    )
    .expect("the guard reads"));
    assert_eq!(from_above.vertex_count(), 5);
}

/// Switches the per-rule meter back off however the test leaves (C#'s `finally`).
struct MeterOff;

impl Drop for MeterOff {
    fn drop(&mut self) {
        FieldCounters::set_meter_on(false);
    }
}

#[test]
fn the_guard_answers_false_when_the_bisection_divides_nothing() {
    // The strip's long edge is the pick, so the bisection cannot start and the rule must split as
    // it asked.
    let mut mesh = strip();
    mesh.is_chain_edge = Some(Rc::new(|a, b| (a == 1 && b == 2) || (a == 2 && b == 1)));
    let mut pending = VecDeque::new();

    FieldCounters::reset();
    FieldCounters::set_meter_on(true);
    {
        let _off = MeterOff;
        assert!(!bisect_instead_of_thinning_on_edge(
            &mut mesh,
            0,
            0,
            1,
            Vector3::new(11.2, 0.0, 0.0),
            0.0,
            &mut pending,
            &mut HashMap::new()
        )
        .expect("the guard reads"));
    }

    assert_eq!(mesh.vertex_count(), 3);
    assert_eq!(pending.len(), 0);
    assert_eq!(FieldCounters::fired_count(LevelLineRule::SliverGuard), 0);
}

fn strip() -> LevelLineMesh {
    LevelLineMesh::from_triangles(
        &[
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(16.0, 0.0, 0.0),
            Vector3::new(0.0, 0.5, 0.0),
        ],
        &[(0, 1, 2)],
        None,
    )
    .expect("the strip builds")
}

fn well_shaped() -> LevelLineMesh {
    LevelLineMesh::from_triangles(
        &[
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(4.0, 0.0, 0.0),
            Vector3::new(2.0, 3.5, 0.0),
        ],
        &[(0, 1, 2)],
        None,
    )
    .expect("the triangle builds")
}
