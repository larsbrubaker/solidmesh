//! Port of `BevelRunPartnersTests.cs`: a run's partners are the runs it shares an END vertex with,
//! read as the straight tube each one's ball sweeps.

use geometry_processing::bevel::bevel_run_partners::{for_chain, try_ball_centre_line};
use geometry_processing::bevel::{BevelPartnerTube, BevelSweptChain};
use geometry_processing::topology::ChainEdge;
use vector_math::Vector3;

use crate::nose_in_notch_test_meshes as nose;

const RADIUS: f64 = 5.0;

fn index_of(names: &[String], name: &str) -> usize {
    names.iter().position(|n| n == name).unwrap()
}

/// C# `ARunsPartnersAreTheRunsItSharesAnEndVertexWith`.
#[test]
fn a_runs_partners_are_the_runs_it_shares_an_end_vertex_with() {
    let part = nose::create();
    let (runs, names) = nose::creases(&part);

    let upper_left = &runs[index_of(&names, "upper left")];
    let (partners, skipped) = for_chain(upper_left, &runs, RADIUS);

    assert_eq!(
        skipped, 0,
        "every crease of this part is one straight mesh edge"
    );
    assert_eq!(
        partners.len(),
        2,
        "the upper-left run shares the ridge end with the lower-left run and the upper crease end with the upper-right one"
    );

    let lower_left = axis(&runs[index_of(&names, "lower left")]);
    let upper_right = axis(&runs[index_of(&names, "upper right")]);
    let lower_right = axis(&runs[index_of(&names, "lower right")]);

    assert!(
        partners.iter().any(|tube| same(tube, lower_left)),
        "the lower-left run shares the nose's ridge end at (5, 5, 10)"
    );
    assert!(
        partners.iter().any(|tube| same(tube, upper_right)),
        "the upper-right run shares the point (10, 10, 13.333)"
    );
    assert!(
        !partners.iter().any(|tube| same(tube, lower_right)),
        "the lower-right run shares no end with this one"
    );

    let lower_left_run = &runs[index_of(&names, "lower left")];
    let (lower_partners, _) = for_chain(lower_left_run, &runs, RADIUS);

    assert!(
        !lower_partners.iter().any(|tube| same(tube, upper_right)),
        "the lower-left run ends at (10, 10, 6.667) and the upper-right one at (10, 10, 13.333)"
    );
}

/// C# `TheBallCentreLineIsWhereTheControlsAxisIs`.
#[test]
fn the_ball_centre_line_is_where_the_controls_axis_is() {
    let part = nose::create();
    let (runs, names) = nose::creases(&part);

    let (axis_point, direction) =
        try_ball_centre_line(&runs[index_of(&names, "upper left")], RADIUS).expect("built");

    let control = Vector3::new(7.5395, 0.4684, 12.9882);
    let offset = axis_point - control;
    let across = offset - (direction * offset.dot(direction));

    assert!(
        across.length() < 0.005,
        "a ball of radius 5 against both faces of this crease is on the control's measured axis: {}",
        across.length()
    );
    assert!(
        (direction
            .dot(Vector3::new(0.6396, 0.6396, 0.4264).get_normal())
            .abs()
            - 1.0)
            .abs()
            <= 1e-3,
        "the ball rolls along the crease"
    );
}

/// C# `ARunThatTurnsIsSkippedRatherThanApproximated`.
#[test]
fn a_run_that_turns_is_skipped_rather_than_approximated() {
    let part = nose::create();
    let (runs, names) = nose::creases(&part);

    let left = runs[index_of(&names, "upper left")].edges[0];
    let down = runs[index_of(&names, "lower left")].edges[0];
    let bent = BevelSweptChain::new(vec![reversed(&left), down], false);

    assert!(
        try_ball_centre_line(&bent, RADIUS).is_none(),
        "a rolling ball on a run that turns does not travel a straight line"
    );

    let (partners, skipped) = for_chain(
        &runs[index_of(&names, "upper right")],
        std::slice::from_ref(&bent),
        RADIUS,
    );

    assert_eq!(partners.len(), 0);
    assert_eq!(skipped, 1, "the caller has to know it was left out");
}

fn reversed(edge: &ChainEdge) -> ChainEdge {
    ChainEdge::new(edge.edge_index, edge.edge, !edge.is_reversed)
}

fn same(tube: &BevelPartnerTube, axis: (Vector3, Vector3)) -> bool {
    let (point, direction) = axis;
    let offset = tube.axis_point - point;

    ((tube.axis_direction.dot(direction)).abs() - 1.0).abs() < 1e-6
        && (offset - (direction * offset.dot(direction))).length() < 1e-6
}

fn axis(run: &BevelSweptChain) -> (Vector3, Vector3) {
    try_ball_centre_line(run, RADIUS).unwrap_or_default()
}
