//! Ports `Tests/GeometryProcessingTests/BevelSweepBackDepthTests.cs`: the FLOOR under the sweep's
//! one retry - a run that folded at full back depth is offered a shallower back, but only while
//! that back is still deep enough to bury the tool in material.

use geometry_processing::bevel::bevel_chain_linker::{self, BevelSweptChain};
use geometry_processing::bevel::bevel_sweep_back_depth::{
    back_depth_the_turn_allows, retry_depth, turn_radius, MINIMUM_RETRY_FRACTION_OF_SIZE,
};
use geometry_processing::topology::FeatureEdgeSet;
use polygon_mesh::Mesh;

use crate::topology_test_meshes::create_arc_prism;

const RADIUS: f64 = 10.0;

const HEIGHT: f64 = 20.0;

/// The arc's facet count, which puts 9.47 degrees and 1.65 mm in each step at [`RADIUS`].
const ARC_FACETS: i32 = 19;

const SIZE: f64 = 2.0;

#[test]
fn the_tightest_turn_is_what_bounds_the_back() {
    let rim = rim(&create_arc_prism(RADIUS, HEIGHT, ARC_FACETS), HEIGHT);

    let turn = turn_radius(&rim.edges, rim.is_closed);

    assert!(turn > 0.0);
    assert!(
        turn < RADIUS,
        "a chord and its turn read the arc's own radius, a little under it for the chording"
    );

    assert_eq!(
        back_depth_the_turn_allows(&rim.edges, rim.is_closed, SIZE, turn / 2.0, f64::INFINITY),
        0.0,
        "a section that never reaches as far in as the turn allows needs no shallower back at all"
    );
}

#[test]
fn the_retry_takes_the_depth_the_turn_allows_while_it_still_buries_the_tool() {
    let rim = rim(&create_arc_prism(RADIUS, HEIGHT, ARC_FACETS), HEIGHT);

    // A reach a few times what the turn allows, which is the shape of every real fold.
    let reach = 4.0 * turn_radius(&rim.edges, rim.is_closed);

    let allowed = back_depth_the_turn_allows(&rim.edges, rim.is_closed, SIZE, reach, f64::INFINITY);

    assert!(
        allowed > SIZE * MINIMUM_RETRY_FRACTION_OF_SIZE,
        "the fixture means nothing unless this depth is one the retry is allowed to take"
    );

    assert_eq!(
        retry_depth(&rim.edges, rim.is_closed, SIZE, reach, f64::INFINITY),
        allowed,
        "a back deep enough to bury the tool is handed straight to the retry"
    );
}

#[test]
fn a_retry_shallower_than_a_tenth_of_the_size_is_not_made_at_all() {
    let rim = rim(&create_arc_prism(RADIUS, HEIGHT, ARC_FACETS), HEIGHT);

    // Deep enough that the turn can only give back a sliver of it.
    let reach = 100.0 * turn_radius(&rim.edges, rim.is_closed);

    let allowed = back_depth_the_turn_allows(&rim.edges, rim.is_closed, SIZE, reach, f64::INFINITY);

    assert!(
        allowed > 0.0,
        "the turn does allow SOMETHING, which is what made the unfloored retry ask for it"
    );
    assert!(
        allowed < SIZE * MINIMUM_RETRY_FRACTION_OF_SIZE,
        "and what it allows is a wafer rather than a back"
    );

    assert_eq!(
        retry_depth(&rim.edges, rim.is_closed, SIZE, reach, f64::INFINITY),
        0.0,
        "so no retry is made: the sweep reports the reason the user's own size earned"
    );
}

/// The closed run of the top face's boundary edges, linked the way a face group's rim is.
fn rim(part: &Mesh, top: f64) -> BevelSweptChain {
    let features = FeatureEdgeSet::create(part);

    let on_the_top: Vec<_> = features
        .edges()
        .iter()
        .filter(|edge| (edge.start.z - top).abs() < 1e-5 && (edge.end.z - top).abs() < 1e-5)
        .copied()
        .collect();

    let mut chains = bevel_chain_linker::link(&on_the_top);
    assert_eq!(chains.len(), 1, "the top's rim links into one run");
    chains.remove(0)
}
