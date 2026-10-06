//! Port of `SectionLoopEntryTests.cs`: the run built from a loop of vertices equals the run built
//! from the chain, and a ring that is not a chain of creases is refused.

use geometry_processing::bevel::sections::section_builder_run::{try_build_chain, try_build_loop};
use geometry_processing::bevel::sections::{bevel_section::SectionSide, SectionPool};
use geometry_processing::bevel::BevelAdvisoryCode;
use geometry_processing::topology::{FeatureEdgeChain, FeatureEdgeSet};
use vector_math::Vector3;

use crate::bore_test_meshes::create_plate_with_a_round_bore;

const SIDES: i32 = 24;
const BORE_RADIUS: f64 = 3.0;
const THICKNESS: f64 = 10.0;
const RADIUS: f64 = 0.4;
const SEGMENTS: i32 = 8;
const BEND_THRESHOLD_DEGREES: f64 = 20.0;

/// C# `TheRunFromTheLoopEqualsTheRunFromTheChain`.
#[test]
fn the_run_from_the_loop_equals_the_run_from_the_chain() {
    let part = create_plate_with_a_round_bore(SIDES, BORE_RADIUS, THICKNESS, 6.0);
    let graph = part.get_mesh_edge_graph();
    let features = FeatureEdgeSet::create_with_graph(&part, &graph, BEND_THRESHOLD_DEGREES);
    let chain = FeatureEdgeChain::propagate(&features, the_upper_mouth(&features)).unwrap();
    assert!(chain.is_closed());
    assert_eq!(chain.count(), SIDES as usize);

    let mut from_chain_pool = SectionPool::new(&part.vertices);
    let from_chain = try_build_chain(
        Some(&chain),
        &part,
        &graph,
        &mut from_chain_pool,
        RADIUS,
        SEGMENTS,
        true,
    )
    .unwrap_or_else(|why| panic!("{:?}", why));

    let ring: Vec<i32> = chain
        .edges()
        .iter()
        .map(|edge| edge.from_vertex())
        .collect();
    let mut from_loop_pool = SectionPool::new(&part.vertices);
    let from_loop = try_build_loop(
        &ring,
        Some(&features),
        &part,
        &graph,
        &mut from_loop_pool,
        RADIUS,
        SEGMENTS,
        true,
    )
    .unwrap_or_else(|why| panic!("{:?}", why));

    assert_eq!(from_loop.sections.len(), from_chain.sections.len());
    assert!(from_loop.is_closed);
    let mut worst: f64 = 0.0;
    for (expected, actual) in from_chain.sections.iter().zip(&from_loop.sections) {
        assert_eq!(actual.chain_vertex, expected.chain_vertex);
        assert_eq!(actual.left_kind, expected.left_kind);
        assert_eq!(expected.right_kind, SectionSide::Slide);
        assert_eq!(actual.right_kind, SectionSide::Offset);
        assert_eq!(actual.profile.len(), expected.profile.len());
        assert!((actual.setback - expected.setback).abs() < 1e-12);
        assert!((actual.station - expected.station).length() < 1e-12);
        assert!((actual.tangent - expected.tangent).length() < 1e-12);
        for k in 0..expected.profile.len() {
            let apart = (from_loop_pool.position_of(actual.profile[k])
                - from_chain_pool.position_of(expected.profile[k]))
            .length();
            worst = worst.max(apart);
        }
    }
    assert!(worst < 1e-12, "{worst}");
}

/// C# `ARingThatIsNotAChainOfCreasesIsRefused`.
#[test]
fn a_ring_that_is_not_a_chain_of_creases_is_refused() {
    let part = create_plate_with_a_round_bore(SIDES, BORE_RADIUS, THICKNESS, 6.0);
    let graph = part.get_mesh_edge_graph();
    let features = FeatureEdgeSet::create_with_graph(&part, &graph, BEND_THRESHOLD_DEGREES);
    let chain = FeatureEdgeChain::propagate(&features, the_upper_mouth(&features)).unwrap();
    let skipping: Vec<i32> = chain
        .edges()
        .iter()
        .step_by(2)
        .map(|edge| edge.from_vertex())
        .collect();

    let Err(why) = try_build_loop(
        &skipping,
        Some(&features),
        &part,
        &graph,
        &mut SectionPool::new(&part.vertices),
        RADIUS,
        SEGMENTS,
        true,
    ) else {
        panic!("a ring that skips vertices is no chain of creases");
    };
    assert_eq!(
        why.advisory().map(|a| a.code()),
        Some(BevelAdvisoryCode::SurfaceConstructionNotBuilt)
    );
}

fn the_upper_mouth(features: &FeatureEdgeSet) -> i32 {
    features
        .edges()
        .iter()
        .position(|edge| on_the_mouth(edge.start) && on_the_mouth(edge.end))
        .expect("the fixture has no upper mouth") as i32
}

fn on_the_mouth(at: Vector3) -> bool {
    (at.z - (THICKNESS / 2.0)).abs() < 1e-6
        && (Vector3::new(at.x, at.y, 0.0).length() - BORE_RADIUS).abs() < 1e-4
}
