//! Port of `SectionRoutingPartitionTests.cs`: which PLANS of a pass the sections accept
//! (`partition`), where `is_simple` only answers for the pass as a whole. The plate with a round
//! bore is the cheapest shape with both cases: the bore's mouth is closed, and a single edge of the
//! plate's outer rim is an open run. C#'s `NotInParallel` guarded the static last route, which
//! these tests do not read.

use std::rc::Rc;
use std::sync::Arc;

use geometry_processing::bevel::bevel_chain_linker::{self, BevelSweptChain};
use geometry_processing::bevel::sections::section_routing::{is_simple, partition};
use geometry_processing::bevel::surface::SurfaceBevelPlan;
use geometry_processing::topology::{FaceRegionCache, FeatureEdge, FeatureEdgeSet};
use polygon_mesh::Mesh;

use crate::bore_test_meshes::create_plate_with_a_round_bore;

const SIDES: i32 = 24;
const BORE_RADIUS: f64 = 3.0;
const THICKNESS: f64 = 10.0;
/// Small enough that nothing here is about room.
const SIZE: f64 = 0.4;
const SEGMENTS: i32 = 8;

/// C# `AClosedMouthIsAcceptedBesideAnOpenRunThatIsDeclined`: the mouth and one open rim edge in
/// one pass - the partition takes the mouth and names the rim run's own clause, while the pass
/// verdict is still a flat no carrying that same sentence.
#[test]
fn a_closed_mouth_is_accepted_beside_an_open_run_that_is_declined() {
    let plate = the_plate();
    let graph = plate.get_mesh_edge_graph();
    let (regions, features) = regions_of(&plate);

    let plans = vec![
        the_mouth_plan(&features, &regions, THICKNESS / 2.0),
        the_open_rim_edge_plan(&features, &regions),
    ];

    let answer = partition(&plans, Some(&plate), Some(&graph), None);

    assert_eq!(
        answer.accepted,
        vec![0],
        "the bore's mouth is the sections' own case whatever else was picked with it"
    );

    assert_eq!(answer.declined.len(), 1);
    assert_eq!(answer.declined[0].0, 1);
    assert_eq!(answer.declined[0].1, "a chain is not closed");

    let pass = is_simple(&plans, Some(&plate), Some(&graph), SEGMENTS, None).unwrap();
    assert!(
        !pass.is_simple(),
        "the pass verdict is unchanged - one plan it cannot build refuses all of it"
    );
    assert_eq!(pass.why_not(), Some("a chain is not closed"));
}

/// C# `APassOfSimplePlansIsAcceptedWhole`: a pass every plan of which is simple is accepted whole,
/// and says the same thing `is_simple` says about it.
#[test]
fn a_pass_of_simple_plans_is_accepted_whole() {
    let plate = the_plate();
    let graph = plate.get_mesh_edge_graph();
    let (regions, features) = regions_of(&plate);

    let plans = vec![
        the_mouth_plan(&features, &regions, THICKNESS / 2.0),
        the_mouth_plan(&features, &regions, -THICKNESS / 2.0),
    ];

    let answer = partition(&plans, Some(&plate), Some(&graph), None);

    assert_eq!(answer.accepted, vec![0, 1]);
    assert_eq!(answer.declined.len(), 0);

    let pass = is_simple(&plans, Some(&plate), Some(&graph), SEGMENTS, None).unwrap();
    assert!(
        pass.is_simple(),
        "both mouths of the straight bore route - {:?}",
        pass.why_not()
    );
}

/// C# `TwoChainsSharingTheirVerticesAreBothDeclined`: two chains that touch are BOTH declined, not
/// one of them accepted - a vertex where another plan's picked edge arrives is a junction on the
/// part, and the sections have no treatment for one. The mouth handed twice is the cheapest way to
/// make every vertex such a vertex.
#[test]
fn two_chains_sharing_their_vertices_are_both_declined() {
    let plate = the_plate();
    let graph = plate.get_mesh_edge_graph();
    let (regions, features) = regions_of(&plate);

    let plans = vec![
        the_mouth_plan(&features, &regions, THICKNESS / 2.0),
        the_mouth_plan(&features, &regions, THICKNESS / 2.0),
    ];

    let answer = partition(&plans, Some(&plate), Some(&graph), None);

    assert_eq!(
        answer.accepted.len(),
        0,
        "neither chain is simple where the other one runs through its vertices"
    );

    let declined: Vec<usize> = answer.declined.iter().map(|(plan, _)| *plan).collect();
    assert_eq!(declined, vec![0, 1]);

    for (plan, why_not) in &answer.declined {
        assert!(
            why_not.contains("carries 4 picked edges"),
            "plan {plan} should name the crowded vertex - instead {why_not}"
        );
    }
}

fn the_plate() -> Mesh {
    // C#'s `land` defaults to 6.
    create_plate_with_a_round_bore(SIDES, BORE_RADIUS, THICKNESS, 6.0)
}

fn regions_of(mesh: &Mesh) -> (Arc<FaceRegionCache>, FeatureEdgeSet) {
    let features = FeatureEdgeSet::create(mesh);
    let regions = FaceRegionCache::new(
        Arc::new(mesh.clone()),
        Some(mesh.get_mesh_edge_graph()),
        features.flat_angle_degrees(),
    );

    (Arc::new(regions), features)
}

/// The closed loop the bore's mouth makes at the given height, planned as a fillet.
fn the_mouth_plan(
    features: &FeatureEdgeSet,
    regions: &Arc<FaceRegionCache>,
    height: f64,
) -> Rc<SurfaceBevelPlan> {
    let mouth: Vec<FeatureEdge> = features
        .edges()
        .iter()
        .filter(|edge| {
            at_the_height(edge.start.z, height)
                && at_the_height(edge.end.z, height)
                && on_the_bore(edge.start.x, edge.start.y)
                && on_the_bore(edge.end.x, edge.end.y)
        })
        .copied()
        .collect();

    plan_of(&single(bevel_chain_linker::link(&mouth)), regions)
}

/// ONE edge of the plate's outer top rim - a chain of one, which is open, standing in for the wall
/// rim of a face pick.
fn the_open_rim_edge_plan(
    features: &FeatureEdgeSet,
    regions: &Arc<FaceRegionCache>,
) -> Rc<SurfaceBevelPlan> {
    let edge = *features
        .edges()
        .iter()
        .find(|candidate| {
            at_the_height(candidate.start.z, THICKNESS / 2.0)
                && at_the_height(candidate.end.z, THICKNESS / 2.0)
                && !on_the_bore(candidate.start.x, candidate.start.y)
        })
        .expect("the plate has an outer top rim");

    let chain = single(bevel_chain_linker::link(&[edge]));

    assert!(
        !chain.is_closed,
        "one edge of the outer rim is an OPEN chain"
    );

    plan_of(&chain, regions)
}

/// C#'s LINQ `Single()`.
fn single(mut chains: Vec<BevelSweptChain>) -> BevelSweptChain {
    assert_eq!(chains.len(), 1, "exactly one chain");
    chains.remove(0)
}

fn plan_of(chain: &BevelSweptChain, regions: &Arc<FaceRegionCache>) -> Rc<SurfaceBevelPlan> {
    SurfaceBevelPlan::try_create(chain, regions, SIZE, true)
        .unwrap_or_else(|reason| panic!("the chain plans at {SIZE} - instead {reason:?}"))
}

fn at_the_height(z: f64, height: f64) -> bool {
    (z - height).abs() < 1e-3
}

fn on_the_bore(x: f64, y: f64) -> bool {
    (((x * x) + (y * y)).sqrt() - BORE_RADIUS).abs() < 1e-3
}
