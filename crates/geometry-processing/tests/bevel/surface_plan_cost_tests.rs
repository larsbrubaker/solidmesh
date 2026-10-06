//! Port of `SurfacePlanCostTests.cs`: what a plan COSTS, as a sanity rung rather than a benchmark.
//! Planning one chain reads the mesh twice at each free end - once for the edges standing at the
//! corner and once for the surfaces its faces belong to - and the obvious reading of each is a walk
//! of the whole mesh. The reading that matters is the FLOOD COUNT (`FaceRegionCache::flood_count`),
//! which says the same thing on any machine and under any load; the clock beside it is a loose net
//! (two seconds) for the costs a flood count cannot see.

use std::sync::Arc;
use std::time::Instant;

use geometry_processing::bevel::bevel_chain_linker::{self, BevelSweptChain};
use geometry_processing::bevel::surface::SurfaceBevelPlan;
use geometry_processing::topology::{FaceGroup, FaceRegionCache, FeatureEdgeSet};

use crate::topology_test_meshes::create_cylinder;

#[test]
fn planning_three_hundred_rim_edges_does_not_walk_the_whole_mesh_each_time() {
    const SIDES: i32 = 600;
    const CHAINS: usize = 300;

    let cylinder = Arc::new(create_cylinder(10.0, 10.0, SIDES));

    assert!(
        cylinder.faces.len() > 2000,
        "the point of the fixture is that a walk of it is expensive"
    );

    let features = FeatureEdgeSet::create(&cylinder);
    let graph = cylinder.get_mesh_edge_graph();
    let regions = Arc::new(FaceRegionCache::new(
        cylinder.clone(),
        Some(graph.clone()),
        features.flat_angle_degrees(),
    ));
    let curve_regions = Arc::new(FaceRegionCache::new(
        cylinder.clone(),
        Some(graph),
        FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES,
    ));

    let rim: Vec<BevelSweptChain> = (0..features.count())
        .filter(|&i| features.edges()[i].start.z > 4.99 && features.edges()[i].end.z > 4.99)
        .take(CHAINS)
        .map(|i| {
            let mut chains = bevel_chain_linker::link(&[features.edges()[i]]);
            assert_eq!(chains.len(), 1);
            chains.remove(0)
        })
        .collect();

    assert_eq!(rim.len(), CHAINS);

    // Everything the plan needs of the flat reading, taken before the clock starts: those floods
    // are the tangent lines' business and not what this rung is about.
    for chain in &rim {
        regions.region(chain.edges[0].left_face());
        regions.region(chain.edges[0].right_face());
    }

    let floods_before = curve_regions.flood_count();
    let clock = Instant::now();

    for chain in &rim {
        let plan = SurfaceBevelPlan::try_create_with_curve_regions(
            chain,
            &regions,
            Some(&curve_regions),
            0.5,
            true,
        )
        .unwrap_or_else(|reason| panic!("the plan refused a rim edge: {reason:?}"));

        // The answer matters as much as the time: a reading that never ran out would be fast and
        // wrong, and every one of these edges stops in the middle of the rim at both ends.
        assert!(
            plan.start_runs_out && plan.end_runs_out,
            "a rim facet's chain runs out at both ends"
        );
    }

    let elapsed = clock.elapsed().as_millis();
    let floods = curve_regions.flood_count() - floods_before;

    assert!(
        floods <= 4,
        "a cylinder has a wall and two caps, and {CHAINS} plans flooded {floods} times between them"
    );

    assert!(
        elapsed < 2000,
        "{CHAINS} plans took {elapsed} ms, which is what a per-end walk of a {} face mesh costs",
        cylinder.faces.len()
    );
}
