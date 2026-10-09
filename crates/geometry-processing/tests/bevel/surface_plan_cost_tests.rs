//! Port of `SurfacePlanCostTests.cs`: what a plan COSTS, as a sanity rung rather than a benchmark.
//! Planning one chain reads the mesh twice at each free end - once for the edges standing at the
//! corner and once for the surfaces its faces belong to - and the obvious reading of each is a walk
//! of the whole mesh. Both readings are COUNTED, so the rung says the same thing on any machine and
//! under any load: the surfaces as the FLOOD COUNT (`FaceRegionCache::flood_count`), and the edges
//! as the GRAPH SCAN COUNT (`FaceRegionCache::graph_scan_count`), one scan for the cache as C#'s
//! one reading per cache makes. C#'s clock stays as the loose net for the costs neither count can
//! see, with C#'s bound (two seconds for work that takes a fraction of one), read on the test
//! thread's own CPU clock rather than the wall (`thread_cpu_time`, divergence 3130): planning runs
//! on the calling thread, and a wall clock read beside the rest of the suite counts the time it
//! waited for a core.

use std::sync::Arc;

use geometry_processing::bevel::bevel_chain_linker::{self, BevelSweptChain};
use geometry_processing::bevel::surface::SurfaceBevelPlan;
use geometry_processing::topology::{FaceGroup, FaceRegionCache, FeatureEdgeSet};

use crate::thread_cpu_time::thread_cpu_time;
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
    let scans_before = curve_regions.graph_scan_count();
    let clock = thread_cpu_time();

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

    let elapsed = (thread_cpu_time() - clock).as_millis();
    let floods = curve_regions.flood_count() - floods_before;
    let scans = curve_regions.graph_scan_count() - scans_before;

    assert!(
        floods <= 4,
        "a cylinder has a wall and two caps, and {CHAINS} plans flooded {floods} times between them"
    );

    // Every one of these chains asks at its ends which edges stand at the corner, and every plan
    // reads the cache's one index of them (C#'s one reading per cache, divergence 656): a scan of
    // every edge in the graph per end, or per plan, is the walk this rung is about.
    assert_eq!(
        scans,
        1,
        "{CHAINS} plans scanned every edge of a {} face mesh {scans} times",
        cylinder.faces.len()
    );

    assert!(
        elapsed < 2000,
        "{CHAINS} plans took {elapsed} ms, which is what a per-end walk of a {} face mesh costs",
        cylinder.faces.len()
    );
}
