//! Rust-only tests of `section_routing` beyond the ported routing tests: the room reading over a
//! routed pass, the plan-less overload falling through to the planned one, and a pass with nothing
//! to route. None reads the process-wide last route, which parallel tests share.

use std::rc::Rc;
use std::sync::Arc;

use geometry_processing::bevel::bevel_chain_linker;
use geometry_processing::bevel::sections::section_routing::{
    has_room, is_simple, is_simple_with_planless, partition, SectionRouteAnswer,
};
use geometry_processing::bevel::surface::SurfaceBevelPlan;
use geometry_processing::topology::{FaceRegionCache, FeatureEdge, FeatureEdgeSet};
use polygon_mesh::Mesh;

use crate::bore_test_meshes::create_plate_with_a_round_bore;

const BORE_RADIUS: f64 = 3.0;
const THICKNESS: f64 = 10.0;

fn the_mouth_plan(plate: &Mesh, size: f64) -> Rc<SurfaceBevelPlan> {
    let features = FeatureEdgeSet::create(plate);
    let regions = Arc::new(FaceRegionCache::new(
        Arc::new(plate.clone()),
        Some(plate.get_mesh_edge_graph()),
        features.flat_angle_degrees(),
    ));
    let on_the_bore = |x: f64, y: f64| ((x * x + y * y).sqrt() - BORE_RADIUS).abs() < 1e-3;
    let mouth: Vec<FeatureEdge> = features
        .edges()
        .iter()
        .filter(|edge| {
            (edge.start.z - THICKNESS / 2.0).abs() < 1e-3
                && (edge.end.z - THICKNESS / 2.0).abs() < 1e-3
                && on_the_bore(edge.start.x, edge.start.y)
                && on_the_bore(edge.end.x, edge.end.y)
        })
        .copied()
        .collect();
    let chain = bevel_chain_linker::link(&mouth).remove(0);
    SurfaceBevelPlan::try_create(&chain, &regions, size, true).unwrap()
}

/// A routed straight-bore mouth has room for a small size, and none for one wider than the
/// plate's land; the plan-less overload with no loops answers exactly as the planned one.
#[test]
fn rust_only_a_routed_mouth_reads_its_room_and_the_planless_overload_falls_through() {
    let plate = create_plate_with_a_round_bore(24, BORE_RADIUS, THICKNESS, 6.0);
    let graph = plate.get_mesh_edge_graph();
    let plans = vec![the_mouth_plan(&plate, 0.4)];

    let SectionRouteAnswer::Sections(route) =
        is_simple_with_planless(&plans, &[], Some(&plate), Some(&graph), 8, None).unwrap()
    else {
        panic!("the straight bore's mouth routes");
    };
    assert_eq!(route.runs.len(), 1);
    assert!(route.runs[0].is_closed);
    assert!(route.pool.count() > route.pool.seeded_count());

    assert!(has_room(&plate, &graph, &route.runs, 0.4).is_ok());

    let too_big = has_room(&plate, &graph, &route.runs, 20.0);
    let failure = too_big
        .expect_err("a 20 radius does not fit a 3 radius bore in a 10 thick plate")
        .expect("the room names its advisory");
    assert!(
        failure.code().to_string().starts_with("Size"),
        "{failure:?}"
    );
}

/// No mesh, or no plans, is "there is nothing to route", and a partition of nothing is empty.
#[test]
fn rust_only_a_pass_with_nothing_to_route_says_so() {
    let plate = create_plate_with_a_round_bore(24, BORE_RADIUS, THICKNESS, 6.0);
    let graph = plate.get_mesh_edge_graph();

    let empty = is_simple(&[], Some(&plate), Some(&graph), 8, None).unwrap();
    assert_eq!(empty.why_not(), Some("there is nothing to route"));

    let plans = vec![the_mouth_plan(&plate, 0.4)];
    let no_mesh = is_simple(&plans, None, Some(&graph), 8, None).unwrap();
    assert_eq!(no_mesh.why_not(), Some("there is nothing to route"));

    let answer = partition(&plans, None, Some(&graph), None);
    assert!(answer.accepted.is_empty() && answer.declined.is_empty());
}
