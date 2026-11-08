//! The lobed prism's junction picked the way a user
//! picks it - the WHOLE top cap as one face and the seam beside it. Three edges meet at the seam's
//! top vertex and one of them is concave, which is the junction
//! `surface_mixed_junction_over_a_curved_wall_tests` builds - but the cap's rim is one CLOSED chain
//! that runs THROUGH that vertex, so until it is opened there the reading sees one chain end where
//! it wants three and refuses the corner by name.
//!
//! The two halves are the same pick read twice: without `surface_chain_opens_at_the_junction` the
//! corner is refused, with it the junction reads and builds - the same bead, axis and 67.4868
//! degree turn the three-runs pick gets.

use std::rc::Rc;
use std::sync::Arc;

use geometry_processing::bevel::bevel_chain_linker::{self, BevelSweptChain};
use geometry_processing::bevel::surface::{
    surface_bevel_builder, surface_chain_join, surface_mesh_closure, SurfaceBevelPlan,
    SurfaceMixedJunction, SurfaceVertexPool,
};
use geometry_processing::bevel::BevelAdvisoryCode;
use geometry_processing::topology::{FaceRegionCache, FeatureEdge, FeatureEdgeSet};
use polygon_mesh::{CancelToken, Mesh};
use vector_math::{math_helper, Vector3};

use crate::lobed_prism_meshes;
use crate::mesh_measurements::shell_count;
use crate::surface_test_chains::join_and_open;

/// The fixture's own fillet radius, which is what "Every Chamfer Works.mcx" is filleted at.
const RADIUS: f64 = 3.756;

/// The turn the facet the bead rolled to names, which is the three-runs pick's own.
const TURN_DEGREES: f64 = 67.4868;

fn the_corner() -> Vector3 {
    Vector3::new(
        lobed_prism_meshes::left_crossing().x,
        0.0,
        lobed_prism_meshes::HEIGHT,
    )
}

/// Left closed, the cap's rim gives the junction no ends: three picked edges touch the vertex, one
/// of them is concave, and the reading refuses the corner by name.
#[test]
fn the_closed_rim_leaves_the_junction_with_no_ends_to_read() {
    let prism = lobed_prism_meshes::create();
    let (plans, _) = the_plans(&prism, false, false);

    // The rim really did arrive closed, and the seam really is the only chain with ends.
    assert_eq!(plans.iter().filter(|plan| plan.is_closed).count(), 1);

    let refused = SurfaceMixedJunction::try_read_all(
        &plans,
        SurfaceVertexPool::near_enough_for(&prism.vertices),
    )
    .expect_err("the closed rim leaves the junction unread");

    assert_eq!(
        refused.code(),
        BevelAdvisoryCode::SurfaceMixedJunctionNotRead,
        "three picked edges touch the vertex and one chain runs through it - a corner it cannot \
         read rather than a crowded one"
    );
}

/// Opened at the junction, the same pick is three chain ends there and reads as ONE junction - the
/// same bead and the same turn the three-runs pick reads.
#[test]
fn the_opened_rim_reads_the_same_junction_the_three_runs_do() {
    let prism = lobed_prism_meshes::create();
    let (plans, vertex) = the_plans(&prism, true, false);

    // Nothing is closed any more, and the three edges at the corner are three chain ends.
    assert!(!plans.iter().any(|plan| plan.is_closed));
    assert_eq!(ends_at(&plans, vertex), 3);

    let junctions = SurfaceMixedJunction::try_read_all(
        &plans,
        SurfaceVertexPool::near_enough_for(&prism.vertices),
    )
    .unwrap_or_else(|refused| panic!("the junction would not read: {refused:?}"));

    assert_eq!(junctions.len(), 1);
    assert_eq!(junctions[0].vertex, vertex);

    let turn = math_helper::radians_to_degrees(junctions[0].turn_radians());
    assert!(
        (turn - TURN_DEGREES).abs() <= 0.01,
        "the bead rolls to the same facet whichever way the three edges were picked - instead \
         {turn}"
    );
}

/// And the whole pick BUILDS: closed, manifold, one shell.
#[test]
fn the_rim_and_its_seam_build() {
    let prism = lobed_prism_meshes::create();
    let graph = prism.get_mesh_edge_graph();
    let (plans, _) = the_plans(&prism, true, false);
    let regions = lobed_regions(&prism);

    let built = surface_bevel_builder::try_build(
        &prism,
        &graph,
        &regions,
        &plans,
        8,
        &CancelToken::new(),
        crate::surface_test_build::build_options(),
    )
    .unwrap_or_else(|reason| panic!("the build refused the opened rim: {reason:?}"))
    .mesh;

    let (closed, open_at) = surface_mesh_closure::is_closed(&built);
    assert!(
        closed,
        "nothing may be left open, and something is at {open_at:?}"
    );
    assert!(built.is_manifold());
    assert_eq!(shell_count(&built), 1);
}

/// BOTH seams picked with the whole cap: the rim is cut at BOTH crossings, into two runs each with
/// a junction at each of its ends, and both junctions READ - one bead, two arms and a turn apiece.
///
/// The BUILD of that pick is a rung further on and is recorded here as what it says today:
/// `SurfaceCornerNotSupported` at the left crossing. The day it builds, this assertion is the thing
/// to turn round, on purpose.
#[test]
fn two_junctions_on_one_rim_are_both_read() {
    let prism = lobed_prism_meshes::create();
    let graph = prism.get_mesh_edge_graph();
    let (plans, vertex) = the_plans(&prism, true, true);
    let regions = lobed_regions(&prism);

    assert!(!plans.iter().any(|plan| plan.is_closed));
    assert_eq!(
        ends_at(&plans, vertex),
        3,
        "the left junction still has its three ends with the right one taken as well"
    );

    let junctions = SurfaceMixedJunction::try_read_all(
        &plans,
        SurfaceVertexPool::near_enough_for(&prism.vertices),
    )
    .unwrap_or_else(|refused| panic!("a junction would not read: {refused:?}"));

    assert_eq!(
        junctions.len(),
        2,
        "one at each crossing of the lobe, which are the fixture's own two seam corners"
    );

    let reason = surface_bevel_builder::try_build(
        &prism,
        &graph,
        &regions,
        &plans,
        8,
        &CancelToken::new(),
        crate::surface_test_build::build_options(),
    )
    .map(|_| ())
    .expect_err("this is the rung after the reading - if it builds now, assert the solid instead");

    let code = reason.advisory().map(|advisory| advisory.code());
    assert_eq!(
        code,
        Some(BevelAdvisoryCode::SurfaceCornerNotSupported),
        "and it says so at the left crossing, by name: {reason:?}"
    );
}

fn lobed_regions(prism: &Mesh) -> Arc<FaceRegionCache> {
    Arc::new(FaceRegionCache::new(
        Arc::new(prism.clone()),
        Some(prism.get_mesh_edge_graph()),
        lobed_prism_meshes::THRESHOLD_DEGREES,
    ))
}

/// The cap taken whole and the seam beside it (and the far seam when `both_seams`), each linked as
/// its own pick, joined, optionally opened at the junction, and planned - with the mesh's own index
/// for the corner read back off the plans.
fn the_plans(prism: &Mesh, open: bool, both_seams: bool) -> (Vec<Rc<SurfaceBevelPlan>>, i32) {
    let regions = lobed_regions(prism);
    let features = FeatureEdgeSet::create_with_angle(prism, lobed_prism_meshes::THRESHOLD_DEGREES);

    let mut chains: Vec<BevelSweptChain> = bevel_chain_linker::link(&the_whole_cap_rim(&features));

    chains.extend(bevel_chain_linker::link(&the_seam_at(
        &features,
        lobed_prism_meshes::left_crossing().x,
    )));

    if both_seams {
        chains.extend(bevel_chain_linker::link(&the_seam_at(
            &features,
            lobed_prism_meshes::right_crossing().x,
        )));
    }

    // This one case joins WITHOUT opening on purpose: it is the test OF the opening, and its
    // `open: false` leg is the shape the builder used to hand the plan before it opened at all.
    let joined = if open {
        join_and_open(&chains)
    } else {
        surface_chain_join::join(&chains)
    };

    let plans: Vec<Rc<SurfaceBevelPlan>> = joined
        .iter()
        .map(|chain| {
            SurfaceBevelPlan::try_create(chain, &regions, RADIUS, true)
                .unwrap_or_else(|reason| panic!("the plan refused a chain: {:?}", reason.code()))
        })
        .collect();

    let vertex = the_vertex(&plans);

    (plans, vertex)
}

/// The whole top cap's rim - every edge of the merged outline, which links into one ring.
fn the_whole_cap_rim(features: &FeatureEdgeSet) -> Vec<FeatureEdge> {
    features
        .edges()
        .iter()
        .filter(|edge| {
            (edge.start.z - lobed_prism_meshes::HEIGHT).abs() < 1e-4
                && (edge.end.z - lobed_prism_meshes::HEIGHT).abs() < 1e-4
        })
        .copied()
        .collect()
}

/// The concave seam where the flat wall runs into the lobe at the crossing `seam_x`; the
/// left and right seams differ only in the crossing.
fn the_seam_at(features: &FeatureEdgeSet, seam_x: f64) -> Vec<FeatureEdge> {
    features
        .edges()
        .iter()
        .filter(|edge| {
            (edge.start.x - seam_x).abs() < 1e-3
                && (edge.end.x - seam_x).abs() < 1e-3
                && edge.start.y.abs() < 1e-4
                && edge.end.y.abs() < 1e-4
        })
        .copied()
        .collect()
}

/// How many chain ends stand at a vertex, which is what the junction is gathered from.
fn ends_at(plans: &[Rc<SurfaceBevelPlan>], vertex: i32) -> i32 {
    let mut ends = 0;

    for plan in plans {
        if plan.is_closed {
            continue;
        }

        ends += i32::from(plan.edges[0].edge.from_vertex() == vertex);
        ends += i32::from(plan.edges[plan.edges.len() - 1].edge.to_vertex() == vertex);
    }

    ends
}

/// The welded mesh's own index for the corner, off the plans that meet there.
fn the_vertex(plans: &[Rc<SurfaceBevelPlan>]) -> i32 {
    for plan in plans {
        for edge in &plan.edges {
            if (edge.from() - the_corner()).length() < 1e-3 {
                return edge.edge.from_vertex();
            }

            if (edge.to() - the_corner()).length() < 1e-3 {
                return edge.edge.to_vertex();
            }
        }
    }

    panic!("no picked edge stands at {:?}", the_corner());
}
