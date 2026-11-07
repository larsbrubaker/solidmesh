//! The chain end that finishes at the REFLEX
//! corner of an L shaped face, where the face the bevel is walking back carries straight on past
//! the corner instead of stopping there.
//!
//! The fixture is `create_l_shaped_grid_prism`: a 20 square less the 10 square bite at
//! `x, y > 10`, 10 tall, whose top cap is an L with its reflex corner at `(10, 10, 10)`. The picked
//! edge is the cap's convex rim edge at `x = 10` running from `y = 20` to that reflex corner. It is
//! not a notched end: the groove's own section at `y = 10` lies at `x` in `[7, 10]`, in the MIDDLE
//! of that cap rather than on the face beyond, so the material past the corner closes the groove
//! itself, exactly as a run-out's does, and the band the walkback takes off stops on that plane
//! rather than running to the cap's far rim at `x = 20`. "Outer Top Bevel.mcx" found it.

use std::f64::consts::PI;
use std::sync::Arc;

use geometry_processing::bevel::bevel_chain_linker;
use geometry_processing::bevel::surface::{
    surface_bevel_builder, surface_mesh_closure, SurfaceBevelPlan,
};
use geometry_processing::topology::FeatureEdgeSet;
use polygon_mesh::CancelToken;
use vector_math::Vector3;

use crate::mesh_measurements::{shell_count, signed_volume};
use crate::surface_strip_tests::{chord_excess_area, wedge_area};
use crate::surface_walkback_checks::regions_of;
use crate::topology_test_meshes::create_l_shaped_grid_prism;

const RADIUS: f64 = 3.0;

const SEGMENTS: i32 = 8;

/// The L prism's own volume: a 20 square less its 10 square bite, 10 tall.
const PRISM_VOLUME: f64 = 3000.0;

/// How near the closed forms below the measured mesh has to come, in mm^3.
const MEASURED: f64 = 1e-4;

#[test]
fn a_fillet_ending_where_its_own_cap_carries_on_closes_with_its_own_section() {
    let prism = create_l_shaped_grid_prism();
    let (regions, features) = regions_of(&Arc::new(prism.clone()));
    let regions = Arc::new(regions);

    let chains = bevel_chain_linker::link(&[features.edges()[the_wall_top_edge(&features)]]);
    assert_eq!(chains.len(), 1);

    let plan = SurfaceBevelPlan::try_create(&chains[0], &regions, RADIUS, true)
        .unwrap_or_else(|reason| panic!("{:?}", reason.code()));

    let built = surface_bevel_builder::try_build_one(
        &prism,
        &prism.get_mesh_edge_graph(),
        &regions,
        plan,
        SEGMENTS,
        &CancelToken::new(),
    )
    .unwrap_or_else(|reason| {
        panic!(
            "the run ends where the cap carries on, which closes with its own section: {reason:?}"
        )
    });

    assert_eq!(
        surface_mesh_closure::open_edge_count(&built).0,
        0,
        "a groove closed by a wall of its own leaves a closed solid"
    );
    assert!(built.is_manifold());
    assert_eq!(shell_count(&built), 1);

    // The whole of the answer in one number: the groove is the fillet's section carried the 10 mm
    // the edge runs and not one millimetre further. A band that ran on past the reflex corner to
    // the cap's far rim would take material off the other side of the L as well.
    let section = wedge_area(RADIUS, PI / 2.0) + chord_excess_area(RADIUS, PI / 2.0, SEGMENTS);

    let expected = PRISM_VOLUME - (10.0 * section);
    let measured = signed_volume(&built);
    assert!(
        (measured - expected).abs() <= MEASURED,
        "the fillet removes its own section over the edge's own 10 mm, and nothing beyond the \
         corner - {measured}, expected {expected}"
    );

    // And said where it matters rather than only in total: nothing was cut out of the cap on the
    // far side of the reflex corner, which is every vertex of the built mesh with y below 10.
    let into_the_far_side: Vec<Vector3> = built
        .vertices
        .iter()
        .map(|&vertex| Vector3::from_vector3_float(vertex))
        .filter(|at| at.y < 10.0 - 1e-9 && at.z > 10.0 - RADIUS && at.z < 10.0 - 1e-9)
        .collect();

    assert_eq!(
        into_the_far_side.len(),
        0,
        "the cap south of the reflex corner is untouched: {into_the_far_side:?}"
    );
}

/// The picked edge: the top of the wall the bite left standing, at `x = 10` from `y = 20` down to
/// the reflex corner at `y = 10`.
fn the_wall_top_edge(features: &FeatureEdgeSet) -> usize {
    let found: Vec<usize> = (0..features.count())
        .filter(|&i| {
            let edge = features.edges()[i];

            (edge.start.x - 10.0).abs() < 1e-9
                && (edge.end.x - 10.0).abs() < 1e-9
                && (edge.start.z - 10.0).abs() < 1e-9
                && (edge.end.z - 10.0).abs() < 1e-9
                && edge.start.y.min(edge.end.y) > 10.0 - 1e-9
        })
        .collect();

    assert_eq!(found.len(), 1, "one edge tops the wall");
    found[0]
}
