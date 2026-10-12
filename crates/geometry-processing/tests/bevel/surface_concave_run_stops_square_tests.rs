//! Port of `SurfaceConcaveRunStopsSquareTests.cs`: the concave mirror of
//! `SurfacePartialRimRunTests` - a bead on ONE facet of a boss's foot, whose crease carries straight
//! on past both of its ends in edges nobody picked. There is no face beyond either corner - the slab
//! top's neighbour is the slab top and the wall's neighbour is the next wall facet a few degrees
//! round - so the bead stops SQUARE at each end, with its own section standing in the plane square
//! to the edge, exactly as the groove of a convex rim run does.
//!
//! Measured on a chamfer, whose section is `d^2 / 2` exactly, and swept along one chord with two
//! square ends - so what the bead adds is the prism `section x chord` and nothing else. No mitre
//! term anywhere: a single facet has no corner of its own. C# `[Arguments]` rows are a loop over the
//! same-named test.

use std::sync::Arc;

use geometry_processing::bevel::bevel_chain_linker;
use geometry_processing::bevel::surface::{
    surface_bevel_builder, surface_mesh_closure, SurfaceBevelPlan,
};
use geometry_processing::topology::{
    EdgeConvexity, FaceGroup, FaceRegionCache, FeatureEdge, FeatureEdgeSet,
};
use polygon_mesh::CancelToken;

use crate::mesh_measurements::{shell_count, signed_volume};
use crate::topology_test_meshes::create_boss_on_slab;

const SIDES: i32 = 48;

const POST_RADIUS: f64 = 6.0;

const SLAB_TOP: f64 = 3.0;

/// Volumes are summed over float vertices, so they are read to a ten thousandth.
const MEASURED: f64 = 1e-4;

#[test]
fn one_facet_of_a_boss_foot_beads_and_stops_square_at_both_ends() {
    for setback in [0.5, 1.0] {
        let part = Arc::new(create_boss_on_slab(POST_RADIUS, SIDES, SLAB_TOP));
        let graph = part.get_mesh_edge_graph();
        let features = FeatureEdgeSet::create(&part);
        let regions = Arc::new(FaceRegionCache::new(
            part.clone(),
            Some(graph.clone()),
            features.flat_angle_degrees(),
        ));
        let curve_regions = Arc::new(FaceRegionCache::new(
            part.clone(),
            Some(graph.clone()),
            FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES,
        ));

        let mut chains = bevel_chain_linker::link(&[one_foot_facet(&features)]);
        assert_eq!(chains.len(), 1);
        let chain = chains.remove(0);

        assert!(
            !chain.is_closed,
            "one facet of the foot is a run with two free ends"
        );

        assert_eq!(
            chain.edges[0].edge.convexity,
            EdgeConvexity::Concave,
            "a post standing on a slab meets it in a bead's crease"
        );

        let plan = SurfaceBevelPlan::try_create_with_curve_regions(
            &chain,
            &regions,
            Some(&curve_regions),
            setback,
            false,
        )
        .unwrap_or_else(|reason| {
            panic!("the crease carries on past both corners, so both ends stop square - instead {reason}")
        });

        assert!(
            plan.start_runs_out,
            "the foot circle goes on past the start of what was picked, and the next wall facet \
             round is not a face standing ACROSS the corner - its plane lies all but along the rail"
        );
        assert!(plan.end_runs_out, "and past the end of it");

        let built = surface_bevel_builder::try_build_one(
            &part,
            &graph,
            &regions,
            plan,
            1,
            &CancelToken::new(),
        )
        .unwrap_or_else(|reason| panic!("{setback}: {reason:?}"));

        assert_eq!(
            surface_mesh_closure::open_edge_count(&built).0,
            0,
            "both square ends are closed by the bead's own section"
        );
        assert!(built.is_manifold());
        assert_eq!(shell_count(&built), 1);

        let chord = 2.0 * POST_RADIUS * (std::f64::consts::PI / SIDES as f64).sin();
        let section = setback * setback / 2.0;
        let added = signed_volume(&built) - signed_volume(&part);

        assert!(
            (added - (section * chord)).abs() <= MEASURED,
            "{setback}: a square ended bead on one straight chord is the swept section and nothing \
             else - instead {added} against {}",
            section * chord
        );
    }
}

/// One edge of the foot circle, which is where the post's wall meets the slab's top.
fn one_foot_facet(features: &FeatureEdgeSet) -> FeatureEdge {
    let foot: Vec<FeatureEdge> = features
        .edges()
        .iter()
        .filter(|edge| {
            edge.convexity == EdgeConvexity::Concave
                && (edge.start.z - SLAB_TOP).abs() < 1e-6
                && (edge.end.z - SLAB_TOP).abs() < 1e-6
        })
        .copied()
        .collect();

    assert_eq!(foot.len(), SIDES as usize, "the foot is {SIDES} edges");

    foot[0]
}
