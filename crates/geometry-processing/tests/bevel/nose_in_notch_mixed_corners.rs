//! The mixed corner at either end of the nose-in-notch part's ridge, read off the two edge plans
//! that meet there - the ridge's cut and the upper crease's bead - as the rim pick itself hands
//! them over.

#![allow(dead_code)]

use std::rc::Rc;
use std::sync::Arc;

use geometry_processing::bevel::bevel_chain_linker;
use geometry_processing::bevel::surface::{
    SurfaceBevelPlan, SurfaceEdgePlan, SurfaceMixedCorner, SurfaceVertexPool,
};
use geometry_processing::topology::{FaceRegionCache, FeatureEdgeSet};
use vector_math::Vector3;

use crate::nose_in_notch_test_meshes;

/// The corner the nose's ridge and its upper crease make at `ridge_end`.
pub fn at_the_ridge_end(ridge_end: Vector3, radius: f64) -> Rc<SurfaceMixedCorner> {
    let mesh = Arc::new(nose_in_notch_test_meshes::create());
    let features = FeatureEdgeSet::create(&mesh);
    let regions = Arc::new(FaceRegionCache::new(
        mesh.clone(),
        Some(mesh.get_mesh_edge_graph()),
        features.flat_angle_degrees(),
    ));
    let near_enough = SurfaceVertexPool::near_enough_for(&mesh.vertices);

    let cut = plan_of(
        &regions,
        &features,
        nose_in_notch_test_meshes::left_ridge_end(),
        nose_in_notch_test_meshes::right_ridge_end(),
        radius,
    );
    let bead = plan_of(
        &regions,
        &features,
        ridge_end,
        nose_in_notch_test_meshes::upper_crease_end(),
        radius,
    );

    let corner = SurfaceMixedCorner::try_read(&cut, &bead, ridge_end, radius, near_enough)
        .unwrap_or_else(|failure| {
            panic!("the oblique corner at {ridge_end:?} would not read: {failure:?}")
        });
    Rc::new(corner)
}

/// The plan of the one feature edge between `from` and `to`, as a chain of that edge
/// alone.
fn plan_of(
    regions: &Arc<FaceRegionCache>,
    features: &FeatureEdgeSet,
    from: Vector3,
    to: Vector3,
    radius: f64,
) -> SurfaceEdgePlan {
    let found: Vec<usize> = (0..features.count())
        .filter(|&i| {
            let edge = &features.edges()[i];
            ((edge.start - from).length() < 1e-6 && (edge.end - to).length() < 1e-6)
                || ((edge.end - from).length() < 1e-6 && (edge.start - to).length() < 1e-6)
        })
        .collect();
    assert_eq!(found.len(), 1, "one feature edge from {from:?} to {to:?}");

    let mut chains = bevel_chain_linker::link(&[features.edges()[found[0]]]);
    assert_eq!(chains.len(), 1);
    let plan = SurfaceBevelPlan::try_create(&chains.remove(0), regions, radius, true)
        .unwrap_or_else(|reason| {
            panic!("the plan refused the edge from {from:?} to {to:?}: {reason:?}")
        });
    plan.edges[0].clone()
}
