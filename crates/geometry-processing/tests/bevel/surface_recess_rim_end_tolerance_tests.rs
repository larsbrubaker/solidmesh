//! The size boundary at the pyramidal recess's rim
//! end - below it the surface construction closes, above it the end cut refuses the profile by
//! name, and NOWHERE may it publish a mesh that is open.
//!
//! The crease beaded here is the upper-left one of "UpperLeftInner should bevel.mcx", from the rim
//! end `(0.0158, 0.0316, 20)` to the apex `(5, 10, 10.0316)`. The left wall's foot at the rim end is
//! only 0.0008 below z = 20, so as the size grows the end profile straddles the top face as well as
//! the left wall, and from 0.0335 up `surface_end_cut` refuses by `SurfaceStripEndNotSupported`. At
//! 0.0325 the build once returned a mesh with two open edges; a result is either closed and
//! manifold or refused BY NAME. The parameter rows are a loop.

use std::sync::Arc;

use geometry_processing::bevel::bevel_chain_linker::{self, BevelSweptChain};
use geometry_processing::bevel::surface::{
    surface_bevel_builder, surface_mesh_closure, surface_room, SurfaceBevelPlan,
    SurfaceBuildFailure, SurfaceBuildOptions,
};
use geometry_processing::topology::{FaceRegionCache, FeatureEdgeSet};
use polygon_mesh::CancelToken;
use vector_math::Vector3;

use crate::pyramid_recess_test_meshes;

const SEGMENTS: i32 = 8;

/// The size the boundary was measured at: TRUE with two open edges before the fix.
const THE_SIZE_THAT_PUBLISHED_AN_OPEN_MESH: f64 = 0.0325;

/// The one case measured: at 0.0325 the construction either refuses by name or hands back a closed
/// manifold shell. It must not hand back an open one.
#[test]
fn the_size_at_the_boundary_does_not_publish_an_open_mesh() {
    assert_closed_or_named(THE_SIZE_THAT_PUBLISHED_AN_OPEN_MESH);
}

/// And the same invariant swept across the boundary, so it is pinned wherever the boundary lies
/// rather than at the one size it was found at.
#[test]
fn every_size_across_the_boundary_is_closed_or_named() {
    for size in [
        0.0300, 0.0305, 0.0310, 0.0315, 0.0320, 0.0325, 0.0330, 0.0335, 0.0340,
    ] {
        assert_closed_or_named(size);
    }
}

/// The invariant itself: plan, fit and build the rim-to-apex crease at `size`, and hold the answer
/// to closed-and-manifold or a named refusal - never a published open mesh.
fn assert_closed_or_named(size: f64) {
    let part =
        pyramid_recess_test_meshes::create_from_the_fixtures_own_triangles(Vector3::ZERO, false);
    let graph = part.get_mesh_edge_graph();
    let features = FeatureEdgeSet::create(&part);
    let regions = Arc::new(FaceRegionCache::new(
        Arc::new(part.clone()),
        Some(graph.clone()),
        features.flat_angle_degrees(),
    ));
    let chain = the_upper_left_crease(&features);

    // A plan refusal is a BevelAdvisory, which always names its reason.
    let Ok(plan) = SurfaceBevelPlan::try_create(&chain, &regions, size, true) else {
        return;
    };

    let plans = vec![plan];

    let fit =
        surface_room::try_fit(&plans, size, &CancelToken::new(), None).expect("not cancelled");

    if !fit.fits {
        assert!(fit.refusal.is_some(), "a refusal at {size} says why");
        return;
    }

    let built = match surface_bevel_builder::try_build(
        &part,
        &graph,
        &regions,
        &plans,
        SEGMENTS,
        &CancelToken::new(),
        SurfaceBuildOptions::default(),
    ) {
        Ok(built) => built.mesh,
        Err(reason) => {
            assert!(
                matches!(reason, SurfaceBuildFailure::Refused(_)),
                "a refusal at {size} says why - instead {reason:?}"
            );
            return;
        }
    };

    let (open_edges, open_at) = surface_mesh_closure::open_edge_count(&built);

    assert_eq!(
        open_edges, 0,
        "the build published a mesh at size {size}, so it is closed - open at {open_at:?}"
    );

    assert!(built.is_manifold(), "and manifold at size {size}");
}

/// The recess's upper-left crease, rim end to apex, as a chain - the one concave crease edge of
/// the fixture running from `(0.0158, 0.0316, 20)` to `(5, 10, 10.0316)`.
fn the_upper_left_crease(features: &FeatureEdgeSet) -> BevelSweptChain {
    let rim = pyramid_recess_test_meshes::rim_end();
    let apex = pyramid_recess_test_meshes::apex_end();

    let found: Vec<usize> = (0..features.count())
        .filter(|&i| {
            let edge = features.edges()[i];

            ((edge.start - rim).length() < 1e-5 && (edge.end - apex).length() < 1e-5)
                || ((edge.start - apex).length() < 1e-5 && (edge.end - rim).length() < 1e-5)
        })
        .collect();
    assert_eq!(
        found.len(),
        1,
        "one crease runs from the rim end to the apex"
    );

    let mut chains = bevel_chain_linker::link(&[features.edges()[found[0]]]);
    assert_eq!(chains.len(), 1);

    chains.remove(0)
}
