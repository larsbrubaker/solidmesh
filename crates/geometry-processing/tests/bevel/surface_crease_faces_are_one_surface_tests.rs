//! A 90 degree crease whose two faces are
//! nevertheless ONE surface, and the sentence the refusal owes it. An L prism's inner corner that
//! fades into a cone blend half way up has its two walls joined round the far side of that blend
//! in steps of 3.75 and 7.5 degrees, so a flood at the picker's 10 degree smooth-wall tolerance
//! walks wall A up the cone, round the fade and back down wall B without ever crossing a feature
//! edge - and the crease's own two faces come back as the same region.
//!
//! The refusal itself is honest (a bead's square cap at the apex would have to be trimmed by the
//! cone's own sectors); what it must say is that the two faces are one surface round the far side
//! of the crease, with the tolerance riding along because narrowing it IS the way to break the
//! flood.

use std::sync::Arc;

use solidmesh::bevel::bevel_chain_linker;
use solidmesh::bevel::surface::SurfaceBevelPlan;
use solidmesh::bevel::{BevelAdvisoryArg, BevelAdvisoryCode};
use solidmesh::topology::{EdgeConvexity, FaceRegionCache, FeatureEdgeSet};
use solidmesh_math::Vector3;

use crate::topology_test_meshes::create_l_prism_with_fading_inner_corner;

/// The picker's smooth-wall rung, above the blend's 7.5 degree facet steps.
const FLAT_ANGLE_DEGREES: f64 = 10.0;

const TOP_RADIUS: f64 = 1.0;

const BLEND_FACETS: i32 = 12;

const HEIGHT: f64 = 10.0;

/// The middle of the sharp half of the inner corner, which runs from z 0 to z 5.
fn crease_midpoint() -> Vector3 {
    Vector3::new(10.0, 10.0, HEIGHT / 4.0)
}

#[test]
fn a_crease_whose_two_faces_flood_into_one_region_is_refused_by_saying_so() {
    let prism = create_l_prism_with_fading_inner_corner(TOP_RADIUS, BLEND_FACETS, HEIGHT);
    let graph = prism.get_mesh_edge_graph();
    let features = FeatureEdgeSet::create_with_angle(&prism, FLAT_ANGLE_DEGREES);
    let regions = Arc::new(FaceRegionCache::new(
        Arc::new(prism.clone()),
        Some(graph),
        FLAT_ANGLE_DEGREES,
    ));

    let concave: Vec<usize> = (0..features.count())
        .filter(|&i| features.edges()[i].convexity == EdgeConvexity::Concave)
        .collect();

    assert_eq!(
        concave.len(),
        1,
        "the fade leaves exactly one crease, the sharp half of the inner corner"
    );

    let mut chains = bevel_chain_linker::link(&[features.edges()[concave[0]]]);
    assert_eq!(chains.len(), 1);
    let chain = chains.remove(0);
    let edge = chain.edges[0];

    assert_eq!(edge.edge.convexity, EdgeConvexity::Concave);

    let left = regions
        .region(edge.left_face())
        .expect("the left face has a region");
    let right = regions
        .region(edge.right_face())
        .expect("the right face has a region");
    assert_eq!(
        SurfaceBevelPlan::key_of(&left),
        SurfaceBevelPlan::key_of(&right),
        "the flood walks one wall up the cone blend and back down the other, so the crease's two \
         faces are one region - which is the thing the refusal has to say"
    );

    let reason =
        match SurfaceBevelPlan::try_create_with_curve_regions(&chain, &regions, None, 2.0, true) {
            Ok(_) => panic!(
            "a bead's square cap at the apex would have to be trimmed by the cone's own sectors"
        ),
            Err(reason) => reason,
        };

    assert_eq!(
        reason.code(),
        BevelAdvisoryCode::SurfaceCreaseFacesAreOneSurface,
        "and it says WHY, rather than claiming nothing round here turns - {reason:?}"
    );

    assert_eq!(
        reason.args().len(),
        2,
        "where, and the tolerance that welded the two faces together - {reason:?}"
    );

    match reason.args()[0] {
        BevelAdvisoryArg::Vector3(at) => assert!(
            (at - crease_midpoint()).length() < 1e-9,
            "named at the middle of the crease that could not be planned - {reason:?}"
        ),
        ref other => panic!("the first argument is where, not {other:?} - {reason:?}"),
    }

    match reason.args()[1] {
        BevelAdvisoryArg::Double(tolerance) => assert!(
            (tolerance - FLAT_ANGLE_DEGREES).abs() <= 1e-9,
            "narrowing the flood is still the knob that breaks it - {reason:?}"
        ),
        ref other => panic!("the second argument is the tolerance, not {other:?} - {reason:?}"),
    }
}
