//! Rust-only tests for slice 19's units that have no C# test class of their own in
//! `GeometryProcessingTests`: the mixed-corner blends a selection asks for (`BevelMixedBlends`,
//! whose C# tests are app-level), and the walkback's reconciling of the standing features two
//! bands reach (`SurfaceWalkbackStandingFeatures`), and the survey that refuses two plans turning
//! a corner at one vertex (`SurfaceTwoCornersAtOneVertex`).

use std::rc::Rc;
use std::sync::Arc;

use geometry_processing::bevel::surface::surface_two_corners_at_one_vertex::try_survey;
use geometry_processing::bevel::surface::surface_walkback_standing_features::{
    none_of_the_feature_is_also_being_cut, try_merge_what_two_bands_both_reached,
};
use geometry_processing::bevel::surface::SurfaceBevelPlan;
use geometry_processing::bevel::surface::{
    SurfaceStandingFeatureWithheld, SurfaceStandingFeatureWithheldFace, SurfaceVertexPool,
};
use geometry_processing::bevel::BevelAdvisoryCode;
use geometry_processing::bevel::{bevel_chain_linker, bevel_mixed_blends};
use geometry_processing::topology::{
    BevelEdgeGroup, BevelEdgeGroupResolution, EdgeConvexity, FaceRegionCache, FeatureEdge,
    FeatureEdgeSet,
};
use polygon_mesh::solids::platonic_solids;
use vector_math::{Vector3, Vector3Float};

use crate::topology_test_meshes::create_l_shaped_prism;

/// A resolved group selecting `edges` among `features`.
fn resolution(features: &Arc<FeatureEdgeSet>, edges: Vec<FeatureEdge>) -> BevelEdgeGroupResolution {
    BevelEdgeGroupResolution {
        group: BevelEdgeGroup::default(),
        features: features.clone(),
        failure_reason: None,
        seed_edge_index: -1,
        edges,
        chain: None,
        face_group: None,
    }
}

/// The L prism's features, its one concave edge, and the two convex top edges meeting it at
/// (10, 10, 10).
fn l_prism_corner() -> (Arc<FeatureEdgeSet>, FeatureEdge, Vec<FeatureEdge>) {
    let mesh = create_l_shaped_prism();
    let features = Arc::new(FeatureEdgeSet::create(&mesh));
    let top = Vector3::new(10.0, 10.0, 10.0);
    let at_top =
        |edge: &FeatureEdge| (edge.start - top).length() < 1e-6 || (edge.end - top).length() < 1e-6;

    let concave = *features
        .edges()
        .iter()
        .find(|e| e.convexity == EdgeConvexity::Concave)
        .expect("the L prism has its reflex edge");
    let convex: Vec<FeatureEdge> = features
        .edges()
        .iter()
        .filter(|e| e.convexity == EdgeConvexity::Convex && at_top(e))
        .cloned()
        .collect();

    assert!(at_top(&concave));
    assert_eq!(convex.len(), 2);

    (features, concave, convex)
}

#[test]
fn rust_only_a_cut_into_a_bead_gets_one_blend_capped_on_the_top() {
    let (features, concave, convex) = l_prism_corner();
    let resolutions = vec![
        resolution(&features, vec![convex[0]]),
        resolution(&features, vec![concave]),
    ];

    let set = bevel_mixed_blends::create(&resolutions, &[true, true], true, 2.0, 8);

    assert_eq!(set.blends().len(), 1);
    assert!(set.warnings().iter().all(Option::is_none));
    assert_eq!(set.blends()[0].groups(), &[0, 1]);
    assert_eq!(set.convex_handovers().len(), 1);
    assert_eq!(set.concave_cap_normals().len(), 1);

    let cap = *set.concave_cap_normals().values().next().unwrap();
    assert!((cap - Vector3::new(0.0, 0.0, 1.0)).length() < 1e-6, "{cap}");
    assert!(set.convex_handovers().values().next().unwrap().is_capped());
}

#[test]
fn rust_only_a_chamfered_pair_is_left_alone_and_says_nothing() {
    let (features, concave, convex) = l_prism_corner();
    let resolutions = vec![
        resolution(&features, vec![convex[0]]),
        resolution(&features, vec![concave]),
    ];

    let set = bevel_mixed_blends::create(&resolutions, &[true, true], false, 2.0, 8);

    assert!(set.blends().is_empty());
    assert!(set.convex_handovers().is_empty());
    assert!(set.warnings().iter().all(Option::is_none));
}

#[test]
fn rust_only_two_cuts_in_one_group_are_named_and_not_blended() {
    let (features, concave, convex) = l_prism_corner();
    let resolutions = vec![
        resolution(&features, convex),
        resolution(&features, vec![concave]),
    ];

    let set = bevel_mixed_blends::create(&resolutions, &[true, true], true, 2.0, 8);

    assert!(set.blends().is_empty());

    for warnings in set.warnings() {
        let warnings = warnings.as_ref().expect("both groups are told");
        assert_eq!(warnings.len(), 1);
        assert_eq!(
            warnings[0].code(),
            BevelAdvisoryCode::BlendSameGroupNotBlended
        );
    }
}

#[test]
fn rust_only_a_three_edge_chamfer_corner_says_it_was_capped() {
    let (features, concave, convex) = l_prism_corner();
    let resolutions = vec![
        resolution(&features, vec![convex[0]]),
        resolution(&features, vec![convex[1]]),
        resolution(&features, vec![concave]),
    ];

    let set = bevel_mixed_blends::create(&resolutions, &[true, true, true], false, 2.0, 8);

    assert!(set.blends().is_empty());
    assert!(set.warnings().iter().all(Option::is_some));
}

fn pool() -> SurfaceVertexPool {
    SurfaceVertexPool::new(&[
        Vector3Float::new_f32(0.0, 0.0, 0.0),
        Vector3Float::new_f32(1.0, 0.0, 0.0),
        Vector3Float::new_f32(2.0, 0.0, 0.0),
    ])
}

fn face(facets: Vec<i32>, crease: (i32, i32)) -> Rc<SurfaceStandingFeatureWithheldFace> {
    Rc::new(SurfaceStandingFeatureWithheldFace::new(
        facets,
        crease,
        vec![crease.0],
        Vector3::ZERO,
        Vector3::new(0.0, 0.0, 1.0),
    ))
}

#[test]
fn rust_only_two_readings_of_one_feature_become_one_record() {
    let first = Rc::new(SurfaceStandingFeatureWithheld::new(
        (0, true),
        Vector3::ZERO,
        vec![face(vec![3, 4], (0, 1))],
    ));
    let second = Rc::new(SurfaceStandingFeatureWithheld::new(
        (2, false),
        Vector3::ZERO,
        vec![face(vec![4, 3], (1, 2))],
    ));
    let mut withheld = vec![first.clone(), second];

    try_merge_what_two_bands_both_reached(&mut withheld, &pool()).expect("one feature");

    assert_eq!(withheld.len(), 1);
    assert!(Rc::ptr_eq(&withheld[0], &first));
    assert_eq!(*first.reaches.borrow(), vec![(0, true), (2, false)]);
    assert_eq!(*first.faces[0].creases.borrow(), vec![(0, 1), (1, 2)]);
    assert_eq!(*first.faces[0].withins.borrow(), vec![vec![0], vec![1]]);
}

#[test]
fn rust_only_two_features_sharing_a_facet_are_refused() {
    let mut withheld = vec![
        Rc::new(SurfaceStandingFeatureWithheld::new(
            (0, true),
            Vector3::ZERO,
            vec![face(vec![3, 4], (0, 1))],
        )),
        Rc::new(SurfaceStandingFeatureWithheld::new(
            (2, false),
            Vector3::ZERO,
            vec![face(vec![4, 5], (1, 2))],
        )),
    ];

    let failure = try_merge_what_two_bands_both_reached(&mut withheld, &pool()).unwrap_err();

    assert_eq!(
        failure.code(),
        BevelAdvisoryCode::SurfaceStandingFeatureUnderTwoCuts
    );
    assert_eq!(
        withheld.len(),
        2,
        "a refused merge leaves the list as it was"
    );
}

#[test]
fn rust_only_a_withheld_facet_inside_a_cut_region_is_named() {
    let withheld = vec![Rc::new(SurfaceStandingFeatureWithheld::new(
        (0, true),
        Vector3::ZERO,
        vec![face(vec![1, 2], (0, 1))],
    ))];

    assert!(none_of_the_feature_is_also_being_cut(&withheld, &[-1, -1, -1], &pool()).is_ok());

    let failure =
        none_of_the_feature_is_also_being_cut(&withheld, &[-1, -1, 0], &pool()).unwrap_err();

    assert_eq!(
        failure.code(),
        BevelAdvisoryCode::SurfaceStandingFeatureDoesNotExtend
    );
}

/// A plan along the cube's top front and top right edges, which turns one corner at (10, -10, 10).
fn cube_corner_plan() -> Rc<SurfaceBevelPlan> {
    let mesh = Arc::new(platonic_solids::create_cube(20.0, 20.0, 20.0));
    let features = FeatureEdgeSet::create(&mesh);
    let on_top_through = |edge: &FeatureEdge, x: f64, y: f64| {
        edge.start.z > 9.9
            && edge.end.z > 9.9
            && ((edge.start.x - x).abs() < 1e-9 && (edge.end.x - x).abs() < 1e-9
                || (edge.start.y - y).abs() < 1e-9 && (edge.end.y - y).abs() < 1e-9)
    };
    let picked: Vec<FeatureEdge> = features
        .edges()
        .iter()
        .filter(|e| on_top_through(e, 10.0, -10.0))
        .cloned()
        .collect();
    assert_eq!(picked.len(), 2);

    let regions = Arc::new(FaceRegionCache::new(
        mesh.clone(),
        None,
        features.flat_angle_degrees(),
    ));
    let chain = bevel_chain_linker::link(&picked).remove(0);

    SurfaceBevelPlan::try_create(&chain, &regions, 1.0, true)
        .unwrap_or_else(|reason| panic!("the plan refused the corner: {reason:?}"))
}

#[test]
fn rust_only_one_plan_turning_a_corner_can_be_built() {
    let plan = cube_corner_plan();

    assert_eq!(plan.edges.len(), 2);
    assert!(try_survey(&[plan]).is_ok());
}

#[test]
fn rust_only_two_plans_turning_a_corner_at_one_vertex_are_refused_there() {
    let plan = cube_corner_plan();
    let corner = plan.edges[0].to();

    let failure = try_survey(&[plan.clone(), plan]).unwrap_err();

    assert_eq!(
        failure.code(),
        BevelAdvisoryCode::SurfaceTwoCornersAtOneVertex
    );
    assert_eq!(
        format!("{:?}", failure.args()),
        format!(
            "{:?}",
            vec![geometry_processing::bevel::BevelAdvisoryArg::Vector3(
                corner
            )]
        )
    );
}
