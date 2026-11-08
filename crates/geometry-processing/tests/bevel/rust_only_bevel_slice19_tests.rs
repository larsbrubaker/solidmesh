//! Rust-only tests for slice 19's units that have no test class of their own: the walkback's
//! reconciling of the standing features two bands reach (`SurfaceWalkbackStandingFeatures`), and
//! the survey that refuses two plans turning a corner at one vertex
//! (`SurfaceTwoCornersAtOneVertex`).

use std::rc::Rc;
use std::sync::Arc;

use solidmesh::bevel::bevel_chain_linker;
use solidmesh::bevel::surface::surface_two_corners_at_one_vertex::try_survey;
use solidmesh::bevel::surface::surface_walkback_standing_features::{
    none_of_the_feature_is_also_being_cut, try_merge_what_two_bands_both_reached,
};
use solidmesh::bevel::surface::SurfaceBevelPlan;
use solidmesh::bevel::surface::{
    SurfaceStandingFeatureWithheld, SurfaceStandingFeatureWithheldFace, SurfaceVertexPool,
};
use solidmesh::bevel::BevelAdvisoryCode;
use solidmesh::topology::{FaceRegionCache, FeatureEdge, FeatureEdgeSet};
use solidmesh_math::{Vector3, Vector3Float};
use solidmesh_mesh::solids::platonic_solids;

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
            vec![solidmesh::bevel::BevelAdvisoryArg::Vector3(corner)]
        )
    );
}
