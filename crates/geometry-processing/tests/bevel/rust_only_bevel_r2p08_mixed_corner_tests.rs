//! Rust-only checks of R2.P08's mixed corner on a REAL corner - the L prism's convex rim meeting
//! its concave inside edge, each planned off the mesh by `SurfaceBevelPlan::try_create`, and the
//! two planned as one chain so the plan reads the corner itself: `SurfaceMixedCorner::try_read`'s
//! success path, either way round, and on a LEANING cap (C# `SurfaceMixedCornerReadingTests.
//! ACapThatLeansIsReadAsAnEllipseAtEveryScale`'s construction); and the patch's ring turned round
//! the corner and emitted.

use std::sync::Arc;

use geometry_processing::bevel::bevel_chain_linker;
use geometry_processing::bevel::surface::{
    SurfaceBevelPlan, SurfaceEdgePlan, SurfaceMixedCorner, SurfaceVertexPool,
};
use geometry_processing::topology::{EdgeConvexity, FaceRegionCache, FeatureEdgeSet};
use polygon_mesh::Mesh;
use vector_math::{Vector3, Vector3Float};

use crate::topology_test_meshes::create_l_shaped_grid_prism;

const RADIUS: f64 = 1.0;

fn top_corner() -> Vector3 {
    Vector3::new(10.0, 10.0, 10.0)
}

fn plan_of(
    mesh: &Arc<Mesh>,
    features: &FeatureEdgeSet,
    edge_index: usize,
    radius: f64,
) -> SurfaceEdgePlan {
    let regions = Arc::new(FaceRegionCache::new(
        mesh.clone(),
        None,
        features.flat_angle_degrees(),
    ));
    let chain = bevel_chain_linker::link(&[features.edges()[edge_index]]).remove(0);
    let plan = SurfaceBevelPlan::try_create(&chain, &regions, radius, true)
        .unwrap_or_else(|reason| panic!("the plan refused the edge: {reason:?}"));
    plan.edges[0].clone()
}

fn the_concave_edge(features: &FeatureEdgeSet) -> usize {
    let found: Vec<usize> = (0..features.count())
        .filter(|&i| features.edges()[i].convexity == EdgeConvexity::Concave)
        .collect();
    assert_eq!(found.len(), 1);
    found[0]
}

fn the_convex_edge_of(
    features: &FeatureEdgeSet,
    corner: Vector3,
    far: Vector3,
    meets: f64,
) -> usize {
    let found: Vec<usize> = (0..features.count())
        .filter(|&i| {
            let edge = &features.edges()[i];
            ((edge.start - corner).length() < meets && (edge.end - far).length() < meets)
                || ((edge.end - corner).length() < meets && (edge.start - far).length() < meets)
        })
        .collect();
    assert_eq!(found.len(), 1);
    found[0]
}

#[test]
fn rust_only_a_real_mixed_corner_reads_the_same_either_way_round() {
    let mesh = Arc::new(create_l_shaped_grid_prism());
    let features = FeatureEdgeSet::create(&mesh);
    let near_enough = SurfaceVertexPool::near_enough_for(&mesh.vertices);
    let convex = plan_of(
        &mesh,
        &features,
        the_convex_edge_of(
            &features,
            top_corner(),
            Vector3::new(10.0, 20.0, 10.0),
            1e-6,
        ),
        RADIUS,
    );
    let concave = plan_of(&mesh, &features, the_concave_edge(&features), RADIUS);

    let corner = SurfaceMixedCorner::try_read(&convex, &concave, top_corner(), RADIUS, near_enough)
        .unwrap_or_else(|failure| panic!("the corner would not read: {failure:?}"));
    let other_way_round =
        SurfaceMixedCorner::try_read(&concave, &convex, top_corner(), RADIUS, near_enough)
            .unwrap_or_else(|failure| panic!("{failure:?}"));
    assert!((other_way_round.tangent_point - corner.tangent_point).length() <= 1e-12);

    // The ball rolls round the bead's cylinder: every spine centre two radii off its axis and a
    // radius inside the flat cap.
    for station in 0..=16 {
        let at = corner
            .spine
            .centre_at(f64::from(station) * corner.turn_radians() / 16.0);
        assert!(
            (corner.axis().distance_to(at) - 2.0 * RADIUS).abs() <= 1e-6,
            "{at:?}"
        );
        assert!(
            ((at - top_corner()).dot(Vector3::new(0.0, 0.0, 1.0)) + RADIUS).abs() <= 1e-6,
            "{at:?}"
        );
    }
}

fn a_cap_that_leans_is_read_as_an_ellipse(scale: f64) {
    let mut mesh = create_l_shaped_grid_prism();
    if scale != 1.0 {
        for at in mesh.vertices.iter_mut() {
            *at = Vector3Float::new(
                f64::from(at.x) * scale,
                f64::from(at.y) * scale,
                f64::from(at.z) * scale,
            );
        }
    }

    // Tilt the cap about the corner's own wall: z = 10 + (y - 10) / 4.
    for at in mesh.vertices.iter_mut() {
        if (f64::from(at.z) - (10.0 * scale)).abs() < 1e-6 * scale {
            *at = Vector3Float::new(
                f64::from(at.x),
                f64::from(at.y),
                (10.0 * scale) + (0.25 * (f64::from(at.y) - (10.0 * scale))),
            );
        }
    }

    let mesh = Arc::new(mesh);
    let features = FeatureEdgeSet::create(&mesh);
    let near_enough = SurfaceVertexPool::near_enough_for(&mesh.vertices);
    let at_the_corner = top_corner() * scale;
    let convex = plan_of(
        &mesh,
        &features,
        the_convex_edge_of(
            &features,
            at_the_corner,
            Vector3::new(10.0, 20.0, 12.5) * scale,
            1e-6 * scale,
        ),
        RADIUS * scale,
    );
    let concave = plan_of(
        &mesh,
        &features,
        the_concave_edge(&features),
        RADIUS * scale,
    );

    let corner = SurfaceMixedCorner::try_read(
        &convex,
        &concave,
        at_the_corner,
        RADIUS * scale,
        near_enough,
    )
    .unwrap_or_else(|failure| {
        panic!("a leaning cap is a reading now, not a refusal - instead {failure:?}")
    });

    let out_of_the_cap = Vector3::new(0.0, -0.25, 1.0).get_normal();
    for station in 0..=16 {
        let at = corner
            .spine
            .centre_at(f64::from(station) * corner.turn_radians() / 16.0);
        assert!(
            (corner.axis().distance_to(at) - 2.0 * RADIUS * scale).abs() <= 1e-6 * scale,
            "the ball touches the bead's cylinder all the way round - {at:?} does not"
        );
        assert!(
            ((at - at_the_corner).dot(out_of_the_cap) + RADIUS * scale).abs() <= 1e-6 * scale,
            "and stands a radius inside the LEANING cap - {at:?} does not"
        );
    }
}

#[test]
fn rust_only_a_cap_that_leans_is_read_as_an_ellipse_at_unit_scale() {
    a_cap_that_leans_is_read_as_an_ellipse(1.0);
}

#[test]
fn rust_only_a_cap_that_leans_is_read_as_an_ellipse_at_a_hundred_times() {
    a_cap_that_leans_is_read_as_an_ellipse(100.0);
}

#[test]
fn rust_only_the_plan_reads_the_real_mixed_corner_itself() {
    // The convex rim and the concave inside edge planned as ONE chain: TryCreate reads the corner
    // where the convexity turns over, and it is the corner the two edges read by hand.
    let mesh = Arc::new(create_l_shaped_grid_prism());
    let features = FeatureEdgeSet::create(&mesh);
    let near_enough = SurfaceVertexPool::near_enough_for(&mesh.vertices);
    let convex_index = the_convex_edge_of(
        &features,
        top_corner(),
        Vector3::new(10.0, 20.0, 10.0),
        1e-6,
    );
    let concave_index = the_concave_edge(&features);
    let regions = Arc::new(FaceRegionCache::new(
        mesh.clone(),
        None,
        features.flat_angle_degrees(),
    ));
    let chains = bevel_chain_linker::link(&[
        features.edges()[convex_index],
        features.edges()[concave_index],
    ]);
    assert_eq!(chains.len(), 1, "the two edges meet at the corner");
    let plan = SurfaceBevelPlan::try_create(&chains[0], &regions, RADIUS, true)
        .unwrap_or_else(|reason| panic!("the chain would not plan: {reason:?}"));

    assert_eq!(plan.edges.len(), 2);
    assert_eq!(plan.mixed_corner_count(), 1);
    let read = plan
        .try_mixed_corner_after(0)
        .expect("the corner is keyed by the edge the chain arrives on");
    assert!((read.corner - top_corner()).length() <= 1e-9);

    let by_hand = SurfaceMixedCorner::try_read(
        &plan_of(&mesh, &features, convex_index, RADIUS),
        &plan_of(&mesh, &features, concave_index, RADIUS),
        top_corner(),
        RADIUS,
        near_enough,
    )
    .unwrap_or_else(|failure| panic!("{failure:?}"));
    assert!((read.tangent_point - by_hand.tangent_point).length() <= 1e-12);
}
