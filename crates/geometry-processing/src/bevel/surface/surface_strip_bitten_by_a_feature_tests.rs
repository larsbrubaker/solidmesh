//! Unit tests of `surface_strip_bitten_by_a_feature`'s private tip reading (`TryTheTip`), which
//! needs private access: where a refusal is named when the ridge runs along the quad.

use std::rc::Rc;
use std::sync::Arc;

use polygon_mesh::solids::platonic_solids;
use vector_math::Vector3;

use super::super::{
    SurfaceEdgePlan, SurfaceLine, SurfaceStandingFeatureWithheld,
    SurfaceStandingFeatureWithheldFace, SurfaceStrip, SurfaceVertexPool,
};
use super::SurfaceStripBittenByAFeature;
use crate::topology::{ChainEdge, EdgeConvexity, FaceGroup, FeatureEdge};

/// A square convex edge along +x from the origin, chamfered at 1: the left face is the top
/// (inward +y) and the right face the front (inward -z), so the strip's one quad leans at 45
/// degrees and runs along +x.
fn square_edge_plan() -> SurfaceEdgePlan {
    let feature = FeatureEdge {
        start: Vector3::new(0.0, 0.0, 0.0),
        end: Vector3::new(10.0, 0.0, 0.0),
        direction: Vector3::new(1.0, 0.0, 0.0),
        normal0: Vector3::new(0.0, 0.0, 1.0),
        normal1: Vector3::new(0.0, -1.0, 0.0),
        turn_angle_degrees: 90.0,
        convexity: EdgeConvexity::Convex,
        ..Default::default()
    };
    let edge = ChainEdge::new(0, feature, false);

    // Any region will do: the strip and the tip read none of it.
    let mesh = platonic_solids::create_cube(2.0, 2.0, 2.0);
    let region = Arc::new(FaceGroup::create(&mesh, 0).expect("a cube face floods"));

    let along = Vector3::new(1.0, 0.0, 0.0);
    let left_inward = Vector3::new(0.0, 1.0, 0.0);
    let right_inward = Vector3::new(0.0, 0.0, -1.0);

    SurfaceEdgePlan::new(
        edge,
        region.clone(),
        region,
        90.0,
        1.0,
        1.0,
        left_inward,
        right_inward,
        left_inward,
        right_inward,
        SurfaceLine::new(left_inward, along),
        SurfaceLine::new(right_inward, along),
        false,
        SurfaceLine::new(left_inward + right_inward, along),
        0.0,
        None,
        None,
        None,
    )
}

/// A feature of two walls, y = 0.5 and z = -0.5, whose ridge runs along +x: parallel to the
/// strip's quad.
fn a_ridge_along_the_quad() -> SurfaceStandingFeatureWithheld {
    let wall = |normal: Vector3| {
        Rc::new(SurfaceStandingFeatureWithheldFace::new(
            vec![0],
            (0, 1),
            Vec::new(),
            Vector3::new(5.0, 0.5, -0.5),
            normal,
        ))
    };

    SurfaceStandingFeatureWithheld::new(
        (0, true),
        Vector3::new(0.0, 0.0, 1.0),
        vec![
            wall(Vector3::new(0.0, 1.0, 0.0)),
            wall(Vector3::new(0.0, 0.0, -1.0)),
        ],
    )
}

#[test]
fn rust_only_a_ridge_along_the_quad_is_refused_at_the_last_rail() {
    let edge = square_edge_plan();
    let strip = SurfaceStrip::try_create(&edge, 4).expect("the chamfer strip");
    let feature = a_ridge_along_the_quad();
    assert!(feature.try_the_ridge().is_some(), "the two walls meet");

    let mut pool = SurfaceVertexPool::new(&[]);
    let refused_at =
        SurfaceStripBittenByAFeature::try_the_tip(&mut pool, &strip, &edge, &feature, 0, 1, 1e-9)
            .expect_err("a ridge along the quad never leaves it");

    // The refusal names a place on the strip, at the last covered rail, never the world origin.
    assert_eq!(refused_at, strip.point_on(0, 0.0));
    assert_ne!(refused_at, Vector3::ZERO);
}
