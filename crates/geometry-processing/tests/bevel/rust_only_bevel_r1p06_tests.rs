//! Rust-only checks of the bevel core's R1.P06 package (the edge plan's constructor, the rows and
//! canal frames of a mixed corner, mitre seams). No C# test class is runnable on these methods
//! alone, so these pin the closed forms the C# remarks state. They also give `SurfaceStrip`'s
//! construction (R1.P03) the hand-built edge plan it was waiting on.

use std::sync::Arc;

use geometry_processing::bevel::surface::{SurfaceEdgePlan, SurfaceLine, SurfaceStrip};
use geometry_processing::topology::{ChainEdge, EdgeConvexity, FaceGroup, FeatureEdge};
use vector_math::Vector3;

use crate::topology_test_meshes::create_isosceles_prism;

/// A square convex edge along +x through the origin, the material in y > 0, z < 0: the left face
/// is the top (normal +z, inward +y) and the right face the front (normal -y, inward -z). Planned
/// at `size` as a chamfer, or as a round when `round`.
fn square_edge_plan(size: f64, round: bool) -> SurfaceEdgePlan {
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

    // Any region will do: the strip reads none of it.
    let mesh = create_isosceles_prism(90.0, 10.0, 5.0);
    let region = Arc::new(FaceGroup::create(&mesh, 0).unwrap());

    let along = Vector3::new(1.0, 0.0, 0.0);
    let left_inward = Vector3::new(0.0, 1.0, 0.0);
    let right_inward = Vector3::new(0.0, 0.0, -1.0);

    SurfaceEdgePlan::new(
        edge,
        region.clone(),
        region,
        90.0,
        size,
        size,
        left_inward,
        right_inward,
        left_inward,
        right_inward,
        SurfaceLine::new(left_inward * size, along),
        SurfaceLine::new(right_inward * size, along),
        round,
        SurfaceLine::new((left_inward + right_inward) * size, along),
        if round { size * 2f64.sqrt() } else { 0.0 },
        None,
        None,
        None,
    )
}

#[test]
fn rust_only_a_chamfer_strip_is_ruled_by_its_two_feet() {
    let strip = SurfaceStrip::try_create(&square_edge_plan(1.0, false), 4).unwrap();

    assert_eq!(
        strip.rail_points,
        vec![Vector3::new(0.0, 1.0, 0.0), Vector3::new(0.0, 0.0, -1.0)]
    );
    assert_eq!(strip.rail_step_degrees, 0.0);

    // Facing out of the material, along the wedge's bisector.
    let half = 0.5f64.sqrt();
    assert!((strip.outward_reference - Vector3::new(0.0, -half, half)).length() < 1e-12);
}

#[test]
fn rust_only_a_round_strip_samples_its_quarter_arc() {
    let strip = SurfaceStrip::try_create(&square_edge_plan(1.0, true), 4).unwrap();
    let center = Vector3::new(0.0, 1.0, -1.0);

    assert_eq!(strip.rail_points.len(), 5);
    assert!((strip.rail_step_degrees - 22.5).abs() < 1e-9);

    // The ends are the feet themselves, and every rail stands a radius from the centre line.
    assert_eq!(strip.rail_points[0], Vector3::new(0.0, 1.0, 0.0));
    assert_eq!(strip.rail_points[4], Vector3::new(0.0, 0.0, -1.0));
    for rail in &strip.rail_points {
        assert!(((*rail - center).length() - 1.0).abs() < 1e-12);
    }

    // The middle rail is the arc's midpoint, out along the bisector.
    let half = 0.5f64.sqrt();
    assert!((strip.rail_points[2] - (center + Vector3::new(0.0, -half, half))).length() < 1e-12);
}

#[test]
fn rust_only_the_canal_bisects_to_the_sign_change() {
    use geometry_processing::bevel::surface::{
        SurfaceMixedCornerDualCanal, SurfaceMixedCornerRows,
    };

    let at = SurfaceMixedCornerDualCanal::bisect(&|x| x - 0.3, 0.0, 1.0, false);
    assert!((at - 0.3).abs() < 1e-15);

    // Square out of the axis, the row sweeps a quarter turn from the plate to the bead.
    let sweep =
        SurfaceMixedCornerRows::sweep_at(Vector3::new(0.0, 0.0, 1.0), Vector3::new(-1.0, 0.0, 0.0));
    assert!((sweep - std::f64::consts::FRAC_PI_2).abs() < 1e-15);
}
