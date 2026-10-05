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
pub fn square_edge_plan(size: f64, round: bool) -> SurfaceEdgePlan {
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

#[test]
fn rust_only_a_chained_run_wraps_only_when_closed() {
    use geometry_processing::bevel::surface::surface_mitre_on_a_chorded_run::{
        try_the_next, try_the_previous,
    };

    let edges = vec![square_edge_plan(1.0, false), square_edge_plan(1.0, false)];

    assert_eq!(try_the_next(&edges, false, 0), Some(1));
    assert_eq!(try_the_next(&edges, false, 1), None);
    assert_eq!(try_the_next(&edges, true, 1), Some(0));
    assert_eq!(try_the_previous(&edges, false, 0), None);
    assert_eq!(try_the_previous(&edges, true, 0), Some(1));

    // A lone edge never wraps onto itself.
    assert_eq!(try_the_next(&edges[..1], true, 0), None);
}

#[test]
fn rust_only_two_pieces_of_one_crease_are_one_line() {
    use geometry_processing::bevel::surface::surface_mitre_seam;

    // The same plan twice shares both faces and lies on the same two lines: one line, and no
    // crossing to read off parallel lines.
    let plan = square_edge_plan(1.0, false);
    assert!(surface_mitre_seam::the_lines_are_one(&plan, &plan, 1e-9));
    assert!(surface_mitre_seam::try_the_point(&plan, &plan, 1e-9).is_none());
}

#[test]
fn rust_only_three_end_rings_close_into_one_loop() {
    use geometry_processing::bevel::surface::surface_corner_patch::try_order_the_sides;

    // The second ring meets the first's end by its own far end, so it is turned over.
    let sides = try_order_the_sides(&[vec![1, 2, 3], vec![5, 4, 3], vec![5, 6, 1]]).unwrap();
    assert_eq!(sides, vec![vec![1, 2, 3], vec![3, 4, 5], vec![5, 6, 1]]);

    // Rings that do not close are refused.
    assert!(try_order_the_sides(&[vec![1, 2, 3], vec![3, 4, 5], vec![5, 6, 7]]).is_none());
}

#[test]
fn rust_only_a_closed_tetrahedron_covers_nothing_twice() {
    use geometry_processing::bevel::surface::surface_mixed_corner_patch::nothing_covers_nothing;
    use geometry_processing::bevel::surface::SurfaceVertexPool;

    let mut pool = SurfaceVertexPool::new(&[]);
    let a = pool.add(Vector3::new(0.0, 0.0, 0.0));
    let b = pool.add(Vector3::new(1.0, 0.0, 0.0));
    let c = pool.add(Vector3::new(0.0, 1.0, 0.0));
    let d = pool.add(Vector3::new(0.0, 0.0, 1.0));

    let closed = [(a, c, b), (a, b, d), (b, c, d), (c, a, d)];
    assert!(nothing_covers_nothing(&pool, &closed).is_ok());

    // A face twice over, and one wound the same way as its neighbour, are both refused.
    assert!(nothing_covers_nothing(&pool, &[(a, c, b), (c, b, a)]).is_err());
    assert!(nothing_covers_nothing(&pool, &[(a, b, c), (a, b, d)]).is_err());
}

#[test]
fn rust_only_skew_lines_and_size_windows() {
    use geometry_processing::bevel::surface::surface_mixed_corner_cut_edge::try_apart;
    use geometry_processing::bevel::surface::surface_wall_ran_out::bound;

    // Two skew lines a unit apart along z.
    let apart = try_apart(
        SurfaceLine::new(Vector3::new(0.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0)),
        SurfaceLine::new(Vector3::new(0.0, 0.0, 1.0), Vector3::new(0.0, 1.0, 0.0)),
    )
    .unwrap();
    assert!((apart - 1.0).abs() < 1e-15);

    // size - 2 >= 0 raises the floor to 2; 3 - size >= 0 lowers the ceiling to 3.
    let (mut lowest, mut highest) = (0.0, f64::INFINITY);
    bound(1.0, -2.0, &mut lowest, &mut highest);
    bound(-1.0, 3.0, &mut lowest, &mut highest);
    assert_eq!((lowest, highest), (2.0, 3.0));

    // A condition that never holds whatever the size closes the window.
    bound(0.0, -1.0, &mut lowest, &mut highest);
    assert_eq!(highest, f64::NEG_INFINITY);
}
