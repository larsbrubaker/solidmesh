//! `SurfaceMixedCorner` read off the L prism's top
//! corner, number by number. At `V = (10, 10, 10)` the cap `T` (`z = 10`), the shared step wall
//! `S` (`x = 10`) and the void wall `W` (`y = 10`) meet; the picked convex edge is `T inter S` out
//! to `(10, 20, 10)`, the picked concave edge `S inter W` down to `(10, 10, 0)`. At `r = 1` the
//! cut's centre line is `(9, y, 9)`, the bead's axis `(11, 11, z)`, the tangent point
//! `P = (10, 11, 9)`, the handover plane `y = 11` and the bead's stop plane `z = 9`.

use std::sync::Arc;

use solidmesh::bevel::bevel_chain_linker;
use solidmesh::bevel::surface::{
    SurfaceBevelPlan, SurfaceEdgePlan, SurfaceMixedCorner, SurfaceVertexPool,
};
use solidmesh::bevel::{BevelAdvisoryArg, BevelAdvisoryCode};
use solidmesh::topology::{EdgeConvexity, FaceRegionCache, FeatureEdgeSet};
use solidmesh_math::math_helper::radians_to_degrees;
use solidmesh_math::{Vector3, Vector3Float};
use solidmesh_mesh::Mesh;

use crate::topology_test_meshes::create_l_shaped_grid_prism;

const RADIUS: f64 = 1.0;

const EXACT: f64 = 1e-9;

fn top_corner() -> Vector3 {
    Vector3::new(10.0, 10.0, 10.0)
}

fn equals(a: Vector3, b: Vector3, tolerance: f64) -> bool {
    a.equals_within(b, tolerance)
}

fn assert_near(actual: f64, expected: f64, within: f64, why: &str) {
    assert!(
        (actual - expected).abs() <= within,
        "{why}: expected {expected} within {within}, got {actual}"
    );
}

/// The two cylinders stand `2r` apart and touch at the corner the shared face is left with.
#[test]
fn the_two_cylinders_are_tangent_at_the_shared_faces_own_corner() {
    let corner = the_mixed_corner();

    assert!(
        equals(corner.corner, top_corner(), EXACT),
        "instead {:?}",
        corner.corner
    );
    assert_near(corner.size, RADIUS, EXACT, "size");

    // The cut's centre line and the bead's axis, each in its own place.
    let cut = corner.cut_center_line();
    assert!(
        cut.distance_to(Vector3::new(9.0, 0.0, 9.0)) < EXACT,
        "the cut's centre line runs along (9, y, 9) - instead {:?} along {:?}",
        cut.point,
        cut.direction
    );
    let axis = corner.axis();
    assert!(
        axis.distance_to(Vector3::new(11.0, 11.0, 0.0)) < EXACT,
        "the bead's axis runs along (11, 11, z) - instead {:?} along {:?}",
        axis.point,
        axis.direction
    );

    assert!(
        cut.direction.dot(axis.direction).abs() < EXACT,
        "the two axes are square to each other"
    );

    // Which makes their common perpendicular 2r long, and its midpoint the one point they touch at.
    assert!(
        equals(corner.tangent_point, Vector3::new(10.0, 11.0, 9.0), EXACT),
        "instead {:?}",
        corner.tangent_point
    );

    assert_near(cut.distance_to(corner.tangent_point), RADIUS, EXACT, "cut");
    assert_near(
        axis.distance_to(corner.tangent_point),
        RADIUS,
        EXACT,
        "axis",
    );

    // And it lies on the face the two edges share.
    assert_near(corner.tangent_point.x, 10.0, EXACT, "P stands on S");
}

/// Each strip stops on the plane through that point square to the OTHER edge's direction: the
/// cut on the handover plane `y = 11`, the bead on `z = 9`, each facing away from the corner.
#[test]
fn each_strip_stops_on_the_plane_through_that_point_facing_away_from_the_corner() {
    let corner = the_mixed_corner();

    assert!(equals(
        corner.cut_stops_at.point,
        corner.tangent_point,
        EXACT
    ));
    assert!(equals(
        corner.bead_stops_at.point,
        corner.tangent_point,
        EXACT
    ));

    // The cut's band runs out along the convex edge, from the corner towards (10, 20, 10).
    assert!(
        equals(
            corner.cut_stops_at.inward,
            Vector3::new(0.0, 1.0, 0.0),
            EXACT
        ),
        "instead {:?}",
        corner.cut_stops_at.inward
    );

    // The bead's runs down the concave edge, from the corner towards (10, 10, 0).
    assert!(
        equals(
            corner.bead_stops_at.inward,
            Vector3::new(0.0, 0.0, -1.0),
            EXACT
        ),
        "instead {:?}",
        corner.bead_stops_at.inward
    );

    // Read as planes: y = 11 and z = 9, keeping y >= 11 and z <= 9.
    assert_near(
        corner
            .cut_stops_at
            .signed_distance(Vector3::new(10.0, 20.0, 10.0)),
        9.0,
        EXACT,
        "cut far end",
    );
    assert_near(
        corner.cut_stops_at.signed_distance(top_corner()),
        -RADIUS,
        EXACT,
        "cut corner",
    );
    assert_near(
        corner
            .bead_stops_at
            .signed_distance(Vector3::new(10.0, 10.0, 0.0)),
        9.0,
        EXACT,
        "bead far end",
    );
    assert_near(
        corner.bead_stops_at.signed_distance(top_corner()),
        -RADIUS,
        EXACT,
        "bead corner",
    );

    // And the handover really is the plane through the bead's axis: the axis lies IN it.
    assert!(
        corner.cut_stops_at.signed_distance(corner.axis().point) < EXACT,
        "the handover plane contains the bead's axis"
    );
}

/// The ball turns a right angle between the two strips - 180 degrees less the bead's own void
/// wedge - and its touch point on the cap traces the arc of radius `2r` about the bead's axis.
#[test]
fn the_ball_turns_the_void_wedges_complement_and_its_touch_point_traces_the_outer_rail() {
    let corner = the_mixed_corner();
    let turn = corner.turn_radians();

    assert_near(radians_to_degrees(turn), 90.0, 1e-9, "turn");
    assert_near(
        radians_to_degrees(turn),
        180.0 - corner.bead.wedge_angle_degrees,
        1e-9,
        &format!(
            "the void wedge the bead sits in is {}",
            corner.bead.wedge_angle_degrees
        ),
    );

    assert!(
        equals(
            corner.outer_rail_centre,
            Vector3::new(11.0, 11.0, 10.0),
            EXACT
        ),
        "the arc is centred where the bead's axis meets the cap - instead {:?}",
        corner.outer_rail_centre
    );
    assert_near(
        corner.outer_rail_radius(),
        2.0 * RADIUS,
        EXACT,
        "rail radius",
    );
    assert!(equals(
        corner.out_of_the_cap_face,
        Vector3::new(0.0, 0.0, 1.0),
        EXACT
    ));
    assert!(equals(
        corner.towards_the_shared_face(),
        Vector3::new(-1.0, 0.0, 0.0),
        EXACT
    ));

    assert!(
        equals(
            corner.outer_rail_point(0.0),
            Vector3::new(9.0, 11.0, 10.0),
            EXACT
        ),
        "instead {:?}",
        corner.outer_rail_point(0.0)
    );
    assert!(
        equals(
            corner.outer_rail_point(turn),
            Vector3::new(11.0, 9.0, 10.0),
            EXACT
        ),
        "instead {:?}",
        corner.outer_rail_point(turn)
    );

    // Every station of it is on the cap face, 2r from the axis.
    for station in 0..=8 {
        let at = corner.outer_rail_point(f64::from(station) * turn / 8.0);

        assert_near(at.z, 10.0, EXACT, "on the cap");
        assert_near(
            (at - corner.outer_rail_centre).length(),
            2.0 * RADIUS,
            EXACT,
            "2r from the axis",
        );
    }

    // At turn zero the ball's centre is on the CUT's own centre line, and at the full turn it is
    // on the centre line the edge nobody picked would have had.
    assert!(
        corner
            .cut_center_line()
            .distance_to(corner.ball_centre_at(0.0))
            < EXACT,
        "instead {:?}",
        corner.ball_centre_at(0.0)
    );
    assert!(
        equals(
            corner.ball_centre_at(turn),
            Vector3::new(11.0, 9.0, 9.0),
            EXACT
        ),
        "instead {:?}",
        corner.ball_centre_at(turn)
    );

    // And the tangent point is where the ball at turn zero touches the shared face.
    assert_near(
        (corner.tangent_point - corner.ball_centre_at(0.0)).length(),
        RADIUS,
        EXACT,
        "tangent point",
    );
}

/// The reading refuses what it cannot describe: a CHAMFER's two strips have no centre line to
/// roll a ball along, so the same corner comes back by name.
#[test]
fn a_chamfers_mixed_corner_is_refused_by_name() {
    let (convex, concave, near_enough) = the_planned_edges(false);

    let failure =
        match SurfaceMixedCorner::try_read(&convex, &concave, top_corner(), RADIUS, near_enough) {
            Ok(_) => panic!("a chamfer's mixed corner read"),
            Err(failure) => failure,
        };
    assert_eq!(failure.code(), BevelAdvisoryCode::SurfaceCornerNotSupported);
    match failure.args()[0] {
        BevelAdvisoryArg::Vector3(at) => assert!((at - top_corner()).length() < EXACT),
        ref other => panic!("the first argument is not a point: {other:?}"),
    }
}

/// A corner whose CAP is not square to the bead's axis READS: the ball's centres are the plane a
/// radius inside the cap met with the cylinder of radius `2r` about the axis. Run at the L's own
/// size and at a hundred times it, with the same 14 degree lean.
fn a_cap_that_leans_is_read_as_an_ellipse_at_every_scale(scale: f64) {
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

    // Tilt the cap about the corner's own wall, so that the corner vertex and both picked edges
    // stay where they are and only the face the outer rail would run across leans.
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
    let regions = regions_of(&mesh, &features);
    let near_enough = SurfaceVertexPool::near_enough_for(&mesh.vertices);
    let at_the_corner = top_corner() * scale;

    let convex = plan_of(
        &regions,
        &features,
        the_convex_edge_of(
            &features,
            at_the_corner,
            Vector3::new(10.0, 20.0, 12.5) * scale,
            1e-6 * scale,
        ),
        true,
        RADIUS * scale,
    );
    let concave = plan_of(
        &regions,
        &features,
        the_concave_edge(&features),
        true,
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

    // The cap the mesh was tilted to: z = 10 + (y - 10) / 4, so its outward normal leans 14 degrees.
    let out_of_the_cap = Vector3::new(0.0, -0.25, 1.0).get_normal();

    for station in 0..=16 {
        let at = corner
            .spine
            .centre_at(f64::from(station) * corner.turn_radians() / 16.0);

        assert_near(
            corner.axis().distance_to(at),
            2.0 * RADIUS * scale,
            1e-6 * scale,
            &format!("the ball touches the bead's cylinder all the way round - {at:?} does not"),
        );
        assert_near(
            (at - at_the_corner).dot(out_of_the_cap),
            -RADIUS * scale,
            1e-6 * scale,
            &format!("and stands a radius inside the LEANING cap - {at:?} does not"),
        );
    }
}

#[test]
fn a_cap_that_leans_is_read_as_an_ellipse_at_every_scale_at_scale_1() {
    a_cap_that_leans_is_read_as_an_ellipse_at_every_scale(1.0);
}

#[test]
fn a_cap_that_leans_is_read_as_an_ellipse_at_every_scale_at_scale_100() {
    a_cap_that_leans_is_read_as_an_ellipse_at_every_scale(100.0);
}

/// The corner read off the two edges' own plans, in either order: which of the two the chain
/// arrives on is the chain's business and not the corner's.
pub fn the_mixed_corner() -> SurfaceMixedCorner {
    let (convex, concave, near_enough) = the_planned_edges(true);

    let corner = SurfaceMixedCorner::try_read(&convex, &concave, top_corner(), RADIUS, near_enough)
        .unwrap_or_else(|failure| panic!("the corner would not read: {failure:?}"));

    match SurfaceMixedCorner::try_read(&concave, &convex, top_corner(), RADIUS, near_enough) {
        Ok(other_way_round)
            if (other_way_round.tangent_point - corner.tangent_point).length() <= EXACT => {}
        _ => panic!("the corner reads differently arrived at the other way round"),
    }

    corner
}

fn the_planned_edges(round_profile: bool) -> (SurfaceEdgePlan, SurfaceEdgePlan, f64) {
    let mesh = Arc::new(create_l_shaped_grid_prism());
    let features = FeatureEdgeSet::create(&mesh);
    let regions = regions_of(&mesh, &features);

    (
        plan_of(
            &regions,
            &features,
            the_convex_edge(&features),
            round_profile,
            RADIUS,
        ),
        plan_of(
            &regions,
            &features,
            the_concave_edge(&features),
            round_profile,
            RADIUS,
        ),
        SurfaceVertexPool::near_enough_for(&mesh.vertices),
    )
}

fn regions_of(mesh: &Arc<Mesh>, features: &FeatureEdgeSet) -> Arc<FaceRegionCache> {
    Arc::new(FaceRegionCache::new(
        mesh.clone(),
        Some(mesh.get_mesh_edge_graph()),
        features.flat_angle_degrees(),
    ))
}

fn plan_of(
    regions: &Arc<FaceRegionCache>,
    features: &FeatureEdgeSet,
    edge_index: usize,
    round_profile: bool,
    radius: f64,
) -> SurfaceEdgePlan {
    let mut chains = bevel_chain_linker::link(&[features.edges()[edge_index]]);
    assert_eq!(chains.len(), 1);
    let chain = chains.remove(0);

    let plan = SurfaceBevelPlan::try_create(&chain, regions, radius, round_profile)
        .unwrap_or_else(|reason| panic!("the plan refused the edge: {reason:?}"));

    plan.edges[0].clone()
}

/// The one reflex edge of the L - the bead's.
fn the_concave_edge(features: &FeatureEdgeSet) -> usize {
    single(features, |i| {
        features.edges()[i].convexity == EdgeConvexity::Concave
    })
}

/// The top rim of the step wall, from the corner out to (10, 20, 10) - the cut's.
fn the_convex_edge(features: &FeatureEdgeSet) -> usize {
    the_convex_edge_of(features, top_corner(), Vector3::new(10.0, 20.0, 10.0), 1e-6)
}

/// The same, on a mesh scaled or tilted, where both of the edge's ends have moved.
fn the_convex_edge_of(
    features: &FeatureEdgeSet,
    corner: Vector3,
    far: Vector3,
    meets: f64,
) -> usize {
    single(features, |i| {
        let edge = &features.edges()[i];

        ((edge.start - corner).length() < meets && (edge.end - far).length() < meets)
            || ((edge.end - corner).length() < meets && (edge.start - far).length() < meets)
    })
}

fn single(features: &FeatureEdgeSet, matches: impl Fn(usize) -> bool) -> usize {
    let found: Vec<usize> = (0..features.count()).filter(|&i| matches(i)).collect();
    assert_eq!(found.len(), 1, "expected exactly one edge, found {found:?}");
    found[0]
}
