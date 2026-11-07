//! The surface bevel's plan, checked against numbers worked
//! on paper. Every case here is one the 2D bevel already answers at a corner - setback
//! r/tan(half the wedge), center r/sin(half the wedge), the tangent points the feet of that
//! center - read in 3D as lines along an edge, so what these tests really pin is that the 3D
//! construction is the 2D one and not a second derivation of it.

use std::sync::Arc;

use geometry_processing::bevel::bevel_chain_linker::{self, BevelSweptChain};
use geometry_processing::bevel::surface::surface_plan_band_ends;
use geometry_processing::bevel::surface::{SurfaceBevelPlan, SurfaceEdgePlan, SurfaceLine};
use geometry_processing::bevel::BevelAdvisoryCode;
use geometry_processing::topology::{
    ChainEdge, EdgeConvexity, FaceGroup, FaceRegionCache, FeatureEdge, FeatureEdgeSet,
};
use polygon_mesh::solids::platonic_solids;
use polygon_mesh::Mesh;
use vector_math::{Vector2, Vector3};

use crate::topology_test_meshes::{create_isosceles_prism, create_l_shaped_prism};

/// Tight, because everything here is a closed form and nothing is iterated to.
const EXACT: f64 = 1e-12;

/// A box edge opens a 90 degree wedge, so a chamfer sets back by its size and a fillet by its
/// radius (r/tan(45) = r). The round's center sits at r/sin(45) along the bisector, which is r
/// from both face planes, and each tangent line is that center's perpendicular foot on its face.
#[test]
fn a_cube_edge_sets_both_its_faces_back_by_the_size_and_hangs_the_centre_on_the_bisector() {
    let cube = platonic_solids::create_cube(20.0, 20.0, 20.0);
    let (regions, features) = regions_of(&cube);
    let chain = chain_of(&features, top_front_edge(&features));

    let chamfer =
        SurfaceBevelPlan::try_create(&chain, &regions, 3.0, false).unwrap_or_else(|reason| {
            panic!("a box edge is the simplest thing a bevel plans - {reason:?}")
        });
    assert_eq!(chamfer.edges.len(), 1);

    let chamfer_edge = &chamfer.edges[0];
    assert!((chamfer_edge.wedge_angle_degrees - 90.0).abs() <= 1e-9);
    assert_eq!(chamfer_edge.convexity(), EdgeConvexity::Convex);
    assert!(
        (chamfer_edge.setback() - 3.0).abs() <= EXACT,
        "a chamfer's setback IS its size, on both faces"
    );
    assert!(
        !chamfer_edge.has_center_line,
        "a straight cut has no centre to swing about"
    );

    let fillet = SurfaceBevelPlan::try_create(&chain, &regions, 3.0, true)
        .unwrap_or_else(|reason| panic!("{reason:?}"));

    let fillet_edge = &fillet.edges[0];
    assert!(
        (fillet_edge.setback() - 3.0).abs() <= EXACT,
        "r / tan(45 degrees) is r"
    );
    assert!(fillet_edge.has_center_line);
    assert!(
        (fillet_edge.center_distance - 3.0 * 2f64.sqrt()).abs() <= EXACT,
        "r / sin(45 degrees) is r root two"
    );

    // The centre stands 3 in from the top face and 3 in from the front face, which on this edge
    // means (y, z) = (-7, 7) - the corner of the 3 mm square inscribed in the box's corner.
    let centre = fillet_edge.center_line.point;
    assert!(
        (centre - Vector3::new(centre.x, -7.0, 7.0)).length() < 1e-9,
        "the centre of a 3 mm fillet on the top front edge of a 20 mm cube is 3 in from each face, at {centre}"
    );

    assert_the_centre_is_tangent_to_both_faces(fillet_edge, 3.0);

    // The same check the L's concave corner gets: a tangent line has to lie IN the face it walks
    // back, on the side the material is on, convex edge or not.
    assert_the_tangent_line_is_inside_its_face(
        &cube,
        fillet_edge,
        &fillet_edge.left_region,
        &fillet_edge.left_tangent_line,
    );
    assert_the_tangent_line_is_inside_its_face(
        &cube,
        fillet_edge,
        &fillet_edge.right_region,
        &fillet_edge.right_tangent_line,
    );
}

/// The formulas are not a right angle in disguise. A 60 degree wedge sets back r/tan(30) =
/// 1.732 r and hangs its centre at r/sin(30) = 2 r.
#[test]
fn a_sixty_degree_wedge_sets_back_by_radius_over_tan_thirty_and_centres_at_twice_the_radius() {
    let prism = create_isosceles_prism(60.0, 20.0, 10.0);
    let (regions, features) = regions_of(&prism);
    let chain = chain_of(&features, apex_edge(&features));

    let plan = SurfaceBevelPlan::try_create(&chain, &regions, 2.0, true)
        .unwrap_or_else(|reason| panic!("{reason:?}"));

    // A mesh stores its vertices as floats, so the fixture's apex is only 60 degrees to about a
    // millionth of one - which is the tolerance on the angle itself and on everything read off it.
    let apex = &plan.edges[0];
    assert!((apex.wedge_angle_degrees - 60.0).abs() <= 1e-5);
    assert!(
        (apex.dihedral_angle_degrees() - 60.0).abs() <= 1e-5,
        "on a convex edge the wedge the profile is drawn in IS the dihedral"
    );
    assert!((apex.setback() - 2.0 / 30f64.to_radians().tan()).abs() <= 1e-6);
    assert!(
        (apex.setback() - 3.4641016151).abs() <= 1e-6,
        "the same number written out, so a change of formula cannot hide behind a change of test"
    );
    assert!(
        (apex.center_distance - 4.0).abs() <= 1e-6,
        "2 / sin(30 degrees) is 4"
    );

    assert_the_centre_is_tangent_to_both_faces(apex, 2.0);
}

/// A concave edge takes the same two numbers as a convex one of the same wedge - the L's inner
/// corner turns 90 the other way, so its VOID wedge is 90 and its setback is the size again.
#[test]
fn the_inner_corner_of_an_l_plans_the_same_setbacks_as_a_box_edge_and_lands_inside_its_faces() {
    let prism = create_l_shaped_prism();
    let (regions, features) = regions_of(&prism);
    let concave: Vec<usize> = (0..features.count())
        .filter(|&i| features.edges()[i].convexity == EdgeConvexity::Concave)
        .collect();
    assert_eq!(concave.len(), 1);
    let chain = chain_of(&features, concave[0]);

    let plan = SurfaceBevelPlan::try_create(&chain, &regions, 3.0, true)
        .unwrap_or_else(|reason| panic!("{reason:?}"));

    let inner = &plan.edges[0];
    assert_eq!(inner.convexity(), EdgeConvexity::Concave);
    assert!(
        (inner.dihedral_angle_degrees() - 270.0).abs() <= 1e-9,
        "the inner corner of an L opens 270 degrees through the material"
    );
    assert!(
        (inner.wedge_angle_degrees - 90.0).abs() <= 1e-9,
        "the profile is drawn in the VOID wedge there, which is the 90 the material leaves"
    );
    assert!((inner.setback() - 3.0).abs() <= EXACT);
    assert!((inner.center_distance - 3.0 * 2f64.sqrt()).abs() <= EXACT);

    assert_the_centre_is_tangent_to_both_faces(inner, 3.0);

    // The reflex corner of the fixture stands at (10, 10), and the two faces meeting there run
    // out along +y and +x: the tangent lines are the lines 3 from the corner on each of them.
    let mut tangent_points: Vec<Vector2> = [
        inner.left_tangent_line.point,
        inner.right_tangent_line.point,
    ]
    .iter()
    .map(|point| Vector2::new(point.x, point.y))
    .collect();
    tangent_points.sort_by(|a, b| a.x.total_cmp(&b.x));

    assert!(
        (tangent_points[0] - Vector2::new(10.0, 13.0)).length() < 1e-9,
        "3 along the x = 10 wall, at {:?}",
        tangent_points[0]
    );
    assert!(
        (tangent_points[1] - Vector2::new(13.0, 10.0)).length() < 1e-9,
        "3 along the y = 10 wall, at {:?}",
        tangent_points[1]
    );

    assert_the_tangent_line_is_inside_its_face(
        &prism,
        inner,
        &inner.left_region,
        &inner.left_tangent_line,
    );
    assert_the_tangent_line_is_inside_its_face(
        &prism,
        inner,
        &inner.right_region,
        &inner.right_tangent_line,
    );
}

/// The top rim of a cube is one closed chain of four edges that all share the top face, and the
/// four tangent lines they draw on it are the walked-back square: consecutive ones cross at the
/// mitre corners, 3 in from each of the box's own corners.
#[test]
fn the_top_rim_of_a_cube_plans_four_edges_whose_tangent_lines_mitre_at_the_walked_back_corners() {
    let cube = platonic_solids::create_cube(20.0, 20.0, 20.0);
    let (regions, features) = regions_of(&cube);
    let rim: Vec<FeatureEdge> = features
        .edges()
        .iter()
        .filter(|edge| edge.start.z > 9.9 && edge.end.z > 9.9)
        .copied()
        .collect();

    assert_eq!(
        rim.len(),
        4,
        "the top face of a box is rimmed by four edges"
    );

    let mut chains = bevel_chain_linker::link(&rim);
    assert_eq!(chains.len(), 1);
    let chain = chains.remove(0);
    assert!(chain.is_closed, "a rim closes on itself");

    let plan = SurfaceBevelPlan::try_create(&chain, &regions, 3.0, false)
        .unwrap_or_else(|reason| panic!("{reason:?}"));

    assert_eq!(plan.edges.len(), 4);
    assert!(plan.is_closed);

    let top_face = (0..cube.faces.len() as i32)
        .find(|&face| face_centre(&cube, face).z > 9.9)
        .expect("the cube has a top face");
    assert!(
        (plan.required_setback(top_face) - 3.0).abs() <= EXACT,
        "all four edges ask the top region for the same 3 mm, and the plan reports the widest"
    );

    let on_the_top: Vec<SurfaceLine> = plan
        .edges
        .iter()
        .map(|edge| tangent_line_in(edge, top_face))
        .collect();

    for i in 0..on_the_top.len() {
        let next = &on_the_top[(i + 1) % on_the_top.len()];

        let corner = on_the_top[i].try_cross(next, 1e-9).unwrap_or_else(|| {
            panic!(
                "consecutive tangent lines on one face meet at its mitre corner, {i} to {}",
                (i + 1) % on_the_top.len()
            )
        });
        assert!((corner.x.abs() - 7.0).abs() < 1e-9, "corner at {corner}");
        assert!((corner.y.abs() - 7.0).abs() < 1e-9, "corner at {corner}");
        assert!((corner.z - 10.0).abs() < 1e-9, "corner at {corner}");
    }
}

/// Which way the chain happens to walk an edge is not geometry. Plan the same cube edge both
/// ways round and the same two lines come back, with only the NAMES left and right swapped over.
#[test]
fn walking_the_same_edge_the_other_way_plans_the_same_lines_with_left_and_right_swapped() {
    let cube = platonic_solids::create_cube(20.0, 20.0, 20.0);
    let (regions, features) = regions_of(&cube);
    let chain = chain_of(&features, top_front_edge(&features));

    let backwards = BevelSweptChain::new(vec![chain.edges[0].reversed()], false);

    let forward = SurfaceBevelPlan::try_create(&chain, &regions, 3.0, true)
        .unwrap_or_else(|reason| panic!("{reason:?}"));
    let reverse = SurfaceBevelPlan::try_create(&backwards, &regions, 3.0, true)
        .unwrap_or_else(|reason| panic!("{reason:?}"));

    let there = &forward.edges[0];
    let back = &reverse.edges[0];

    assert!((back.wedge_angle_degrees - there.wedge_angle_degrees).abs() <= EXACT);
    assert!((back.setback() - there.setback()).abs() <= EXACT);
    assert!((back.center_distance - there.center_distance).abs() <= EXACT);
    assert_eq!(
        back.left_region_key(),
        there.right_region_key(),
        "the face on the left of one walk is the face on the right of the other"
    );
    assert_eq!(back.right_region_key(), there.left_region_key());

    assert_the_same_line(
        &there.left_tangent_line,
        &back.right_tangent_line,
        "the left tangent line read from the other end",
    );
    assert_the_same_line(
        &there.right_tangent_line,
        &back.left_tangent_line,
        "the right tangent line read from the other end",
    );
    assert_the_same_line(
        &there.center_line,
        &back.center_line,
        "the centre line, which has no side to swap",
    );

    // And the inward directions the walkback cuts by swap with them, rather than turning round.
    assert!(
        (there.left_inward - back.right_inward).length() < 1e-9,
        "{} against {}",
        there.left_inward,
        back.right_inward
    );
    assert!(
        (there.right_inward - back.left_inward).length() < 1e-9,
        "{} against {}",
        there.right_inward,
        back.left_inward
    );
}

/// The refusal. A triangulation diagonal across a flat face has two faces and a dihedral of 180:
/// there is no corner there to set anything back from.
#[test]
fn an_edge_between_two_coplanar_faces_is_refused() {
    let cube = platonic_solids::create_cube(20.0, 20.0, 20.0);
    let (regions, _features) = regions_of(&cube);
    let graph = cube.get_mesh_edge_graph();

    let flat = (0..graph.edge_count())
        .filter_map(|i| {
            FeatureEdge::try_classify(&cube, &graph, i, FeatureEdge::default_flat_angle_degrees())
        })
        .find(|edge| edge.convexity == EdgeConvexity::Flat && edge.direction.length_squared() > 0.0)
        .expect("a cube's faces are triangulated, so it has diagonals");

    let chain = BevelSweptChain::new(vec![ChainEdge::new(0, flat, false)], false);

    match SurfaceBevelPlan::try_create(&chain, &regions, 3.0, true) {
        Ok(_) => panic!("a face's own diagonal is not an edge of the shape"),
        Err(reason) => assert_eq!(reason.code(), BevelAdvisoryCode::FacesCoplanar),
    }
}

/// A corner's two band ends, read straight off the pair of directions that meet there. Both of
/// them name the side the band is ON, and the one at the FINISH of an edge is the one that turns
/// its answer round to do it - including where the chain doubles straight back on itself.
#[test]
fn a_corners_two_band_ends_both_point_into_the_band_they_bound() {
    let corner = Vector3::new(3.0, 4.0, 5.0);
    let arriving = Vector3::new(1.0, 0.0, 0.0);
    let here = Vector3::new(0.0, 1.0, 0.0);
    let leaving = Vector3::new(-1.0, 0.0, 0.0);

    let start =
        surface_plan_band_ends::band_end_where_the_chain_arrives_along(corner, arriving, here);
    let finish =
        surface_plan_band_ends::band_end_where_the_chain_leaves_along(corner, here, leaving);

    assert_eq!(start.point, corner);
    assert!(
        (start.inward - (arriving + here).get_normal()).length() < 1e-9,
        "the mitre plane between the two edges is the bisector, facing the way this edge lies"
    );
    assert!(
        (finish.inward + (here + leaving).get_normal()).length() < 1e-9,
        "and at the finish it is the same plane turned to face back down the edge"
    );

    // Each end keeps the stretch of edge its own band lies along, and gives up the stretch past
    // the corner.
    assert!(
        start.signed_distance(corner + here * 0.001) > 0.0,
        "the band this end bounds runs on from the corner"
    );
    assert!(
        start.signed_distance(corner - here * 0.001) < 0.0,
        "and stops at it"
    );

    assert!(
        finish.signed_distance(corner - here * 0.001) > 0.0,
        "the band this end bounds runs back from the corner"
    );
    assert!(
        finish.signed_distance(corner + here * 0.001) < 0.0,
        "and stops at it"
    );

    // And the degenerate turn: the chain doubles back, there is no bisector, and each end falls
    // back to the plane square to the edge - still facing into the band.
    let doubled_back =
        surface_plan_band_ends::band_end_where_the_chain_leaves_along(corner, here, -here);

    assert!(
        (doubled_back.inward + here).length() < 1e-9,
        "the band this end bounds lies BEHIND the corner, back along the edge"
    );

    assert_eq!(
        surface_plan_band_ends::band_end_where_the_chain_arrives_along(corner, -here, here).inward,
        here,
        "and the one at the start of a doubled back edge bounds the band ahead of it"
    );
}

/// That two lines are the same line: the same direction to a sign, and each passing through the
/// other's point.
fn assert_the_same_line(one: &SurfaceLine, other: &SurfaceLine, which: &str) {
    assert!(
        one.direction.cross(other.direction).length() < 1e-12,
        "{which} runs the same way, {} against {}",
        one.direction,
        other.direction
    );
    assert!(
        one.distance_to(other.point) < 1e-9,
        "{which} passes through {}",
        other.point
    );
    assert!(
        other.distance_to(one.point) < 1e-9,
        "{which} passes through {}",
        one.point
    );
}

/// Tangency, checked from the centre outwards: the centre line stands exactly the radius off each
/// face plane, and the point of the tangent line nearest it is that perpendicular foot.
fn assert_the_centre_is_tangent_to_both_faces(plan: &SurfaceEdgePlan, radius: f64) {
    assert_the_centre_is_tangent_to(
        plan,
        &plan.left_tangent_line,
        plan.edge.left_normal(),
        radius,
        "left",
    );
    assert_the_centre_is_tangent_to(
        plan,
        &plan.right_tangent_line,
        plan.edge.right_normal(),
        radius,
        "right",
    );
}

fn assert_the_centre_is_tangent_to(
    plan: &SurfaceEdgePlan,
    tangent_line: &SurfaceLine,
    normal: Vector3,
    radius: f64,
    side: &str,
) {
    let centre = plan.center_line.point;
    let on_the_plane = (centre - plan.from()).dot(normal);

    assert!(
        (on_the_plane.abs() - radius).abs() < EXACT,
        "the centre line is one radius off the {side} face plane, and stands {on_the_plane} off it"
    );

    let foot = tangent_line.closest_point_to(centre);
    assert!(
        ((foot - centre).length() - radius).abs() < EXACT,
        "the {side} tangent line's nearest point to the centre is a radius away"
    );
    assert!(
        (foot - centre).get_normal().cross(normal).length() < 1e-9,
        "and the way to it is square to the {side} face, which is what makes it the foot"
    );
}

/// That a tangent line really lies in the face it walks back: on the face's plane, the setback
/// from the edge, and on the side of the edge the face's own body is on.
fn assert_the_tangent_line_is_inside_its_face(
    mesh: &Mesh,
    plan: &SurfaceEdgePlan,
    region: &Arc<FaceGroup>,
    tangent_line: &SurfaceLine,
) {
    let normal = face_normal(mesh, region.faces()[0]);
    let point = tangent_line.point;

    assert!(
        (point - plan.from()).dot(normal).abs() < EXACT,
        "the tangent point {point} lies on the face's own plane"
    );

    let edge_line = SurfaceLine::new(plan.from(), plan.edge.direction());
    assert!(
        (edge_line.distance_to(point) - plan.setback()).abs() < EXACT,
        "and the setback from the edge"
    );

    let toward_the_face = region
        .faces()
        .iter()
        .fold(Vector3::ZERO, |sum, &face| sum + face_centre(mesh, face))
        / region.faces().len() as f64;
    assert!(
        (point - edge_line.closest_point_to(point)).dot(toward_the_face - plan.from()) > 0.0,
        "and on the side of the edge the face's material is on, not out in the air"
    );
}

/// The regions of a mesh, flooded at the same flat tolerance its feature edges were found at.
fn regions_of(mesh: &Mesh) -> (Arc<FaceRegionCache>, FeatureEdgeSet) {
    let features = FeatureEdgeSet::create(mesh);
    let regions = FaceRegionCache::new(
        Arc::new(mesh.clone()),
        Some(mesh.get_mesh_edge_graph()),
        features.flat_angle_degrees(),
    );

    (Arc::new(regions), features)
}

/// The one-edge chain a single feature edge links into.
fn chain_of(features: &FeatureEdgeSet, edge_index: usize) -> BevelSweptChain {
    let mut chains = bevel_chain_linker::link(&[features.edges()[edge_index]]);
    assert_eq!(chains.len(), 1);
    chains.remove(0)
}

/// The top front edge of a 20 mm cube: from (-10, -10, 10) to (10, -10, 10).
fn top_front_edge(features: &FeatureEdgeSet) -> usize {
    single(features, |edge| {
        (edge.start.y + 10.0).abs() < 1e-9
            && (edge.end.y + 10.0).abs() < 1e-9
            && edge.start.z > 9.9
            && edge.end.z > 9.9
    })
}

/// The apex edge of the isosceles prism, which runs up the z axis at the origin.
fn apex_edge(features: &FeatureEdgeSet) -> usize {
    single(features, |edge| {
        Vector2::new(edge.start.x, edge.start.y).length() < 1e-9
            && Vector2::new(edge.end.x, edge.end.y).length() < 1e-9
    })
}

/// The one feature edge index that matches; fails on none or several.
fn single(features: &FeatureEdgeSet, is_it: impl Fn(&FeatureEdge) -> bool) -> usize {
    let found: Vec<usize> = (0..features.count())
        .filter(|&i| is_it(&features.edges()[i]))
        .collect();
    assert_eq!(found.len(), 1, "exactly one feature edge matches");
    found[0]
}

/// Whichever of an edge plan's two tangent lines lies in the region `face_index` belongs to.
fn tangent_line_in(plan: &SurfaceEdgePlan, face_index: i32) -> SurfaceLine {
    if plan.left_region.faces().contains(&face_index) {
        plan.left_tangent_line
    } else {
        plan.right_tangent_line
    }
}

fn face_centre(mesh: &Mesh, face_index: i32) -> Vector3 {
    let face = &mesh.faces[face_index as usize];

    (Vector3::from_vector3_float(mesh.vertices[face.v0 as usize])
        + Vector3::from_vector3_float(mesh.vertices[face.v1 as usize])
        + Vector3::from_vector3_float(mesh.vertices[face.v2 as usize]))
        / 3.0
}

fn face_normal(mesh: &Mesh, face_index: i32) -> Vector3 {
    let face = &mesh.faces[face_index as usize];
    let position0 = Vector3::from_vector3_float(mesh.vertices[face.v0 as usize]);

    (Vector3::from_vector3_float(mesh.vertices[face.v1 as usize]) - position0)
        .cross(Vector3::from_vector3_float(mesh.vertices[face.v2 as usize]) - position0)
        .get_normal()
}
