//! Ports `MatterCADTests/Standard/FaceGroupTests.cs`: face-group flood fill - the "logical
//! surface" a click on a face selects, and the feature edges that bound it, which is what a bevel
//! is actually applied to.

use geometry_processing::topology::{EdgeConvexity, FaceGroup, FeatureEdge, FeatureEdgeSet};
use polygon_mesh::solids::platonic_solids;
use polygon_mesh::Mesh;

use crate::topology_test_meshes;

const CYLINDER_SIDES: i32 = 64;

fn assert_within(actual: f64, expected: f64, tolerance: f64) {
    assert!(
        (actual - expected).abs() <= tolerance,
        "expected {expected} within {tolerance}, got {actual}"
    );
}

fn cylinder() -> Mesh {
    topology_test_meshes::create_cylinder(10.0, 10.0, CYLINDER_SIDES)
}

/// Selecting the top of a box means selecting its four rim edges - the plan's worked example.
#[test]
fn cube_face_group_is_two_triangles_bounded_by_four_rim_edges() {
    let cube = platonic_solids::create_cube(20.0, 20.0, 20.0);

    // the first two faces of a cube are the two triangles of one side, here the +Z one
    let group = FaceGroup::create(&cube, 0).unwrap();

    assert_eq!(group.faces().len(), 2);
    assert_eq!(group.faces()[0], 0);
    assert_eq!(group.faces()[1], 1);

    assert_eq!(group.boundary_edges().len(), 4);
    assert_eq!(group.open_boundary_mesh_edges().len(), 0);
    for edge in group.boundary_edges() {
        assert_eq!(edge.convexity, EdgeConvexity::Convex);
        assert_within(edge.start.z, 10.0, 0.001);
        assert_within(edge.end.z, 10.0, 0.001);
    }
}

/// Pins the curve tolerance both ways: at 30 degrees a tessellated wall is one surface bounded by
/// its two rims, at 2 degrees each 5.625 degree facet is a surface of its own.
#[test]
fn cylinder_wall_is_one_surface_at_thirty_degrees_and_one_facet_at_two() {
    let cylinder = cylinder();
    let wall_face = first_wall_face(&cylinder);

    let whole_wall = FaceGroup::create(&cylinder, wall_face).unwrap();

    // two triangles per facet, and a rim edge at the top and bottom of each
    assert_eq!(whole_wall.faces().len(), CYLINDER_SIDES as usize * 2);
    assert_eq!(
        whole_wall.boundary_edges().len(),
        CYLINDER_SIDES as usize * 2
    );
    assert!(whole_wall
        .boundary_edges()
        .iter()
        .all(|edge| edge.convexity == EdgeConvexity::Convex));
    for edge in whole_wall.boundary_edges() {
        assert_within(edge.turn_angle_degrees, 90.0, 0.01);
    }

    let one_facet = FaceGroup::create_with_tolerance(&cylinder, wall_face, 2.0).unwrap();

    assert_eq!(one_facet.faces().len(), 2);

    // the two facet edges either side, plus the piece of each rim
    assert_eq!(one_facet.boundary_edges().len(), 4);
    assert_eq!(
        one_facet
            .boundary_edges()
            .iter()
            .filter(|edge| edge.direction.z.abs() > 0.999)
            .count(),
        2
    );
}

/// Smoothness is a magnitude, not a direction: turning a cylinder inside out makes every facet
/// edge concave and leaves the group exactly as it was.
#[test]
fn gentle_concave_folds_are_crossed_just_like_convex_ones() {
    let mut inside_out = cylinder();
    inside_out.reverse_faces();

    let group = FaceGroup::create(&inside_out, first_wall_face(&inside_out)).unwrap();

    assert_eq!(group.faces().len(), CYLINDER_SIDES as usize * 2);
    assert_eq!(group.boundary_edges().len(), CYLINDER_SIDES as usize * 2);
    assert!(group
        .boundary_edges()
        .iter()
        .all(|edge| edge.convexity == EdgeConvexity::Concave));
}

/// The fill stops at the inner corner of an L, and hands that concave edge back as part of the
/// group's boundary.
#[test]
fn l_shape_group_stops_at_the_concave_edge() {
    let l_shape = topology_test_meshes::create_l_shaped_prism();
    let features = FeatureEdgeSet::create(&l_shape);
    let concave_edge = single_concave(features.edges());

    let group = FaceGroup::create(&l_shape, concave_edge.face0).unwrap();

    // one wall of the L, two triangles, bounded by its two vertical edges and the two cap rims
    assert_eq!(group.faces().len(), 2);
    assert!(group.faces().contains(&concave_edge.face0));
    assert!(!group.faces().contains(&concave_edge.face1));

    assert_eq!(group.boundary_edges().len(), 4);
    let boundary_concave = single_concave(group.boundary_edges());
    assert_eq!(
        boundary_concave.mesh_edge_index,
        concave_edge.mesh_edge_index
    );
}

/// An open rim bounds the group but is not bevelable, so it is reported apart from the feature
/// edges rather than mixed in with them.
#[test]
fn open_rim_is_reported_apart_from_the_bevelable_boundary() {
    let mut open_box = platonic_solids::create_cube(20.0, 20.0, 20.0);

    // drop the +Z side, so the sides around it now end at an open rim
    open_box.faces.drain(0..2);

    let group = FaceGroup::create(&open_box, 0).unwrap();

    assert_eq!(group.faces().len(), 2);

    assert_eq!(group.open_boundary_mesh_edges().len(), 1);
    assert_eq!(group.boundary_edges().len(), 3);
    assert!(group
        .boundary_edges()
        .iter()
        .all(|edge| edge.convexity == EdgeConvexity::Convex));
}

/// `LabelSurfaces` is the same flood run from every face at once, so it has to hand back exactly
/// what `Create` does from any seed.
#[test]
fn labelling_every_face_agrees_with_the_group_grown_from_a_seed() {
    let cylinder = cylinder();
    let wall_face = first_wall_face(&cylinder);

    let surface_of_face = FaceGroup::label_surfaces(
        &cylinder,
        &cylinder.get_mesh_edge_graph(),
        FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES,
    );

    assert_eq!(
        surface_of_face.len(),
        cylinder.faces.len(),
        "every face is labelled, including any the fill would never reach from this seed"
    );

    let wall = FaceGroup::create(&cylinder, wall_face).unwrap();
    let mut labelled: Vec<i32> = (0..surface_of_face.len() as i32)
        .filter(|&face| surface_of_face[face as usize] == surface_of_face[wall_face as usize])
        .collect();
    let mut wall_faces = wall.faces().to_vec();
    labelled.sort_unstable();
    wall_faces.sort_unstable();

    assert_eq!(
        labelled, wall_faces,
        "the component holding the seed IS the group grown from it"
    );

    // Dense from zero, which is what makes an id usable as an index and comparable by equality.
    let mut distinct = surface_of_face.clone();
    distinct.sort_unstable();
    distinct.dedup();

    assert_eq!(*distinct.first().unwrap(), 0);
    assert_eq!(*distinct.last().unwrap(), distinct.len() as i32 - 1);

    assert_eq!(
        distinct.len(),
        3,
        "a cylinder is a wall and two caps, and nothing else at this tolerance"
    );
}

fn single_concave(edges: &[FeatureEdge]) -> FeatureEdge {
    let concave: Vec<&FeatureEdge> = edges
        .iter()
        .filter(|edge| edge.convexity == EdgeConvexity::Concave)
        .collect();
    assert_eq!(concave.len(), 1, "expected exactly one concave edge");
    *concave[0]
}

/// The first face of the revolved wall, found by its normal being level rather than by index.
fn first_wall_face(mesh: &Mesh) -> i32 {
    mesh.faces
        .iter()
        .position(|face| face.normal.z.abs() < 0.001)
        .expect("no wall face found") as i32
}
