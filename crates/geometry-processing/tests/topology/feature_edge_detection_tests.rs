//! Feature-edge detection and dihedral classification, the topology foundation the fillet and
//! chamfer operations stand on.

use solidmesh::topology::{EdgeConvexity, FeatureEdge, FeatureEdgeSet};
use solidmesh_mesh::solids::platonic_solids;
use solidmesh_mesh::Mesh;

use crate::topology_test_meshes;

const ANGLE_TOLERANCE: f64 = 0.01;

fn assert_within(actual: f64, expected: f64, tolerance: f64) {
    assert!(
        (actual - expected).abs() <= tolerance,
        "expected {expected} within {tolerance}, got {actual}"
    );
}

fn cube() -> Mesh {
    platonic_solids::create_cube(20.0, 20.0, 20.0)
}

#[test]
fn cube_has_twelve_convex_feature_edges() {
    let cube = cube();

    // 12 edges of the shape plus one triangulation diagonal across each of the 6 faces
    assert_eq!(cube.get_mesh_edge_graph().edge_count(), 18);

    let features = FeatureEdgeSet::create(&cube);

    assert_eq!(features.count(), 12);
    assert!(features
        .edges()
        .iter()
        .all(|edge| edge.convexity == EdgeConvexity::Convex));

    for edge in features.edges() {
        assert_within(edge.turn_angle_degrees, 90.0, ANGLE_TOLERANCE);
        assert_within(edge.dihedral_angle_degrees(), 90.0, ANGLE_TOLERANCE);
    }
}

/// The classifier answers for any manifold edge, not just the sharp ones - the flood fill that
/// walks across flat-ish edges needs the flat answer as much as chains need the sharp one.
#[test]
fn classifier_reports_cube_triangulation_diagonals_as_flat() {
    let cube = cube();
    let graph = cube.get_mesh_edge_graph();

    let mut flat_count = 0;
    for edge_index in 0..graph.edge_count() {
        let edge = FeatureEdge::try_classify(
            &cube,
            &graph,
            edge_index,
            FeatureEdge::default_flat_angle_degrees(),
        )
        .expect("every cube edge is manifold");

        if edge.convexity == EdgeConvexity::Flat {
            flat_count += 1;
            assert_within(edge.turn_angle_degrees, 0.0, ANGLE_TOLERANCE);
        }
    }

    assert_eq!(flat_count, 6);
}

/// Face0 is always the lower of the two face indices, because MeshEdgeGraph hands an edge's faces
/// over in ascending order. A saved BevelEdgeGroup anchors AnchorNormal0 to Face0, so this pins it.
#[test]
fn face0_is_always_the_lower_face_index() {
    for mesh in [cube(), topology_test_meshes::create_l_shaped_prism()] {
        let features = FeatureEdgeSet::create(&mesh);

        assert!(features.count() > 0);
        for edge in features.edges() {
            assert!(edge.face0 < edge.face1);
        }
    }
}

#[test]
fn l_shape_inner_edge_is_concave_and_the_rest_are_convex() {
    let l_shape = topology_test_meshes::create_l_shaped_prism();

    let features = FeatureEdgeSet::create(&l_shape);

    let concave: Vec<&FeatureEdge> = features
        .edges()
        .iter()
        .filter(|edge| edge.convexity == EdgeConvexity::Concave)
        .collect();
    assert_eq!(concave.len(), 1);
    assert_eq!(
        features
            .edges()
            .iter()
            .filter(|edge| edge.convexity == EdgeConvexity::Convex)
            .count(),
        features.count() - 1
    );

    // the reflex corner of the L, running the full height of the prism
    let inner_edge = concave[0];
    assert_within(inner_edge.direction.z.abs(), 1.0, ANGLE_TOLERANCE);
    let mid_point = (inner_edge.start + inner_edge.end) / 2.0;
    assert_within(mid_point.x, 10.0, 0.01);
    assert_within(mid_point.y, 10.0, 0.01);
    assert_within(inner_edge.turn_angle_degrees, -90.0, ANGLE_TOLERANCE);
    assert_within(inner_edge.dihedral_angle_degrees(), 270.0, ANGLE_TOLERANCE);

    // six walls meeting at six vertical edges, and each wall meeting the two caps
    assert_eq!(
        features
            .edges()
            .iter()
            .filter(|edge| edge.direction.z.abs() > 0.999)
            .count(),
        6
    );
    assert_eq!(features.count(), 18);
}

/// A bevel needs two faces to cut between, so the rim of an open mesh is not bevelable however
/// sharply it reads on screen.
#[test]
fn open_rim_edges_are_not_feature_edges() {
    let mut open_box = cube();

    // the first two faces are the +Z face of the cube; without them its four rim edges have one
    // adjacent face each
    open_box.faces.drain(0..2);

    let features = FeatureEdgeSet::create(&open_box);

    // the 4 bottom edges and the 4 vertical ones survive; the 4 top ones are boundary now
    assert_eq!(features.count(), 8);

    let on_open_rim = features
        .edges()
        .iter()
        .filter(|edge| edge.start.z > 9.99 && edge.end.z > 9.99)
        .count();
    assert_eq!(on_open_rim, 0);
}

/// Pins that the flat tolerance is a real parameter: a 64 sided cylinder turns 5.625 degrees at
/// each wall facet - sharp at the default half-degree tolerance, smooth at 10 degrees.
#[test]
fn cylinder_wall_facets_are_feature_edges_only_at_a_tight_tolerance() {
    const SIDES: i32 = 64;
    let cylinder = topology_test_meshes::create_cylinder(10.0, 10.0, SIDES);

    let at_default_tolerance = FeatureEdgeSet::create(&cylinder);
    let at_ten_degrees = FeatureEdgeSet::create_with_angle(&cylinder, 10.0);

    let wall_facet_edges: Vec<&FeatureEdge> = at_default_tolerance
        .edges()
        .iter()
        .filter(|edge| edge.direction.z.abs() > 0.999)
        .collect();
    assert_eq!(wall_facet_edges.len(), SIDES as usize);
    for edge in wall_facet_edges {
        assert_within(
            edge.turn_angle_degrees,
            360.0 / SIDES as f64,
            ANGLE_TOLERANCE,
        );
    }

    // the two cap rims are all that is left of a cylinder once its facets read as one surface
    assert_eq!(
        at_ten_degrees
            .edges()
            .iter()
            .filter(|edge| edge.direction.z.abs() > 0.999)
            .count(),
        0
    );
    assert_eq!(
        at_ten_degrees.count(),
        at_default_tolerance.count() - SIDES as usize
    );
    assert_eq!(at_ten_degrees.count(), SIDES as usize * 2);
    for edge in at_ten_degrees.edges() {
        assert_within(edge.turn_angle_degrees, 90.0, ANGLE_TOLERANCE);
    }
}

/// Holds the documented sign convention: with the edge oriented the way Face0 winds it,
/// cross(normal0, normal1) . Direction is positive on a convex edge and negative on a concave one.
#[test]
fn convexity_sign_follows_face0_winding() {
    let cube = cube();
    for edge in FeatureEdgeSet::create(&cube).edges() {
        assert!(signed_turn(&cube, edge) > 0.0);
        assert_eq!(edge.convexity, EdgeConvexity::Convex);
    }

    let l_shape = topology_test_meshes::create_l_shaped_prism();
    let l_features = FeatureEdgeSet::create(&l_shape);
    let concave: Vec<&FeatureEdge> = l_features
        .edges()
        .iter()
        .filter(|edge| edge.convexity == EdgeConvexity::Concave)
        .collect();
    assert_eq!(concave.len(), 1);
    let inner_edge = concave[0];
    assert!(signed_turn(&l_shape, inner_edge) < 0.0);

    // Face0 really is the face that winds the edge Start to End - the convention rests on it
    let face0 = l_shape.faces[inner_edge.face0 as usize];
    let wound = [face0.v0, face0.v1, face0.v2];
    let start_at = wound
        .iter()
        .position(|&vertex| vertex == inner_edge.start_vertex)
        .expect("Face0 carries the edge's start vertex");
    assert_eq!(wound[(start_at + 1) % 3], inner_edge.end_vertex);
}

/// A mesh turned inside out reads every convex edge as concave, which is the honest answer.
#[test]
fn reversed_winding_flips_convexity() {
    let mut inside_out_cube = cube();
    inside_out_cube.reverse_faces();

    let features = FeatureEdgeSet::create(&inside_out_cube);

    assert_eq!(features.count(), 12);
    assert!(features
        .edges()
        .iter()
        .all(|edge| edge.convexity == EdgeConvexity::Concave));
}

/// Chain propagation walks vertex to vertex, and stops where three or more feature edges meet.
#[test]
fn every_cube_corner_carries_three_feature_edges() {
    let cube = cube();

    let features = FeatureEdgeSet::create(&cube);

    for vertex_index in 0..cube.vertices.len() as i32 {
        assert_eq!(features.edges_at_vertex(vertex_index).len(), 3);
    }

    // a vertex no feature edge reaches answers empty rather than throwing
    assert_eq!(
        features
            .edges_at_vertex(cube.vertices.len() as i32 + 1)
            .len(),
        0
    );
}

/// The documented convexity expression, written out so the test pins the convention rather than
/// whatever the implementation happens to compute.
fn signed_turn(mesh: &Mesh, edge: &FeatureEdge) -> f64 {
    let normal0 = mesh.faces[edge.face0 as usize].normal.as_vector3();
    let normal1 = mesh.faces[edge.face1 as usize].normal.as_vector3();

    normal0.cross(normal1).dot(edge.direction)
}
