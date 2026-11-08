//! Edge picking - turning a ray hit on a face into the feature edge the user meant, and turning
//! that edge into the candidate readings the viewport lets them cycle through.

use std::collections::HashSet;
use std::sync::Arc;

use solidmesh::topology::{
    bevel_edge_group_resolver, edge_picking, pick_interpretations, BevelSelectionKind, FeatureEdge,
    FeatureEdgePick, FeatureEdgeSet, PickInterpretation, PickInterpretationKind, TopologyError,
};
use solidmesh_math::{Vector2, Vector3, Vector3Float};
use solidmesh_mesh::solids::platonic_solids;
use solidmesh_mesh::Mesh;

use crate::topology_test_meshes;

const POSITION_TOLERANCE: f64 = 0.001;

/// A pick tolerance in mesh-local units. The viewport converts pixels into this; the core never
/// sees a pixel.
const PICK_TOLERANCE: f64 = 1.0;

const CYLINDER_SIDES: i32 = 64;

fn assert_within(actual: f64, expected: f64, tolerance: f64) {
    assert!(
        (actual - expected).abs() <= tolerance,
        "expected {expected} within {tolerance}, got {actual}"
    );
}

fn pick(
    mesh: &Mesh,
    features: &FeatureEdgeSet,
    face: i32,
    hit: Vector3,
    tolerance: f64,
) -> FeatureEdgePick {
    edge_picking::pick_feature_edge(mesh, features, face, hit, tolerance).unwrap()
}

#[test]
fn picks_the_nearest_edge_of_the_face_that_was_hit() {
    let cube = platonic_solids::create_cube(20.0, 20.0, 20.0);
    let features = FeatureEdgeSet::create(&cube);

    assert_eq!(features.count(), 12);

    // every edge of the cube, picked from both of the faces it lies on
    for (edge_index, edge) in features.edges().iter().enumerate() {
        for face_index in [edge.face0, edge.face1] {
            let hit = just_inside_the_face(&cube, face_index, (edge.start + edge.end) / 2.0);

            let pick = pick(&cube, &features, face_index, hit, PICK_TOLERANCE);

            assert!(pick.is_hit());
            assert_eq!(pick.edge_index(), edge_index as i32);
            assert!(pick.distance_to_edge() < PICK_TOLERANCE);
        }
    }
}

/// The middle of a 20 mm face is nowhere near an edge, and a pick tolerance of a millimetre says
/// so rather than reaching for the closest edge on the part.
#[test]
fn a_hit_in_the_middle_of_a_face_misses_every_edge() {
    let cube = platonic_solids::create_cube(20.0, 20.0, 20.0);
    let features = FeatureEdgeSet::create(&cube);

    for face_index in 0..cube.faces.len() as i32 {
        let pick = pick(
            &cube,
            &features,
            face_index,
            centroid(&cube, face_index),
            PICK_TOLERANCE,
        );

        assert!(!pick.is_hit());
        assert_eq!(pick.edge_index(), -1);
        assert_eq!(pick.distance_to_edge(), f64::INFINITY);
    }
}

/// A hit position or a tolerance that is not a number is a broken caller, not a miss.
#[test]
fn a_not_a_number_hit_is_refused_rather_than_answered_with_an_edge() {
    let cube = platonic_solids::create_cube(20.0, 20.0, 20.0);
    let features = FeatureEdgeSet::create(&cube);
    let on_an_edge = (features.edges()[0].start + features.edges()[0].end) / 2.0;
    let face = features.edges()[0].face0;

    let refused = |hit: Vector3, tolerance: f64| {
        matches!(
            edge_picking::pick_feature_edge(&cube, &features, face, hit, tolerance),
            Err(TopologyError::NotFinite { .. })
        )
    };

    assert!(refused(Vector3::new(f64::NAN, 0.0, 0.0), PICK_TOLERANCE));
    assert!(refused(
        Vector3::new(0.0, f64::INFINITY, 0.0),
        PICK_TOLERANCE
    ));
    assert!(refused(on_an_edge, f64::NAN));
    assert!(refused(on_an_edge, f64::INFINITY));
}

/// The nearest edge wins, not the first one the candidate walk happens to reach. Both hits are on
/// the same 50:1 sliver triangle, and moving the hit across it moves the pick between long sides.
#[test]
fn on_a_sliver_face_the_nearest_edge_wins_whichever_order_they_are_visited_in() {
    let plate = create_fan_prism(
        &[
            Vector2::new(0.0, 0.0),
            Vector2::new(20.0, 0.0),
            Vector2::new(20.0, 0.4),
            Vector2::new(0.0, 0.4),
        ],
        2.0,
    );

    let features = FeatureEdgeSet::create(&plate);
    let sliver_face = top_fan_face_index(4, 0);

    let near_the_near_side = pick(
        &plate,
        &features,
        sliver_face,
        Vector3::new(10.0, 0.1, 2.0),
        0.5,
    );
    assert_pick_runs(
        &near_the_near_side,
        Vector3::new(0.0, 0.0, 2.0),
        Vector3::new(20.0, 0.0, 2.0),
    );
    assert_within(
        near_the_near_side.distance_to_edge(),
        0.1,
        POSITION_TOLERANCE,
    );

    let near_the_far_side = pick(
        &plate,
        &features,
        sliver_face,
        Vector3::new(19.0, 0.35, 2.0),
        0.5,
    );
    assert_pick_runs(
        &near_the_far_side,
        Vector3::new(20.0, 0.4, 2.0),
        Vector3::new(0.0, 0.4, 2.0),
    );
    assert_within(
        near_the_far_side.distance_to_edge(),
        0.05,
        POSITION_TOLERANCE,
    );

    assert_ne!(
        near_the_near_side.edge_index(),
        near_the_far_side.edge_index()
    );
}

/// Distance is measured to the edge segment, not to the line it lies on.
#[test]
fn an_edge_whose_line_runs_close_but_whose_segment_does_not_is_not_picked() {
    let notched = create_notched_prism();
    let features = FeatureEdgeSet::create(&notched);
    let face_beside_the_notch = top_fan_face_index(5, 2);
    let past_the_end_of_the_near_rim_edge = Vector3::new(-2.0, 0.05, 2.0);

    let tight = pick(
        &notched,
        &features,
        face_beside_the_notch,
        past_the_end_of_the_near_rim_edge,
        0.5,
    );
    assert!(!tight.is_hit());

    // widened until something is in range, it is the edge that really is nearest - the rim edge
    // running down to the notch, 1.24 away - and not the one whose line passes at 0.05
    let loose = pick(
        &notched,
        &features,
        face_beside_the_notch,
        past_the_end_of_the_near_rim_edge,
        3.0,
    );
    assert_pick_runs(
        &loose,
        Vector3::new(-4.0, -3.0, 2.0),
        Vector3::new(0.0, 0.0, 2.0),
    );
    assert_within(loose.distance_to_edge(), 1.24, 0.01);
}

/// A triangulation diagonal is not an edge of the shape and can never be picked, however exactly
/// the ray lands on it.
#[test]
fn a_triangulation_diagonal_is_never_picked() {
    let notched = create_notched_prism();
    let features = FeatureEdgeSet::create(&notched);
    let cap_face = top_fan_face_index(5, 2);

    // the midpoint of the fan diagonal from the notch vertex (0, 0) to the far corner (-6, 6)
    let on_the_diagonal = Vector3::new(-3.0, 3.0, 2.0);

    let tight = pick(&notched, &features, cap_face, on_the_diagonal, 0.5);
    assert!(!tight.is_hit());

    let loose = pick(&notched, &features, cap_face, on_the_diagonal, 3.0);
    assert_pick_runs(
        &loose,
        Vector3::new(-6.0, 6.0, 2.0),
        Vector3::new(-4.0, -3.0, 2.0),
    );
    assert_within(loose.distance_to_edge(), 2.278, 0.01);
}

/// A cube edge is one edge at every tolerance a bevel would use; what is left to cycle through is
/// the two FACES it divides - and every reading has to store as what it showed.
#[test]
fn a_cube_edge_offers_itself_and_the_rim_of_each_face_it_divides() {
    let cube = Arc::new(platonic_solids::create_cube(20.0, 20.0, 20.0));
    let features = Arc::new(FeatureEdgeSet::create(&cube));

    for seed in 0..features.count() {
        let interpretations =
            pick_interpretations::build_default(&cube, &features, seed as i32).unwrap();

        assert_eq!(interpretations.len(), 3);
        assert_eq!(
            interpretations[0].kind(),
            PickInterpretationKind::CollinearRun
        );
        assert_eq!(interpretations[0].count(), 1);
        assert_eq!(
            interpretations[0].edges()[0].mesh_edge_index,
            features.edges()[seed].mesh_edge_index
        );
        assert_eq!(
            interpretations[0].group().selection_kind,
            BevelSelectionKind::Edge
        );
        assert_eq!(
            interpretations[0].group().flat_angle_degrees,
            FeatureEdge::default_flat_angle_degrees()
        );

        for interpretation in &interpretations[1..3] {
            assert_eq!(interpretation.kind(), PickInterpretationKind::FaceGroup);
            assert_eq!(
                interpretation.count(),
                4,
                "a cube face is bounded by four edges"
            );
            assert_eq!(
                interpretation.group().selection_kind,
                BevelSelectionKind::FaceGroup
            );
        }

        for interpretation in &interpretations {
            assert_group_resolves_to_the_same_edges(&cube, interpretation);
        }
    }
}

/// A tessellated rim is one edge at the tolerance that reads every wall facet as an edge, and
/// the whole 64 edge circle at the tolerance that reads the wall as one surface; the third
/// reading is the wall FACET the click landed on.
#[test]
fn a_cylinder_rim_edge_offers_the_edge_the_whole_rim_and_the_wall_facet() {
    let cylinder = Arc::new(topology_test_meshes::create_cylinder(
        10.0,
        10.0,
        CYLINDER_SIDES,
    ));
    let features = Arc::new(FeatureEdgeSet::create(&cylinder));
    let seed = index_of_rim_edge(&features, 5.0);

    let interpretations = pick_interpretations::build_default(&cylinder, &features, seed).unwrap();

    assert_eq!(interpretations.len(), 3);

    assert_eq!(
        interpretations[0].kind(),
        PickInterpretationKind::CollinearRun
    );
    assert_eq!(interpretations[0].count(), 1);
    assert_eq!(
        interpretations[0].flat_angle_degrees(),
        FeatureEdge::default_flat_angle_degrees()
    );
    assert!(interpretations[0].chain().is_none());

    assert_eq!(
        interpretations[1].kind(),
        PickInterpretationKind::TangentChain
    );
    assert_eq!(interpretations[1].count(), CYLINDER_SIDES as usize);
    assert_eq!(
        interpretations[1].flat_angle_degrees(),
        pick_interpretations::SMOOTH_WALL_FLAT_ANGLE_DEGREES
    );
    assert!(interpretations[1].chain().unwrap().is_closed());
    assert_eq!(
        interpretations[1].group().selection_kind,
        BevelSelectionKind::TangentChain
    );
    assert_eq!(
        interpretations[1].group().flat_angle_degrees,
        pick_interpretations::SMOOTH_WALL_FLAT_ANGLE_DEGREES
    );

    // the rim it was picked on, not the one at the other end
    for edge in interpretations[1].edges() {
        assert_within(edge.start.z, 5.0, POSITION_TOLERANCE);
        assert_within(edge.end.z, 5.0, POSITION_TOLERANCE);
    }

    assert_eq!(interpretations[2].kind(), PickInterpretationKind::FaceGroup);
    assert_eq!(
        interpretations[2].count(),
        4,
        "at the pick's own tolerance the wall is facets, so the face under the cursor is one quad"
    );
    assert_eq!(
        interpretations[2].group().selection_kind,
        BevelSelectionKind::FaceGroup
    );

    // what the UI stores has to come back as what the user saw
    for interpretation in &interpretations {
        assert_group_resolves_to_the_same_edges(&cylinder, interpretation);
    }
}

/// The chooser cycles through the readings, so the same click has to produce the same readings in
/// the same order every time.
#[test]
fn the_ladder_is_deterministic() {
    let cylinder = Arc::new(topology_test_meshes::create_cylinder(
        10.0,
        10.0,
        CYLINDER_SIDES,
    ));
    let features = Arc::new(FeatureEdgeSet::create(&cylinder));
    let seed = index_of_rim_edge(&features, 5.0);

    let first = pick_interpretations::build_default(&cylinder, &features, seed).unwrap();
    let second = pick_interpretations::build_default(&cylinder, &features, seed).unwrap();

    assert_eq!(second.len(), first.len());

    for (a, b) in first.iter().zip(&second) {
        assert_eq!(b.kind(), a.kind());
        assert_eq!(b.flat_angle_degrees(), a.flat_angle_degrees());
        assert_eq!(b.curve_tolerance_degrees(), a.curve_tolerance_degrees());
        assert_eq!(b.count(), a.count());

        for (ea, eb) in a.edges().iter().zip(b.edges()) {
            assert_eq!(eb.mesh_edge_index, ea.mesh_edge_index);
        }
    }
}

/// Storing a reading and resolving it again has to give back the edges the user was looking at
/// when they chose it.
fn assert_group_resolves_to_the_same_edges(mesh: &Mesh, interpretation: &PickInterpretation) {
    let resolved = bevel_edge_group_resolver::resolve(interpretation.group(), mesh);

    assert!(resolved.is_resolved());
    assert_eq!(resolved.edges.len(), interpretation.count());

    let expected: HashSet<i32> = interpretation
        .edges()
        .iter()
        .map(|edge| edge.mesh_edge_index)
        .collect();

    for edge in &resolved.edges {
        assert!(expected.contains(&edge.mesh_edge_index));
    }
}

fn assert_pick_runs(pick: &FeatureEdgePick, start: Vector3, end: Vector3) {
    assert!(pick.is_hit());

    let edge = pick.edge();
    let runs_one_way = matches(edge.start, start) && matches(edge.end, end);
    let runs_the_other = matches(edge.start, end) && matches(edge.end, start);

    assert!(
        runs_one_way || runs_the_other,
        "expected the edge from {start} to {end}, picked the one from {} to {}",
        edge.start,
        edge.end
    );
}

fn matches(a: Vector3, b: Vector3) -> bool {
    (a - b).length() < POSITION_TOLERANCE
}

/// A point on the face, a twentieth of the way from a point on its boundary toward its middle -
/// where a click aimed at an edge actually lands.
fn just_inside_the_face(mesh: &Mesh, face_index: i32, on_the_boundary: Vector3) -> Vector3 {
    on_the_boundary + ((centroid(mesh, face_index) - on_the_boundary) * 0.05)
}

fn centroid(mesh: &Mesh, face_index: i32) -> Vector3 {
    let face = mesh.faces[face_index as usize];

    (mesh.vertices[face.v0 as usize].as_vector3()
        + mesh.vertices[face.v1 as usize].as_vector3()
        + mesh.vertices[face.v2 as usize].as_vector3())
        / 3.0
}

fn index_of_rim_edge(features: &FeatureEdgeSet, rim_height: f64) -> i32 {
    features
        .edges()
        .iter()
        .position(|edge| {
            (edge.start.z - rim_height).abs() < POSITION_TOLERANCE
                && (edge.end.z - rim_height).abs() < POSITION_TOLERANCE
        })
        .unwrap_or_else(|| panic!("No feature edge lies in the rim at z = {rim_height}."))
        as i32
}

/// A prism with a notch: five sided, wound counter clockwise, with a reflex corner at the origin
/// whose two rim edges run away from it.
fn create_notched_prism() -> Mesh {
    create_fan_prism(
        &[
            Vector2::new(0.0, 0.0),
            Vector2::new(4.0, 0.0),
            Vector2::new(4.0, 6.0),
            Vector2::new(-6.0, 6.0),
            Vector2::new(-4.0, -3.0),
        ],
        2.0,
    )
}

/// Builds a closed prism from a counter clockwise polygon: a cap at z = 0, another at `height`,
/// and a two triangle wall for each polygon edge. Both caps are fan triangulated from the
/// polygon's first vertex, which fixes which triangle holds which hit point and makes the fan
/// spokes triangulation diagonals.
fn create_fan_prism(polygon_counter_clockwise: &[Vector2], height: f64) -> Mesh {
    let mut mesh = Mesh::new();
    let sides = polygon_counter_clockwise.len() as i32;

    for corner in polygon_counter_clockwise {
        mesh.vertices
            .push(Vector3Float::new(corner.x, corner.y, 0.0));
    }

    for corner in polygon_counter_clockwise {
        mesh.vertices
            .push(Vector3Float::new(corner.x, corner.y, height));
    }

    // bottom cap first, then top cap, then the walls - the order top_fan_face_index reads
    for i in 1..sides - 1 {
        mesh.faces.add(0, i + 1, i, &mesh.vertices);
    }

    for i in 1..sides - 1 {
        mesh.faces
            .add(sides, sides + i, sides + i + 1, &mesh.vertices);
    }

    for i in 0..sides {
        let next = (i + 1) % sides;

        mesh.faces.add(i, next, sides + next, &mesh.vertices);
        mesh.faces.add(i, sides + next, sides + i, &mesh.vertices);
    }

    mesh
}

/// Where `create_fan_prism` puts one triangle of the top cap's fan: the bottom cap's `sides - 2`
/// triangles come first.
fn top_fan_face_index(sides: i32, fan_triangle: i32) -> i32 {
    (sides - 2) + fan_triangle
}
