//! Ports `BevelGroupResolutionAdvisoryTests`: a group that fails to resolve says why as a
//! `BevelAdvisory` (a code and the values it quotes), never as a sentence. Every distinct refusal
//! of both resolvers - the mesh-edge resolver (`bevel_edge_group_resolver`, on the standard cube,
//! a pair of cubes and an open tube) and the path-corner resolver (`bevel_corner_group_resolver`,
//! on a square and a comb) - is driven here and pinned to its code and argument list.

use agg_path::{VertexSource, VertexStorage};
use geometry_processing::bevel::{BevelAdvisory, BevelAdvisoryArg, BevelAdvisoryCode};
use geometry_processing::topology::bevel_corner_group_resolver::{
    create_from_corner, resolve, DEFAULT_DIRECTION_TOLERANCE_DEGREES,
    DEFAULT_DRIFT_FRACTION_OF_EXTENT,
};
use geometry_processing::topology::bevel_edge_group_resolver::{
    self, DEFAULT_DIRECTION_TOLERANCE_DEGREES as EDGE_DIRECTION_TOLERANCE_DEGREES,
    DEFAULT_DRIFT_FRACTION_OF_FEATURE_EXTENT, DEFAULT_NORMAL_TOLERANCE_DEGREES,
};
use geometry_processing::topology::{
    BevelCornerGroup, BevelEdgeGroup, BevelSelectionKind, FaceGroup, FeatureEdgeSet, PathCornerSet,
};
use polygon_mesh::solids::platonic_solids::create_cube;
use polygon_mesh::Mesh;
use vector_math::{math_helper, AxisAlignedBoundingBox, Vector2, Vector3, Vector3Float};

const BEND_THRESHOLD: f64 = 10.0;

fn square(side: f64) -> Vec<Vector2> {
    vec![
        Vector2::new(0.0, 0.0),
        Vector2::new(side, 0.0),
        Vector2::new(side, side),
        Vector2::new(0.0, side),
    ]
}

/// A sawtooth of identical teeth, the 2D reading of the two-cube fixture.
fn comb(teeth: i32) -> Vec<Vector2> {
    const PITCH: f64 = 6.0;
    let mut points = Vec::new();
    for i in 0..teeth {
        points.push(Vector2::new(i as f64 * PITCH, 5.0));
        points.push(Vector2::new((i as f64 * PITCH) + (PITCH / 2.0), 0.0));
    }
    // back along the bottom, far enough below the teeth that nothing down here is near one
    points.push(Vector2::new(
        ((teeth - 1) as f64 * PITCH) + (PITCH / 2.0),
        -10.0,
    ));
    points.push(Vector2::new(0.0, -10.0));
    points
}

fn closed_path(points: &[Vector2]) -> impl VertexSource {
    let mut storage = VertexStorage::new();
    storage.move_to_vector(points[0]);
    for point in &points[1..] {
        storage.line_to_vector(*point);
    }
    storage.close_polygon();
    storage
}

fn index_of_corner_at(corners: &PathCornerSet, position: Vector2) -> i32 {
    corners
        .corners
        .iter()
        .position(|corner| (corner.position - position).length() < 1e-9)
        .map_or(-1, |i| i as i32)
}

fn double_arg(arg: &BevelAdvisoryArg) -> f64 {
    match arg {
        BevelAdvisoryArg::Double(value) => *value,
        other => panic!("expected a double argument, got {other:?}"),
    }
}

#[test]
fn a_path_with_no_corners_fails_naming_the_bend_threshold() {
    const SHARPER_THAN_A_SQUARE: f64 = 100.0;
    let square = closed_path(&square(20.0));
    let group = create_from_corner(&PathCornerSet::create(&square, BEND_THRESHOLD), 0).unwrap();
    let no_corners = PathCornerSet::create(&square, SHARPER_THAN_A_SQUARE);
    assert_eq!(no_corners.count(), 0);

    let resolution = resolve(&group, &no_corners);
    assert!(!resolution.is_resolved());
    assert_eq!(
        resolution.failure_reason,
        Some(BevelAdvisory::new(
            BevelAdvisoryCode::PathHasNoCorners,
            vec![SHARPER_THAN_A_SQUARE.into()]
        ))
    );
}

#[test]
fn a_corner_group_with_no_anchor_directions_fails_saying_so() {
    let corners = PathCornerSet::create(&closed_path(&square(20.0)), BEND_THRESHOLD);
    let resolution = resolve(&BevelCornerGroup::default(), &corners);
    assert!(!resolution.is_resolved());
    assert_eq!(
        resolution.failure_reason,
        Some(BevelAdvisory::new(
            BevelAdvisoryCode::GroupHasNoAnchorDirections,
            Vec::new()
        ))
    );
}

#[test]
fn a_corner_nothing_matches_fails_quoting_the_gates_it_was_judged_by() {
    let square = square(20.0);
    let corners = PathCornerSet::create(&closed_path(&square), BEND_THRESHOLD);
    let group = create_from_corner(&corners, index_of_corner_at(&corners, Vector2::ZERO)).unwrap();

    let triangle: Vec<Vector2> = square[1..].to_vec();
    let remaining = PathCornerSet::create(&closed_path(&triangle), BEND_THRESHOLD);
    let resolution = resolve(&group, &remaining);
    assert!(!resolution.is_resolved());

    let reason = resolution.failure_reason.unwrap();
    assert_eq!(reason.code(), BevelAdvisoryCode::NoCornerMatchesAnchor);
    assert_eq!(reason.args().len(), 4);
    assert_eq!(
        reason.args()[0],
        BevelAdvisoryArg::Vector2(group.anchor_position)
    );
    assert_eq!(
        reason.args()[1],
        BevelAdvisoryArg::Int(remaining.count() as i32)
    );
    let expected_drift = remaining.extent * DEFAULT_DRIFT_FRACTION_OF_EXTENT;
    assert!((double_arg(&reason.args()[2]) - expected_drift).abs() <= 1e-9);
    assert_eq!(
        reason.args()[3],
        BevelAdvisoryArg::Double(DEFAULT_DIRECTION_TOLERANCE_DEGREES)
    );
}

#[test]
fn a_corner_two_corners_fit_equally_well_fails_quoting_both_scores() {
    let comb = comb(5);
    let corners = PathCornerSet::create(&closed_path(&comb), BEND_THRESHOLD);
    let middle_tooth = Vector2::new(12.0, 5.0);
    let group = create_from_corner(&corners, index_of_corner_at(&corners, middle_tooth)).unwrap();

    // the tooth it was made on filed off, leaving its two identical neighbours either side
    let filed_off: Vec<Vector2> = comb
        .iter()
        .copied()
        .filter(|point| (*point - middle_tooth).length() > 1e-9)
        .collect();
    let resolution = resolve(
        &group,
        &PathCornerSet::create(&closed_path(&filed_off), BEND_THRESHOLD),
    );
    assert!(!resolution.is_resolved());

    let reason = resolution.failure_reason.unwrap();
    assert_eq!(
        reason.code(),
        BevelAdvisoryCode::AnchorMatchesMoreThanOneCorner
    );
    assert_eq!(reason.args().len(), 3);
    assert_eq!(
        reason.args()[0],
        BevelAdvisoryArg::Vector2(group.anchor_position)
    );
    assert!((double_arg(&reason.args()[1]) - double_arg(&reason.args()[2])).abs() <= 1e-9);
}

const CUBE_SIZE: f64 = 20.0;

const TUBE_SIDES: i32 = 12;

fn top_front_midpoint() -> Vector3 {
    Vector3::new(0.0, -CUBE_SIZE / 2.0, CUBE_SIZE / 2.0)
}

#[test]
fn a_mesh_with_no_feature_edges_fails_naming_the_flat_tolerance() {
    let group = top_front_edge_group(
        BevelSelectionKind::Edge,
        FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES,
    );

    // one lone triangle: three open edges, no two faces meeting anywhere, so no feature edge at
    // any tolerance
    let mut triangle = Mesh::new();
    triangle.create_face(&[
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(10.0, 0.0, 0.0),
        Vector3::new(0.0, 10.0, 0.0),
    ]);

    let resolution = bevel_edge_group_resolver::resolve(&group, &triangle);

    assert!(!resolution.is_resolved());
    assert_eq!(
        resolution.failure_reason,
        Some(BevelAdvisory::new(
            BevelAdvisoryCode::NoFeatureEdgesAtTolerance,
            vec![group.flat_angle_degrees.into()]
        ))
    );
}

#[test]
fn a_group_with_no_anchor_direction_fails_saying_so() {
    let cube = create_cube(CUBE_SIZE, CUBE_SIZE, CUBE_SIZE);

    // what a corrupt file leaves behind: the anchor midpoint present, the direction at its default
    let group = BevelEdgeGroup {
        selection_kind: BevelSelectionKind::Edge,
        anchor_midpoint: top_front_midpoint(),
        ..BevelEdgeGroup::default()
    };

    let resolution = bevel_edge_group_resolver::resolve(&group, &cube);

    assert!(!resolution.is_resolved());
    assert_eq!(
        resolution.failure_reason,
        Some(BevelAdvisory::new(
            BevelAdvisoryCode::GroupHasNoAnchorDirection,
            vec![]
        ))
    );
}

#[test]
fn an_anchor_nothing_matches_fails_quoting_the_gates_it_was_judged_by() {
    let group = top_front_edge_group(
        BevelSelectionKind::Edge,
        FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES,
    );

    // ten times the size: the same edge is 127 mm from where the anchor remembers it
    let huge = create_cube(CUBE_SIZE * 10.0, CUBE_SIZE * 10.0, CUBE_SIZE * 10.0);

    let resolution = bevel_edge_group_resolver::resolve(&group, &huge);

    assert!(!resolution.is_resolved());

    let reason = resolution.failure_reason.clone().expect("a refusal");

    assert_eq!(reason.code(), BevelAdvisoryCode::NoFeatureEdgeMatchesAnchor);
    assert_eq!(reason.args().len(), 5);
    assert_eq!(reason.args()[0], group.anchor_midpoint.into());
    assert_eq!(
        reason.args()[1],
        (resolution.features.count() as i32).into()
    );
    let drift = double_arg(&reason.args()[2]);
    let wanted = feature_extent(&resolution.features) * DEFAULT_DRIFT_FRACTION_OF_FEATURE_EXTENT;
    assert!(
        (drift - wanted).abs() <= 1e-9,
        "the drift window is the one number the user can read the refusal against ({drift})"
    );
    assert_eq!(reason.args()[3], EDGE_DIRECTION_TOLERANCE_DEGREES.into());
    assert_eq!(reason.args()[4], DEFAULT_NORMAL_TOLERANCE_DEGREES.into());
}

#[test]
fn an_anchor_two_edges_fit_equally_well_fails_quoting_both_scores() {
    let group = top_front_edge_group(
        BevelSelectionKind::Edge,
        FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES,
    );

    // the same cube twice, half a millimetre either side of where the anchor was made: two top
    // front edges, each as good a match as the other
    let mut pair = Mesh::new();
    append_cube(&mut pair, Vector3::new(-0.5, 0.0, 0.0));
    append_cube(&mut pair, Vector3::new(0.5, 0.0, 0.0));

    let resolution = bevel_edge_group_resolver::resolve(&group, &pair);

    assert!(!resolution.is_resolved());
    assert_eq!(resolution.seed_edge_index, -1);

    let reason = resolution.failure_reason.clone().expect("a refusal");

    assert_eq!(
        reason.code(),
        BevelAdvisoryCode::AnchorMatchesMoreThanOneEdge
    );
    assert_eq!(reason.args().len(), 3);
    assert_eq!(reason.args()[0], group.anchor_midpoint.into());
    assert!(
        (double_arg(&reason.args()[1]) - double_arg(&reason.args()[2])).abs() <= 1e-9,
        "the two copies sit at the same distance with the same faces, so neither can win"
    );
}

#[test]
fn a_face_region_bounded_only_by_open_edges_fails_counting_them() {
    const FLOOD_ACROSS_FACETS: f64 = 45.0;

    let tube = open_tube(10.0, 10.0, TUBE_SIDES);
    let features = FeatureEdgeSet::create(&tube);

    assert_eq!(
        features.count(),
        TUBE_SIDES as usize,
        "the facet joins are the only feature edges an open tube has; its rims have one face each"
    );

    let group = bevel_edge_group_resolver::create_from_edge(
        &features,
        0,
        BevelSelectionKind::FaceGroup,
        FLOOD_ACROSS_FACETS,
    )
    .expect("edge 0 is in the set");

    let resolution = bevel_edge_group_resolver::resolve(&group, &tube);

    assert!(!resolution.is_resolved());
    assert_eq!(
        resolution.failure_reason,
        Some(BevelAdvisory::new(
            BevelAdvisoryCode::FaceRegionBoundedOnlyByOpenEdges,
            vec![group.anchor_midpoint.into(), (TUBE_SIDES * 2).into()]
        )),
        "the whole wall floods, and both rims - a facet's worth of open edges each - bound it"
    );
}

#[test]
fn a_face_region_that_never_turns_fails_naming_the_curve_tolerance() {
    // flood across the cube's own 90 degree corners and the region is the whole closed surface
    const FLOOD_ACROSS_CORNERS: f64 = 95.0;

    let group = top_front_edge_group(BevelSelectionKind::FaceGroup, FLOOD_ACROSS_CORNERS);

    let resolution =
        bevel_edge_group_resolver::resolve(&group, &create_cube(CUBE_SIZE, CUBE_SIZE, CUBE_SIZE));

    assert!(!resolution.is_resolved());
    assert_eq!(
        resolution.failure_reason,
        Some(BevelAdvisory::new(
            BevelAdvisoryCode::FaceRegionHasNoBevelableBoundary,
            vec![group.anchor_midpoint.into(), FLOOD_ACROSS_CORNERS.into()]
        ))
    );
}

/// A group anchored on the top front edge of the standard cube, resolvable on that cube and on
/// nothing much else.
fn top_front_edge_group(
    selection_kind: BevelSelectionKind,
    curve_tolerance_degrees: f64,
) -> BevelEdgeGroup {
    let cube = create_cube(CUBE_SIZE, CUBE_SIZE, CUBE_SIZE);
    let features = FeatureEdgeSet::create(&cube);

    for i in 0..features.count() {
        let edge = &features.edges()[i];
        if ((edge.start + edge.end) / 2.0 - top_front_midpoint()).length() < 1e-6 {
            return bevel_edge_group_resolver::create_from_edge(
                &features,
                i as i32,
                selection_kind,
                curve_tolerance_degrees,
            )
            .expect("the edge is in the set");
        }
    }

    panic!("the cube has no top front edge");
}

/// Adds a standard cube to a mesh, moved by an offset, sharing vertices within itself and with
/// nothing else - copied vertex by vertex, since an unwelded mesh has no feature edges.
fn append_cube(mesh: &mut Mesh, offset: Vector3) {
    let cube = create_cube(CUBE_SIZE, CUBE_SIZE, CUBE_SIZE);
    let first_vertex = mesh.vertices.len() as i32;

    for vertex in &cube.vertices {
        mesh.vertices.push(Vector3Float::from_vector3(
            Vector3::from_vector3_float(*vertex) + offset,
        ));
    }

    for face in cube.faces.iter() {
        let (v0, v1, v2) = (face.v0, face.v1, face.v2);
        let vertices = mesh.vertices.clone();
        mesh.faces.add(
            first_vertex + v0,
            first_vertex + v1,
            first_vertex + v2,
            &vertices,
        );
    }
}

/// The diagonal of the feature edges' bounding box, which is the scale the resolver measures
/// drift in.
fn feature_extent(features: &FeatureEdgeSet) -> f64 {
    let mut bounds = AxisAlignedBoundingBox::empty();

    for edge in features.edges() {
        bounds.expand_to_include_point(edge.start);
        bounds.expand_to_include_point(edge.end);
    }

    bounds.size().length()
}

/// A tube wall with no caps, wound outward, whose rims are open boundaries.
fn open_tube(radius: f64, height: f64, sides: i32) -> Mesh {
    let mut tube = Mesh::new();

    for side in 0..sides {
        let angle = math_helper::TAU * side as f64 / sides as f64;
        let x = radius * angle.cos();
        let y = radius * angle.sin();
        tube.vertices
            .push(Vector3Float::from_vector3(Vector3::new(x, y, 0.0)));
        tube.vertices
            .push(Vector3Float::from_vector3(Vector3::new(x, y, height)));
    }

    for side in 0..sides {
        let next_side = (side + 1) % sides;
        let bottom = side * 2;
        let top = bottom + 1;
        let next_bottom = next_side * 2;
        let next_top = next_bottom + 1;
        let vertices = tube.vertices.clone();
        tube.faces.add(bottom, next_bottom, next_top, &vertices);
        tube.faces.add(bottom, next_top, top, &vertices);
    }

    tube
}
