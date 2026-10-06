//! Port of `BevelPairCornerRecognitionTests.cs`: two selected runs meeting over an unselected
//! crease between surfaces both overshoot the corner (and pair, taking the ball, where the crease
//! is concave); a shallow step keeps its cap with the surface labelling in hand.

use std::sync::Arc;

use geometry_processing::bevel::bevel_chain_end_cap::ResolveOptions;
use geometry_processing::bevel::bevel_working_mesh::signed_volume;
use geometry_processing::bevel::{bevel_chain_linker, BevelChainEnds};
use geometry_processing::topology::{
    bevel_edge_group_resolver, BevelEdgeGroupResolution, BevelSelectionKind, EdgeConvexity,
    FaceGroup, FeatureEdge, FeatureEdgeSet,
};
use polygon_mesh::Mesh;
use vector_math::math_helper::radians_to_degrees;
use vector_math::Vector3;

use crate::topology_test_meshes::{create_bore_block_with_chord_wall, create_shallow_ridge_block};
use crate::warped_wall_test_meshes::{create_warped_wall_block, create_warped_wall_l};

/// C# `TwoRunsMeetingOverACreaseBetweenAFacettedBoreAndAFlatWallBothOvershootTheCorner`.
#[test]
fn two_runs_meeting_over_a_crease_between_a_facetted_bore_and_a_flat_wall_both_overshoot_the_corner(
) {
    let radius = 2.0;
    let mesh = create_bore_block_with_chord_wall(24);
    let features = Arc::new(FeatureEdgeSet::create_with_angle(
        &mesh,
        FeatureEdge::default_flat_angle_degrees(),
    ));
    let junction = vertex_at(
        &mesh,
        &features,
        Vector3::new((64.0f64 - 36.0).sqrt(), 6.0, 10.0),
        0.2,
    );
    assert_eq!(features.edges_at_vertex(junction).len(), 3);
    assert_both_runs_overshoot_the_corner(
        &mesh,
        &features,
        junction,
        radius,
        EdgeConvexity::Convex,
    );
}

/// C# `TwoRunsMeetingOverACreaseOnAWarpedWallBothOvershootTheCorner`.
#[test]
fn two_runs_meeting_over_a_crease_on_a_warped_wall_both_overshoot_the_corner() {
    let radius = 2.0;
    let mesh = create_warped_wall_block();
    assert!(mesh.is_manifold());
    let features = Arc::new(FeatureEdgeSet::create_with_angle(&mesh, 10.0));
    let junction = vertex_at(&mesh, &features, Vector3::new(40.0, 14.0, 20.0), 1e-4);
    assert_eq!(features.edges_at_vertex(junction).len(), 3);
    assert_the_warped_faces_differ_a_little(&features, junction);
    assert_both_runs_overshoot_the_corner(
        &mesh,
        &features,
        junction,
        radius,
        EdgeConvexity::Convex,
    );
}

/// C# `TwoRunsMeetingOverAConcaveCreaseOnAWarpedWallPairAndTakeTheBall`.
#[test]
fn two_runs_meeting_over_a_concave_crease_on_a_warped_wall_pair_and_take_the_ball() {
    let radius = 2.0;
    let mesh = create_warped_wall_l();
    assert!(mesh.is_manifold());
    let warped_quads = 50.0;
    let per_quad_excess = 16.0 * 0.32 / 12.0;
    let volume = signed_volume(&mesh);
    assert!(
        (volume - (32000.0 + (warped_quads * per_quad_excess))).abs() <= 1e-3,
        "{volume}"
    );
    let features = Arc::new(FeatureEdgeSet::create_with_angle(&mesh, 10.0));
    let junction = vertex_at(&mesh, &features, Vector3::new(40.0, 14.0, 20.0), 1e-4);
    assert_eq!(features.edges_at_vertex(junction).len(), 3);
    assert_the_warped_faces_differ_a_little(&features, junction);
    assert_both_runs_overshoot_the_corner(
        &mesh,
        &features,
        junction,
        radius,
        EdgeConvexity::Concave,
    );
}

/// C# `AShallowStepKeepsItsCapEvenWithTheSurfaceLabellingInHand`.
#[test]
fn a_shallow_step_keeps_its_cap_even_with_the_surface_labelling_in_hand() {
    let radius = 1.0;
    let mesh = create_shallow_ridge_block(20.0);
    let features = FeatureEdgeSet::create(&mesh);
    let corner = vertex_at(&mesh, &features, Vector3::new(10.0, 10.0, 5.0), 1e-4);
    assert_eq!(features.edges_at_vertex(corner).len(), 4);

    let at_corner: Vec<FeatureEdge> = features
        .edges_at_vertex(corner)
        .iter()
        .map(|&i| features.edges()[i as usize])
        .collect();
    let ridge = *at_corner
        .iter()
        .find(|e| {
            (e.start.z - e.end.z).abs() < 1e-6
                && (e.start.x - e.end.x).abs() > 1e-6
                && (e.start.y - e.end.y).abs() > 1e-6
        })
        .unwrap();
    assert!((ridge.turn_angle_degrees.abs() - 20.0).abs() <= 0.1);
    let rim = *at_corner
        .iter()
        .find(|e| {
            (e.start.z - 5.0).abs() < 1e-6
                && (e.end.z - 5.0).abs() < 1e-6
                && (e.start.y - 10.0).abs() < 1e-6
                && (e.end.y - 10.0).abs() < 1e-6
        })
        .unwrap();
    let vertical = *at_corner
        .iter()
        .find(|e| (e.start.x - e.end.x).abs() < 1e-6 && (e.start.y - e.end.y).abs() < 1e-6)
        .unwrap();
    assert_eq!(vertical.convexity, rim.convexity);

    let selected = [rim, vertical];
    let run = bevel_chain_linker::link(&[rim]).remove(0);
    let surfaces = FaceGroup::label_surfaces(
        &mesh,
        &mesh.get_mesh_edge_graph(),
        FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES,
    );
    let mut ask = || Some(surfaces.as_slice());
    let with_labels = BevelChainEnds::resolve(
        &run.edges,
        run.is_closed,
        radius,
        Some(&features),
        Some(&selected),
        None,
        ResolveOptions {
            surface_of_face: Some(&mut ask),
            ..Default::default()
        },
    );
    let without_labels = BevelChainEnds::resolve(
        &run.edges,
        run.is_closed,
        radius,
        Some(&features),
        Some(&selected),
        None,
        ResolveOptions::default(),
    );

    let at_start = run.edges[0].from_vertex() == corner;
    let at_corner_without = if at_start {
        &without_labels.start
    } else {
        &without_labels.end
    };
    let at_corner_with = if at_start {
        &with_labels.start
    } else {
        &with_labels.end
    };
    assert!(at_corner_without.is_capped());
    assert!(at_corner_with.is_capped());
    let ramp_normal = if (ridge.normal0.z - 1.0).abs() < 1e-6 {
        ridge.normal1
    } else {
        ridge.normal0
    };
    assert!(
        (at_corner_without
            .plane_normal
            .dot(Vector3::new(1.0, 0.0, 0.0))
            .abs()
            - 1.0)
            .abs()
            <= 1e-6
    );
    assert!((at_corner_with.plane_normal.dot(ramp_normal).abs() - 1.0).abs() <= 1e-6);
}

fn assert_the_warped_faces_differ_a_little(features: &FeatureEdgeSet, junction: i32) {
    let (crease_normal, rim_normal) = warped_faces_at(features, junction);
    let between_faces = radians_to_degrees(crease_normal.dot(rim_normal).clamp(-1.0, 1.0).acos());
    assert!(between_faces > 1.0, "{between_faces}");
    assert!(
        between_faces < FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES,
        "{between_faces}"
    );
}

fn assert_both_runs_overshoot_the_corner(
    mesh: &Mesh,
    features: &Arc<FeatureEdgeSet>,
    junction: i32,
    radius: f64,
    expected_crease: EdgeConvexity,
) {
    let rim = rim_edges_at(features, junction);
    assert_eq!(rim.len(), 2, "two convex rim runs meet at the junction");
    let crease: Vec<FeatureEdge> = features
        .edges_at_vertex(junction)
        .iter()
        .filter(|i| !rim.contains(i))
        .map(|&i| features.edges()[i as usize])
        .collect();
    assert_eq!(crease.len(), 1);
    let crease = crease[0];

    let selected_across_groups: Vec<FeatureEdge> =
        rim.iter().map(|&i| features.edges()[i as usize]).collect();
    let surfaces = FaceGroup::label_surfaces(
        mesh,
        &mesh.get_mesh_edge_graph(),
        FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES,
    );

    for &seed in &rim {
        let group = bevel_edge_group_resolver::create_from_edge(
            features,
            seed,
            BevelSelectionKind::Edge,
            FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES,
        )
        .unwrap();
        let resolution = BevelEdgeGroupResolution {
            group,
            features: features.clone(),
            failure_reason: None,
            seed_edge_index: seed,
            edges: vec![features.edges()[seed as usize]],
            chain: None,
            face_group: None,
        };
        let run = bevel_chain_linker::link_resolution(&resolution).remove(0);
        let mut warnings = Vec::new();
        let mut ask = || Some(surfaces.as_slice());
        let ends = BevelChainEnds::resolve(
            &run.edges,
            run.is_closed,
            radius,
            Some(features),
            Some(&selected_across_groups),
            Some(&mut warnings),
            ResolveOptions {
                surface_of_face: Some(&mut ask),
                ..Default::default()
            },
        );

        let at_start = run.edges[0].from_vertex() == junction;
        let cap = if at_start { &ends.start } else { &ends.end };
        let pair_corner = if at_start {
            ends.start_pair_corner
        } else {
            ends.end_pair_corner
        };
        let overshoot_corner = if at_start {
            ends.start_overshoot_corner
        } else {
            ends.end_overshoot_corner
        };

        assert!(
            !cap.is_capped(),
            "the run overshoots the corner rather than capping"
        );
        assert_eq!(overshoot_corner, Some(junction));
        assert_eq!(
            crease.convexity, expected_crease,
            "the crease turns {} degrees",
            crease.turn_angle_degrees
        );
        if expected_crease == EdgeConvexity::Convex {
            assert_eq!(pair_corner, None);
        } else {
            assert_eq!(pair_corner, Some(junction));
        }
    }
}

fn rim_edges_at(features: &FeatureEdgeSet, vertex: i32) -> Vec<i32> {
    features
        .edges_at_vertex(vertex)
        .iter()
        .copied()
        .filter(|&i| {
            let e = &features.edges()[i as usize];
            e.convexity == EdgeConvexity::Convex && (e.start.z - e.end.z).abs() < 1e-6
        })
        .collect()
}

fn warped_faces_at(features: &FeatureEdgeSet, vertex: i32) -> (Vector3, Vector3) {
    let incident: Vec<FeatureEdge> = features
        .edges_at_vertex(vertex)
        .iter()
        .map(|&i| features.edges()[i as usize])
        .collect();
    let crease = *incident
        .iter()
        .find(|e| (e.start.z - e.end.z).abs() > 1e-6)
        .unwrap();
    let rim = *incident
        .iter()
        .find(|e| {
            e.mesh_edge_index != crease.mesh_edge_index
                && e.normal0.z.abs() > 1e-6
                && e.normal1.z.abs() > 1e-6
        })
        .unwrap();
    let warped = |edge: &FeatureEdge| {
        if edge.normal0.y.abs() > 0.5 && edge.normal0.x.abs() > 1e-6 {
            edge.normal0
        } else {
            edge.normal1
        }
    };
    (warped(&crease), warped(&rim))
}

fn vertex_at(mesh: &Mesh, features: &FeatureEdgeSet, position: Vector3, tolerance: f64) -> i32 {
    for edge in features.edges() {
        for vertex in [edge.start_vertex, edge.end_vertex] {
            if (Vector3::from_vector3_float(mesh.vertices[vertex as usize]) - position).length()
                <= tolerance
            {
                return vertex;
            }
        }
    }
    panic!("no feature vertex within {tolerance} of {position:?}");
}
