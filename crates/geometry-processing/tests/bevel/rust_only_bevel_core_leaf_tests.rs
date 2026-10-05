//! Rust-only checks of the first leaf cluster of the bevel core (`BevelCornerFaceRing`,
//! `BevelCreaseMerge`, `BevelEndCap`, `BevelConstructionMode`). Their C# coverage comes through
//! the whole-bevel test classes that land with later slices; these pin the leaf behaviour now.

use geometry_processing::bevel::bevel_corner_face_ring::{try_order, try_order_at_vertex};
use geometry_processing::bevel::bevel_crease_merge::merge;
use geometry_processing::bevel::{BevelConstructionMode, BevelEndCap};
use geometry_processing::topology::{FeatureEdge, FeatureEdgeSet};
use polygon_mesh::solids::platonic_solids;
use vector_math::Vector3;

#[test]
fn rust_only_a_cube_corner_closes_a_ring_of_three_planes() {
    let cube = platonic_solids::create_cube(20.0, 20.0, 20.0);
    let features = FeatureEdgeSet::create(&cube);

    let corner = (0..cube.vertices.len() as i32)
        .find(|&v| features.edges_at_vertex(v).len() == 3)
        .expect("a cube corner meets three creases");

    let normals = try_order_at_vertex(Some(&features), corner).expect("three creases close");
    assert_eq!(normals.len(), 3);

    // Each consecutive pair of planes is one of the corner's creases: the normals are mutually
    // perpendicular, and the ring walks every one of them once.
    for i in 0..3 {
        assert!(normals[i].dot(normals[(i + 1) % 3]).abs() < 1e-9);
    }

    let edges: Vec<FeatureEdge> = features
        .edges_at_vertex(corner)
        .iter()
        .map(|&i| features.edges()[i as usize])
        .collect();
    let ring = try_order(&edges).expect("same ring from the selection");
    assert_eq!(ring.normals, normals);
    assert_eq!(
        ring.edge_of_sector[0], 0,
        "the walk starts through the first edge"
    );

    // Two of the three creases leave the ring open.
    assert!(try_order(&edges[..2]).is_none());
    assert!(try_order_at_vertex(None, corner).is_none());
}

#[test]
fn rust_only_creases_sharing_a_mesh_edge_merge_transitively_named_by_the_lowest() {
    let creases = vec![
        Some(vec![1, 2]),
        Some(vec![7]),
        Some(vec![2, 3]),
        None,
        Some(vec![]),
        Some(vec![3, 9]),
        Some(vec![7, 8]),
    ];

    assert_eq!(merge(&creases), vec![vec![0, 2, 5], vec![1, 6]]);
}

#[test]
fn rust_only_an_end_cap_slides_a_profile_point_onto_its_plane() {
    let cap = BevelEndCap::new(Vector3::new(1.0, 0.0, 0.0), 10.0);
    assert!(cap.is_capped());
    assert!(!cap.drives_through);
    assert_eq!(cap.seam_points, None);

    let slide = cap.slide_to_plane(
        Vector3::new(2.0, 0.0, 0.0),
        Vector3::new(1.0, 5.0, 0.0),
        Vector3::new(2.0, 0.0, 0.0),
    );
    assert_eq!(slide, 3.5);

    assert!(!BevelEndCap::default().is_capped());
}

#[test]
fn rust_only_the_construction_mode_saves_by_name() {
    assert_eq!(
        BevelConstructionMode::default(),
        BevelConstructionMode::Boolean
    );
    assert_eq!(
        serde_json::to_string(&BevelConstructionMode::Surface).unwrap(),
        "\"Surface\""
    );
    let read: BevelConstructionMode = serde_json::from_str("\"Boolean\"").unwrap();
    assert_eq!(read, BevelConstructionMode::Boolean);
    assert_eq!(BevelConstructionMode::Surface.to_string(), "Surface");
}
