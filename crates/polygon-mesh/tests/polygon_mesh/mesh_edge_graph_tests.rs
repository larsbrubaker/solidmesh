//! Port of `Agg.PolygonMesh/MeshEdgeGraphTests.cs`: the edge topology that both
//! `MeshEdgeGraph` and `MeshEdge::create_mesh_edge_list` must produce, written from the
//! geometry rather than from either implementation.

use std::collections::{HashMap, HashSet};

use polygon_mesh::solids::platonic_solids;
use polygon_mesh::Mesh;
use vector_math::Vector3Float;

type Adjacency = HashMap<(i32, i32), Vec<i32>>;

#[test]
fn closed_solids_have_two_faces_on_every_edge() {
    for mesh in [
        platonic_solids::create_tetrahedron(1.0),
        platonic_solids::create_cube(10.0, 10.0, 10.0),
        platonic_solids::create_octahedron(1.0),
        platonic_solids::create_icosahedron(1.0),
    ] {
        let adjacency = edge_adjacency_from_graph(&mesh);

        // Euler's formula for a closed genus zero surface: V - E + F = 2.
        assert_eq!(adjacency.len(), mesh.vertices.len() + mesh.faces.len() - 2);
        for faces in adjacency.values() {
            assert_eq!(faces.len(), 2);
        }

        // Every face contributes exactly its own three edges.
        let total: usize = adjacency.values().map(Vec::len).sum();
        assert_eq!(total, mesh.faces.len() * 3);
    }
}

#[test]
fn boundary_and_non_manifold_edges_report_every_adjacent_face() {
    let mesh = create_three_faces_sharing_one_edge();
    let expected: Adjacency = [
        ((0, 1), vec![0, 1, 2]), // the non-manifold spine, three faces hinge on it
        ((1, 2), vec![0]),
        ((0, 2), vec![0]),
        ((0, 3), vec![1]),
        ((1, 3), vec![1]),
        ((1, 4), vec![2]),
        ((0, 4), vec![2]),
    ]
    .into_iter()
    .collect();

    assert_adjacency_matches(&expected, &edge_adjacency_from_graph(&mesh));
    assert_adjacency_matches(&expected, &edge_adjacency_from_mesh_edge_list(&mesh));
}

#[test]
fn mesh_edge_list_agrees_with_flat_graph() {
    for mesh in [
        platonic_solids::create_tetrahedron(1.0),
        platonic_solids::create_cube(10.0, 10.0, 10.0),
        platonic_solids::create_icosahedron(1.0),
        create_three_faces_sharing_one_edge(),
    ] {
        assert_adjacency_matches(
            &edge_adjacency_from_mesh_edge_list(&mesh),
            &edge_adjacency_from_graph(&mesh),
        );
    }
}

#[test]
fn empty_mesh_produces_no_edges() {
    let graph = Mesh::new().get_mesh_edge_graph();
    assert_eq!(graph.edge_count(), 0);
}

#[test]
fn edge_vertices_are_ordered_low_to_high_and_are_unique() {
    let graph = platonic_solids::create_icosahedron(1.0).get_mesh_edge_graph();
    let mut seen = HashSet::new();
    for edge_index in 0..graph.edge_count() {
        assert!(graph.get_vertex0(edge_index) < graph.get_vertex1(edge_index));
        assert!(seen.insert((graph.get_vertex0(edge_index), graph.get_vertex1(edge_index))));
    }
}

/// Three triangles hinged on the shared edge (0, 1). Gives one non-manifold edge with three
/// faces and six boundary edges with one face each.
fn create_three_faces_sharing_one_edge() -> Mesh {
    let mut mesh = Mesh::new();
    mesh.vertices.push(Vector3Float::new(0.0, 0.0, 0.0));
    mesh.vertices.push(Vector3Float::new(1.0, 0.0, 0.0));
    mesh.vertices.push(Vector3Float::new(0.0, 1.0, 0.0));
    mesh.vertices.push(Vector3Float::new(0.0, -1.0, 0.0));
    mesh.vertices.push(Vector3Float::new(0.0, 0.0, 1.0));
    mesh.faces.add(0, 1, 2, &mesh.vertices);
    mesh.faces.add(0, 3, 1, &mesh.vertices);
    mesh.faces.add(0, 1, 4, &mesh.vertices);
    mesh
}

fn edge_adjacency_from_graph(mesh: &Mesh) -> Adjacency {
    let graph = mesh.get_mesh_edge_graph();
    let mut adjacency = Adjacency::new();
    for edge_index in 0..graph.edge_count() {
        let faces = (0..graph.get_face_count(edge_index))
            .map(|face| graph.get_face(edge_index, face))
            .collect();
        let key = (graph.get_vertex0(edge_index), graph.get_vertex1(edge_index));
        assert!(
            adjacency.insert(key, faces).is_none(),
            "duplicate edge {key:?}"
        );
    }
    adjacency
}

fn edge_adjacency_from_mesh_edge_list(mesh: &Mesh) -> Adjacency {
    let mut adjacency = Adjacency::new();
    for mesh_edge in mesh.get_mesh_edges() {
        let key = (mesh_edge.vertex0_index, mesh_edge.vertex1_index);
        assert!(
            adjacency.insert(key, mesh_edge.faces().to_vec()).is_none(),
            "duplicate edge {key:?}"
        );
    }
    adjacency
}

fn assert_adjacency_matches(expected: &Adjacency, actual: &Adjacency) {
    assert_eq!(actual.len(), expected.len());
    for (key, faces) in expected {
        // Face order within an edge is part of the contract - callers index faces[0]/faces[1].
        assert_eq!(actual.get(key), Some(faces), "edge {key:?}");
    }
}
