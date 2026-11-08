//! Which picked edge a refusal's place blames (`part::refusal_place`) and the level-line answer
//! as a mesh (`part::answer_mesh`), read on bare edges and positions.

use solidmesh::bevel::part::answer_mesh::the_mesh_of;
use solidmesh::bevel::part::refusal_place::the_edge_nearest_the_place_among;
use solidmesh::bevel::{BevelAdvisory, BevelAdvisoryArg, BevelAdvisoryCode};
use solidmesh::topology::FeatureEdge;
use solidmesh_math::Vector3;

fn feature_edge(index: i32, start: Vector3, end: Vector3) -> FeatureEdge {
    FeatureEdge {
        mesh_edge_index: index,
        start,
        end,
        ..FeatureEdge::default()
    }
}

#[test]
fn rust_only_a_refusal_is_blamed_on_the_nearest_pick_only_when_it_is_a_clear_winner() {
    let near = vec![feature_edge(
        7,
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(10.0, 0.0, 0.0),
    )];
    let far = vec![feature_edge(
        9,
        Vector3::new(0.0, 5.0, 0.0),
        Vector3::new(10.0, 5.0, 0.0),
    )];
    let at = |y: f64| {
        BevelAdvisory::new(
            BevelAdvisoryCode::SizeRemovesNothing,
            vec![
                BevelAdvisoryArg::Double(2.0),
                BevelAdvisoryArg::Vector3(Vector3::new(5.0, y, 0.0)),
            ],
        )
    };

    let units = [near.clone(), far.clone()];
    assert_eq!(
        the_edge_nearest_the_place_among(&units, Some(&at(0.5)), 1.0),
        Some(7)
    );
    // Halfway between the two is nobody's.
    assert_eq!(
        the_edge_nearest_the_place_among(&units, Some(&at(2.5)), 1.0),
        None
    );
    // A refusal that names no place blames nothing.
    let placeless = BevelAdvisory::new(BevelAdvisoryCode::SizeRemovesNothing, vec![]);
    assert_eq!(
        the_edge_nearest_the_place_among(&units, Some(&placeless), 1.0),
        None
    );
    assert_eq!(the_edge_nearest_the_place_among(&units, None, 1.0), None);
}

#[test]
fn rust_only_the_mesh_of_keeps_only_the_vertices_a_triangle_stands_on() {
    let positions = [
        Vector3::new(9.0, 9.0, 9.0),
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
    ];
    let mesh = the_mesh_of(&positions, &[(3, 1, 2)]);

    assert_eq!(mesh.vertices.len(), 3);
    assert_eq!(mesh.faces.len(), 1);
    // numbered in the order the triangle reaches them
    assert_eq!(mesh.vertices[0].x, 0.0);
    assert_eq!(mesh.vertices[0].y, 1.0);
    assert_eq!(
        (mesh.faces[0].v0, mesh.faces[0].v1, mesh.faces[0].v2),
        (0, 1, 2)
    );
}
