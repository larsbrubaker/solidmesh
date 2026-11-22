//! Whether two faces standing at one vertex lie in one face region there ([`share_a_region_at`]):
//! the faces round the vertex walked across every mesh edge at it that is no feature edge. The
//! chain linker reads it at a junction of picked creases (`bevel_chain_linker::link_on_the_mesh`)
//! to tell the crease that carries a run on from one that ends on it.

use solidmesh_mesh::{Mesh, MeshEdgeGraph};

use super::feature_edge::edge_between;
use super::feature_edge_set::FeatureEdgeSet;

/// Whether `from_face` and `to_face`, both standing at `vertex`, are one face region round it: the
/// walk from `from_face` round the vertex reaches `to_face` without crossing a feature edge of
/// `features`, a boundary edge or a non-manifold one. A curved wall's facets meet across edges too
/// shallow to be features, so its faces at a vertex are one region however many there are; two
/// faces with a crease between them on every way round are two.
pub fn share_a_region_at(
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    features: &FeatureEdgeSet,
    vertex: i32,
    from_face: i32,
    to_face: i32,
) -> bool {
    if from_face == to_face {
        return true;
    }

    let mut reached = vec![from_face];
    let mut next = 0;
    while next < reached.len() {
        let face = mesh.faces[reached[next] as usize];
        next += 1;
        let corners = [face.v0, face.v1, face.v2];
        if !corners.contains(&vertex) {
            continue;
        }

        for &other in corners.iter().filter(|&&corner| corner != vertex) {
            let Some(edge) = edge_between(graph, vertex, other) else {
                continue;
            };
            if graph.get_face_count(edge) != 2 || features.index_of_mesh_edge(edge) >= 0 {
                continue;
            }

            for &beyond in graph.get_faces(edge) {
                if beyond == to_face {
                    return true;
                }
                if !reached.contains(&beyond) {
                    reached.push(beyond);
                }
            }
        }
    }

    false
}
