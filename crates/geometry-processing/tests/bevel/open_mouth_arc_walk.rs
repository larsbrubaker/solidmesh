//! The arc readers the open-mouth arc section tests share: the inner face's top mouth arc - the 40
//! feature edges of the bore's top mouth that lie in the top face's plane - walked end to end as an
//! open chain, and the same arc found again by vertex pair on a moved copy of the part
//! ([`crate::inner_face_test_mesh`], [`crate::rigid_motion_meshes`]).

#![allow(dead_code)]

use std::collections::{BTreeMap, HashMap, HashSet};

use solidmesh_mesh::Mesh;

use solidmesh::topology::{ChainEdge, FeatureEdgeSet};

use crate::inner_face_test_mesh;

/// The plane of the cube's -X wall, which the fillet's two ends must die on.
pub const MINUS_X_PLANE: f64 = -19.0;

/// The walk's vertex pairs found again among `features`, each oriented the way
/// the walk runs.
pub fn the_same_arc_on(features: &FeatureEdgeSet, walk: &[(i32, i32)]) -> Vec<ChainEdge> {
    let mut by_pair: HashMap<(i32, i32), i32> = HashMap::new();

    for (index, edge) in features.edges().iter().enumerate() {
        by_pair.insert((edge.start_vertex, edge.end_vertex), index as i32);
    }

    let mut chain = Vec::new();

    for &(from, to) in walk {
        if let Some(&index) = by_pair.get(&(from, to)) {
            chain.push(ChainEdge::new(
                index,
                features.edges()[index as usize],
                false,
            ));
        } else {
            let index = by_pair[&(to, from)];

            chain.push(ChainEdge::new(
                index,
                features.edges()[index as usize],
                true,
            ));
        }
    }

    chain
}

/// The arc's vertex pairs in order, read on the fixture where it was made.
pub fn the_top_mouth_arc_walk() -> Vec<(i32, i32)> {
    let part = inner_face_test_mesh::the_part();
    let graph = part.get_mesh_edge_graph();
    let features = FeatureEdgeSet::create_with_graph(
        &part,
        &graph,
        inner_face_test_mesh::BEND_THRESHOLD_DEGREES,
    );

    the_top_mouth_arc(&part, &features)
        .iter()
        .map(|edge| (edge.from_vertex(), edge.to_vertex()))
        .collect()
}

/// The 40 feature edges of the top mouth that lie in the top face's plane,
/// walked end to end as an open chain. Found by geometry rather than by index so the fixture may be
/// re-tessellated.
pub fn the_top_mouth_arc(_part: &Mesh, features: &FeatureEdgeSet) -> Vec<ChainEdge> {
    let mut on_top = Vec::new();

    for (index, edge) in features.edges().iter().enumerate() {
        if (edge.start.z - 20.0).abs() < 1e-6
            && (edge.end.z - 20.0).abs() < 1e-6
            && edge.start.x > MINUS_X_PLANE + 1e-6
            && edge.end.x > MINUS_X_PLANE + 1e-6
        {
            on_top.push(index as i32);
        }
    }

    walked_open(&on_top, features)
}

/// The given feature edges ordered end to end, starting from the lowest-numbered
/// end that has one neighbour.
fn walked_open(indices: &[i32], features: &FeatureEdgeSet) -> Vec<ChainEdge> {
    let mut by_vertex: BTreeMap<i32, Vec<i32>> = BTreeMap::new();

    for &index in indices {
        let edge = &features.edges()[index as usize];

        for vertex in [edge.start_vertex, edge.end_vertex] {
            by_vertex.entry(vertex).or_default().push(index);
        }
    }

    // The lowest-numbered end starts the walk, so it is the same every run; the BTreeMap is
    // already in that order.
    let start = *by_vertex
        .iter()
        .find(|(_, at)| at.len() == 1)
        .expect("an open arc has an end")
        .0;

    let mut walked = Vec::new();
    let mut used = HashSet::new();
    let mut at = start;

    while let Some(&next) = by_vertex[&at].iter().find(|index| !used.contains(*index)) {
        used.insert(next);

        let edge = &features.edges()[next as usize];
        let reversed = edge.end_vertex == at;

        walked.push(ChainEdge::new(next, *edge, reversed));
        at = if reversed {
            edge.start_vertex
        } else {
            edge.end_vertex
        };
    }

    walked
}
