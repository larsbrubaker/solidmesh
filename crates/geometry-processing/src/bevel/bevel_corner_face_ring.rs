//! The one question both corner constructions have to ask their selected ends before they can build
//! anything: do these edges close a RING of faces round the vertex, and in what order do those
//! faces go?
//!
//! `BevelCornerPatches` needs the order - the shell it draws is the spherical polygon whose
//! corners are those normals - and `BevelMixedBlends` needs only the answer, because a ring of
//! exactly three is what makes its pairing of edges to faces unambiguous. It is one walk either
//! way.
//!
//! Faces are matched by NORMAL rather than by face index, because a face index is one triangle: a
//! cube corner has three planes but up to six triangles on them, and which pair of triangles an
//! edge lands on depends on which way each square happened to be split. Two triangles closer
//! together than a feature edge's own flat angle are the same surface by definition.

use vector_math::Vector3;

use crate::topology::{FeatureEdge, FeatureEdgeSet};

/// A closed ring of planes round one vertex: `normals[i]` is one normal per distinct plane,
/// walked round the vertex, and `edge_of_sector[i]` is the index (into the edges given) of the
/// edge between `normals[i]` and the next one round.
#[derive(Debug, Clone, PartialEq)]
pub struct CornerFaceRing {
    /// One normal per distinct plane, in the order they go round the vertex.
    pub normals: Vec<Vector3>,
    /// Which of the given edges separates each neighbouring pair of planes.
    pub edge_of_sector: Vec<usize>,
}

/// The corner's face planes in the order they go round the vertex, or `None` where the selected
/// edges do not close a ring of them - which is what a partial selection looks like from here,
/// and is exactly the case whose ends are still capped against the faces of whatever was left
/// out. `edges` are the selected ends at one vertex, in any order.
///
/// The patch's shell is one wall per sector, and each of those walls is the seam a run ends on
/// (`BevelSeamRing`), so the seam has to be published against the run's own mesh edge: the walk
/// writes down which edge it stepped through to get from one plane to the next.
pub fn try_order(edges: &[FeatureEdge]) -> Option<CornerFaceRing> {
    let count = edges.len();

    // Plane index per distinct normal, and which two planes each selected edge separates
    let mut planes: Vec<Vector3> = Vec::new();
    let mut edge_planes: Vec<(usize, usize)> = Vec::with_capacity(count);

    for edge in edges {
        let first = plane_of(&mut planes, edge.normal0);
        let second = plane_of(&mut planes, edge.normal1);
        edge_planes.push((first, second));

        if first == second {
            return None;
        }
    }

    if planes.len() != count {
        return None;
    }

    // Every plane has to be shared by exactly two of the selected edges, or the ring is open. Every
    // plane was created by an edge, so each list is non-empty.
    let mut edges_on_plane: Vec<Vec<usize>> = vec![Vec::new(); planes.len()];

    for (i, &(first, second)) in edge_planes.iter().enumerate() {
        edges_on_plane[first].push(i);
        edges_on_plane[second].push(i);
    }

    if edges_on_plane.iter().any(|on_plane| on_plane.len() != 2) {
        return None;
    }

    // Walk edge to plane to edge; N steps have to come back to where they started
    let mut ordered = Vec::with_capacity(count);
    let mut sector_edges = Vec::with_capacity(count);
    let mut seen = vec![false; planes.len()];

    let mut current_edge = 0usize;
    let mut current_plane = edge_planes[0].0;

    for _ in 0..count {
        if seen[current_plane] {
            return None;
        }
        seen[current_plane] = true;

        ordered.push(planes[current_plane]);

        // The edge stepped through here is the one between this plane and the next, which is
        // this sector's own run.
        sector_edges.push(current_edge);

        current_plane = if edge_planes[current_edge].0 == current_plane {
            edge_planes[current_edge].1
        } else {
            edge_planes[current_edge].0
        };
        current_edge = if edges_on_plane[current_plane][0] == current_edge {
            edges_on_plane[current_plane][1]
        } else {
            edges_on_plane[current_plane][0]
        };
    }

    if current_edge != 0 || current_plane != edge_planes[0].0 {
        return None;
    }

    Some(CornerFaceRing {
        normals: ordered,
        edge_of_sector: sector_edges,
    })
}

/// The same ring read off the SURFACE at a vertex rather than off a selection: every feature
/// edge meeting there, in place of the ones somebody picked. Answers the ring's normals.
///
/// `features` must be the creases of the whole PART, at a flat angle of the part's own
/// (`FeatureEdge::default_flat_angle_degrees`, held by `BevelCornerVoids`) rather than of any
/// group's pick: a group's set is built at whatever tolerance that group was resolved with, and a
/// coarse one both misses creases the corner is made of and makes the answer depend on which
/// group asked. Two selected creases at a cube's inside corner leave the ring open (correctly -
/// no patch to build), but all three creases close it on the octant of void the corner opens.
///
/// Convexity is not read, deliberately: a MIXED vertex closes a ring like any other, and the cone
/// that follows is still a subset of the void, so a mixed corner can only understate it.
pub fn try_order_at_vertex(features: Option<&FeatureEdgeSet>, vertex: i32) -> Option<Vec<Vector3>> {
    let features = features?;

    let incident = features.edges_at_vertex(vertex);
    let edges: Vec<FeatureEdge> = incident
        .iter()
        .map(|&i| features.edges()[i as usize])
        .collect();

    if edges.len() > 2 {
        try_order(&edges).map(|ring| ring.normals)
    } else {
        None
    }
}

fn plane_of(planes: &mut Vec<Vector3>, normal: Vector3) -> usize {
    for (i, plane) in planes.iter().enumerate() {
        if plane.dot(normal) > FeatureEdge::COPLANAR_FACE_DOT {
            return i;
        }
    }

    planes.push(normal);

    planes.len() - 1
}
