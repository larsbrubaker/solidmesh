//! The ONE walk of a side's fan at a chain vertex: from the face the ARRIVING chain edge carries
//! on that side, face to face across the edges that meet at the vertex, to the LEAVING chain edge.
//! Ports `Bevel/Sections/SectionFanWalk.cs`.
//!
//! **Why it is one box.** `SectionBuilder` walks the fan to find the crease a boundary slides down
//! and `SectionFaceRebuild` walks it to rewrite the vertex; the boundary has to be used on exactly
//! the faces it was computed for, so those two walks may not be allowed to drift apart. Both call
//! this (`SectionEnd` too, round a whole ring).
//!
//! **What comes out.** The faces in the order they were turned through, and the edges CROSSED
//! between them - every mesh edge leaving the vertex strictly inside the fan, in order, which is
//! where a crease is looked for and where the re-mesh splits a slide side into its two flat
//! halves.

use std::collections::HashMap;

use polygon_mesh::{Mesh, MeshEdgeGraph};
use vector_math::Vector3;

use crate::bevel::{BevelAdvisory, BevelAdvisoryCode};
use crate::topology::{ChainEdge, FeatureEdge};

/// Every mesh edge by the pair of vertices it joins ([`edges_by_vertex_pair`], [`pair_key`]).
/// Only ever looked up, so a `HashMap`'s order cannot leak out.
pub type EdgeAtPair = HashMap<i64, i32>;

/// A fan turned through (C#'s two `out` lists).
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct FanWalk {
    /// The fan's faces, in the order they were turned through.
    pub faces: Vec<i32>,
    /// The edges stepped across, one fewer than there are faces: `crossed[j]` is the mesh edge
    /// between `faces[j]` and `faces[j + 1]`.
    pub crossed: Vec<i32>,
}

/// Turns the fan on one side of the chain at one chain vertex.
///
/// `mesh` is the part the chain was picked on, `graph` its edge graph and `edge_at_pair` its edges
/// looked up by the pair of vertices each joins. `vertex` is the chain vertex the fan turns about,
/// `arriving` the chain edge that ends there and `leaving` the one that starts there; `on_the_left`
/// which side of the walk to turn through; `station` where to name a refusal, on the part. The
/// error is why not, where the faces at the vertex do not form a fan at all. (C# also hands back
/// the partial lists on a refusal; none of its callers reads them.)
#[allow(clippy::too_many_arguments)]
pub fn try_walk(
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    edge_at_pair: &EdgeAtPair,
    vertex: i32,
    arriving: &ChainEdge,
    leaving: &ChainEdge,
    on_the_left: bool,
    station: Vector3,
) -> Result<FanWalk, BevelAdvisory> {
    try_walk_to(
        mesh,
        graph,
        edge_at_pair,
        vertex,
        if on_the_left {
            arriving.left_face()
        } else {
            arriving.right_face()
        },
        arriving.edge.mesh_edge_index,
        leaving.edge.mesh_edge_index,
        station,
    )
}

/// The same walk named by its two BOUNDS rather than by two chain edges: from `start_face`,
/// turning away from `start_edge`, to `stop_edge`.
///
/// This is what an open run's END vertex needs. There the fan on a side is not bounded by two
/// picked edges - there is only one - but by the chain edge and the CREASE that side shares with
/// the transverse face, which is exactly the edge `SectionEnd` recorded the boundary as sliding
/// down. The faces beyond that crease are the transverse face's own and belong to neither side.
#[allow(clippy::too_many_arguments)]
pub fn try_walk_to(
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    edge_at_pair: &EdgeAtPair,
    vertex: i32,
    start_face: i32,
    start_edge: i32,
    stop_edge: i32,
    station: Vector3,
) -> Result<FanWalk, BevelAdvisory> {
    let refusal = || {
        BevelAdvisory::new(
            BevelAdvisoryCode::SurfaceCornerNotSupported,
            vec![station.into()],
        )
    };

    let mut walk = FanWalk::default();
    let to = stop_edge;
    let mut face = start_face;
    let mut walked = start_edge;

    // The fan can be no longer than the faces of the mesh; the cap is only against a mesh whose
    // faces around one vertex do not form a fan at all.
    for _ in 0..=mesh.faces.len() {
        walk.faces.push(face);

        let Some(next) = try_the_other_edge_at(mesh, face, vertex, walked, edge_at_pair) else {
            return Err(refusal());
        };

        if next == to {
            return Ok(walk);
        }

        if graph.get_face_count(next) != 2 {
            return Err(refusal());
        }

        walk.crossed.push(next);

        face = if graph.get_face(next, 0) == face {
            graph.get_face(next, 1)
        } else {
            graph.get_face(next, 0)
        };
        walked = next;
    }

    Err(refusal())
}

/// Whether a mesh edge is a fold of the part rather than a line the triangulator drew across a
/// flat piece of it. An edge with anything other than two faces counts as one: it is not somewhere
/// a boundary may cross without being noticed.
pub fn is_a_crease(mesh: &Mesh, graph: &MeshEdgeGraph, edge: i32) -> bool {
    if graph.get_face_count(edge) != 2 {
        return true;
    }

    let one = Vector3::from_vector3_float(mesh.faces[graph.get_face(edge, 0) as usize].normal);
    let other = Vector3::from_vector3_float(mesh.faces[graph.get_face(edge, 1) as usize].normal);

    one.dot(other) < FeatureEdge::COPLANAR_FACE_DOT
}

/// Every mesh edge, looked up by the pair of vertices it joins. Where two edges join the same pair
/// the later one wins, as C#'s indexer assignment does.
pub fn edges_by_vertex_pair(graph: &MeshEdgeGraph) -> EdgeAtPair {
    let mut found = HashMap::with_capacity(graph.edge_count().max(0) as usize);

    for edge in 0..graph.edge_count() {
        found.insert(
            pair_key(graph.get_vertex0(edge), graph.get_vertex1(edge)),
            edge,
        );
    }

    found
}

/// The other of the two edges of `face` that meet at `vertex` - the one that is not `walked`. Read
/// off the ORIGINAL mesh, because the fan is a walk over the part's own topology and a
/// substitution changes none of it. Where both qualify (a face naming the vertex twice), the later
/// corner's edge is the answer, as in C#.
fn try_the_other_edge_at(
    mesh: &Mesh,
    face: i32,
    vertex: i32,
    walked: i32,
    edge_at_pair: &EdgeAtPair,
) -> Option<i32> {
    let drawn = &mesh.faces[face as usize];
    let mut other = -1;

    for corner in [drawn.v0, drawn.v1, drawn.v2] {
        if corner == vertex {
            continue;
        }

        let Some(&edge) = edge_at_pair.get(&pair_key(vertex, corner)) else {
            continue;
        };

        if edge != walked {
            other = edge;
        }
    }

    (other >= 0).then_some(other)
}

/// The key a vertex pair is looked up by, the same whichever way round it is named: the smaller
/// index in the high 32 bits, the larger's bit pattern (C# `(uint)`) in the low.
pub fn pair_key(one: i32, other: i32) -> i64 {
    let (smaller, larger) = if one < other {
        (one, other)
    } else {
        (other, one)
    };

    ((smaller as i64) << 32) | (larger as u32 as i64)
}
