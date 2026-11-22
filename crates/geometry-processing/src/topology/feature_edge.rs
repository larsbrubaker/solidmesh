//! One manifold edge of a mesh with the geometry a bevel needs ([`FeatureEdge`], with its
//! [`EdgeConvexity`]). [`super::FeatureEdgeSet`] keeps
//! the ones that are edges of the shape; chains and face groups are built from them. A face too
//! thin for its corners to give it a normal reads the face it lies along
//! ([`the_normal_an_edge_reads`], by `sliver_normal`'s one rule), so a sliver adds no crease.

use solidmesh_math::{math_helper, Vector3};

use super::sliver_normal::{
    normal_of_corners, trusted_normal_across_slivers, SINGLE_PRECISION_PLACEMENT,
};
use solidmesh_mesh::{Face, Mesh, MeshEdgeGraph};

/// Which way the solid folds where two faces meet along an edge.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum EdgeConvexity {
    /// The two faces are within the flat tolerance of the same plane. The edge is a
    /// triangulation artifact rather than an edge of the shape.
    #[default]
    Flat,
    /// The material is on the inside of the fold - a box corner. Beveling one cuts material
    /// away.
    Convex,
    /// The material is on the outside of the fold - the inner corner of an L. Filleting one adds
    /// material.
    Concave,
}

/// One manifold edge of a mesh together with the geometry a bevel/fillet needs: the two faces
/// that meet there, where the edge runs, and how far and which way the surface turns across it.
///
/// Sign convention (chain propagation and tool construction depend on it): the edge is stored
/// oriented the way `face0` winds it, so walking Face0's three vertices in order passes
/// `start_vertex` immediately before `end_vertex`, and `direction` points Start to End. With `n0`
/// and `n1` the outward normals of Face0 and Face1, the edge is convex when
/// `cross(n0, n1) . direction > 0` and concave when that is negative; `turn_angle_degrees` is
/// `atan2(cross(n0, n1) . direction, n0 . n1)` in degrees, so it carries that same sign.
///
/// The convention does not care which of the two faces is called Face0: swapping them reverses
/// both the cross product and the edge direction, and the two sign flips cancel. It does care
/// that the mesh is wound consistently outward - a mesh turned inside out reports every convex
/// edge as concave.
///
/// `Default` is every index 0, every vector zero, flat.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct FeatureEdge {
    /// The index of this edge in the [`MeshEdgeGraph`] it was classified from.
    pub mesh_edge_index: i32,
    /// The mesh vertex the edge runs from, in `face0`'s winding order.
    pub start_vertex: i32,
    /// The mesh vertex the edge runs to, in `face0`'s winding order.
    pub end_vertex: i32,
    /// The position of `start_vertex`.
    pub start: Vector3,
    /// The position of `end_vertex`.
    pub end: Vector3,
    /// The unit vector from `start` to `end`.
    pub direction: Vector3,
    /// The face whose winding runs this edge from `start_vertex` to `end_vertex`.
    pub face0: i32,
    /// The other face on this edge, which winds it the other way.
    pub face1: i32,
    /// The outward unit normal of `face0`. Carried here so that everything walking feature edges
    /// (chains, tool frames) works from the snapshot rather than reaching back into a mesh that
    /// may have moved on.
    pub normal0: Vector3,
    /// The outward unit normal of `face1`.
    pub normal1: Vector3,
    /// How far the surface turns crossing this edge, in degrees: positive convex, negative
    /// concave, zero flat. A box edge turns +90, the inner edge of an L turns -90.
    pub turn_angle_degrees: f64,
    /// Which way the solid folds here. Only `Convex` and `Concave` edges are feature edges.
    pub convexity: EdgeConvexity,
}

impl FeatureEdge {
    /// Two faces closer together in direction than this dot product are treated as one plane.
    /// It is the number the bevel work has always used to drop triangulation diagonals.
    pub const COPLANAR_FACE_DOT: f64 = 0.99996;

    /// The default flat tolerance, about 0.512 degrees. It is `acos(COPLANAR_FACE_DOT)` rather
    /// than a round half degree so that detection here selects exactly the edge set a hand-rolled
    /// coplanar-face scan at that dot product would. (A function because `acos` is not `const`.)
    pub fn default_flat_angle_degrees() -> f64 {
        math_helper::radians_to_degrees(libm::acos(Self::COPLANAR_FACE_DOT))
    }

    /// The angle between the two faces measured through the material, in degrees. A box edge is
    /// 90, the inner edge of an L is 270, a flat edge is 180.
    pub fn dihedral_angle_degrees(&self) -> f64 {
        180.0 - self.turn_angle_degrees
    }

    /// Classifies one edge of a mesh, whatever its dihedral angle. `None` for edges a bevel can
    /// say nothing about: boundary edges (one face), non-manifold edges (three or more), and
    /// edges where no face with a normal stands in for a face too thin to have one
    /// ([`the_normal_an_edge_reads`]). The result may be `Flat`.
    pub fn try_classify(
        mesh: &Mesh,
        graph: &MeshEdgeGraph,
        mesh_edge_index: i32,
        flat_angle_degrees: f64,
    ) -> Option<FeatureEdge> {
        if graph.get_face_count(mesh_edge_index) != 2 {
            return None;
        }

        let face0 = graph.get_face(mesh_edge_index, 0);
        let face1 = graph.get_face(mesh_edge_index, 1);

        // The whole sign convention hangs on taking the edge direction from Face0's winding, so
        // the graph's index-ordered vertex pair has to be re-oriented against that face before
        // anything else.
        let (start_vertex, end_vertex) = try_orient_to_winding(
            &mesh.faces[face0 as usize],
            graph.get_vertex0(mesh_edge_index),
            graph.get_vertex1(mesh_edge_index),
        )?;

        let start = Vector3::from_vector3_float(mesh.vertices[start_vertex as usize]);
        let end = Vector3::from_vector3_float(mesh.vertices[end_vertex as usize]);
        let along_edge = end - start;
        if along_edge.length_squared() <= 0.0 {
            return None;
        }

        let normal0 = the_normal_an_edge_reads(mesh, graph, face0)?;
        let normal1 = the_normal_an_edge_reads(mesh, graph, face1)?;

        let direction = along_edge.get_normal();

        // cross(n0, n1) is perpendicular to both normals and so runs along the edge: dotting it
        // with the edge direction recovers the signed sine of the turn, which atan2 pairs with
        // the cosine to give a full -180..180 angle (a plain acos of the dot loses the
        // convex/concave sign).
        let sine_of_turn = normal0.cross(normal1).dot(direction);
        let cosine_of_turn = normal0.dot(normal1);
        let turn_angle_degrees =
            math_helper::radians_to_degrees(libm::atan2(sine_of_turn, cosine_of_turn));

        let mut convexity = EdgeConvexity::Flat;
        if turn_angle_degrees > flat_angle_degrees {
            convexity = EdgeConvexity::Convex;
        } else if turn_angle_degrees < -flat_angle_degrees {
            convexity = EdgeConvexity::Concave;
        }

        Some(FeatureEdge {
            mesh_edge_index,
            start_vertex,
            end_vertex,
            start,
            end,
            direction,
            face0,
            face1,
            normal0,
            normal1,
            turn_angle_degrees,
            convexity,
        })
    }
}

/// Puts the edge's two vertices in the order the face walks them, which is what fixes the sign
/// of `turn_angle_degrees`. `None` if the face does not walk this pair at all, which a degenerate
/// face repeating a vertex can manage.
fn try_orient_to_winding(face: &Face, vertex_a: i32, vertex_b: i32) -> Option<(i32, i32)> {
    let wound = [face.v0, face.v1, face.v2];

    for i in 0..3 {
        let from = wound[i];
        let to = wound[(i + 1) % 3];

        if from == vertex_a && to == vertex_b {
            return Some((vertex_a, vertex_b));
        }

        if from == vertex_b && to == vertex_a {
            return Some((vertex_b, vertex_a));
        }
    }

    None
}

/// The face's normal, taken from its vertex positions rather than from its cached `Face::normal`:
/// the cached one is f32 and can be stale (nothing forces a recalculation after a caller edits
/// vertices), and a wrong normal here silently mislabels an edge convex or concave.
///
/// `None` on a face too thin for its corners to be trusted with a normal ([`normal_of_corners`]):
/// a turn read from it would be a fact about where its corners were rounded to, not about the
/// shape.
fn try_get_face_normal(mesh: &Mesh, face_index: i32) -> Option<Vector3> {
    let face = mesh.faces[face_index as usize];
    normal_of_corners(
        [
            Vector3::from_vector3_float(mesh.vertices[face.v0 as usize]),
            Vector3::from_vector3_float(mesh.vertices[face.v1 as usize]),
            Vector3::from_vector3_float(mesh.vertices[face.v2 as usize]),
        ],
        // A `Mesh` stores its vertices in single precision.
        SINGLE_PRECISION_PLACEMENT,
    )
}

/// The outward normal an edge of `face_index` reads its turn with, by the one rule
/// (`sliver_normal::trusted_normal_across_slivers`): the face's own where its corners give it one
/// ([`try_get_face_normal`]), and otherwise that of the face the sliver lies along. Read this way
/// a sliver inside a flat face adds no crease (the normal its corners give it is noise that can
/// point anywhere, even straight back into the part), and a sliver lying along a real crease
/// leaves the crease to the edges around it. `None` where no face with a normal is reached.
fn the_normal_an_edge_reads(
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    face_index: i32,
) -> Option<Vector3> {
    trusted_normal_across_slivers(
        face_index,
        &|face| try_get_face_normal(mesh, face),
        &|face| {
            let f = mesh.faces[face as usize];
            [f.v0, f.v1, f.v2]
        },
        &|v| Vector3::from_vector3_float(mesh.vertices[v as usize]),
        &|face, (a, b)| {
            let edge = edge_between(graph, a, b)?;
            if graph.get_face_count(edge) != 2 {
                return None;
            }
            graph.get_faces(edge).iter().copied().find(|&f| f != face)
        },
    )
}

/// The graph's index of the edge joining `a` and `b`: the graph lists its edges sorted by their
/// lower vertex index, then their higher.
pub(crate) fn edge_between(graph: &MeshEdgeGraph, a: i32, b: i32) -> Option<i32> {
    let key = (a.min(b), a.max(b));
    let (mut low, mut high) = (0, graph.edge_count());
    while low < high {
        let middle = low + (high - low) / 2;
        match (graph.get_vertex0(middle), graph.get_vertex1(middle)).cmp(&key) {
            std::cmp::Ordering::Less => low = middle + 1,
            std::cmp::Ordering::Greater => high = middle,
            std::cmp::Ordering::Equal => return Some(middle),
        }
    }

    None
}
