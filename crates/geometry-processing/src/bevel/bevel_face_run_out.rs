//! How far a run may carry on PAST the end of its crease, and what stops it there
//! (`Bevel/BevelFaceRunOut.cs`): what stopped a run's tangent line ([`BevelRunOutBound`]), and
//! the walk along that line over the face the bead rides ([`BevelFaceRunOut`]). Creating it and
//! measuring land with the core's packages as `impl` blocks.

use std::collections::{HashMap, HashSet, VecDeque};
use std::rc::Rc;
use std::sync::Arc;

use indexmap::IndexSet;
use polygon_mesh::{Mesh, MeshEdgeGraph};

use vector_math::Vector3;

use super::BevelSurfaceWalk;
use crate::topology::{EdgeConvexity, FeatureEdgeSet};

/// C# `BevelRunOutBound`: what stopped a run's tangent line, which is what decides how the bead
/// ends there.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum BevelRunOutBound {
    /// The mesh ran out - an open rim, or a non-manifold junction. Nothing to run out onto.
    #[default]
    OpenBoundary = 0,
    /// The face ENDS: past this edge the surface falls away, so the bead's fill tapers to nothing
    /// on the edge itself. The cube's front corner edge on "5mm Bevel Around Center.mcx", where
    /// both of the left beads' tangent lines converge onto the 45 degree knife tip.
    ConvexEdge,
    /// A WALL rises: the bead runs out flat against it, which is what the control does where the
    /// four beads eat the notch's unselected centre crease.
    ConcaveEdge,
}

/// C# `BevelFaceRunOut` (a sealed class): the part's surface read once, at the same "is this still
/// one surface" tolerance `BevelFaceFit` walks with, so a run-out and a width refusal never
/// disagree about where a face ends. Shared by every bead of a rebuild (`Rc`).
///
/// A crease ends where either of its two faces ends. Where the face that ends is the OTHER one,
/// the bead does not stop with the crease: it rides the surviving face until its own tangent line
/// leaves it. So this answers a LINE question - how far along does the tangent line stay on this
/// face, and what is at the end of it - started on the tangent line and not on the crease, which
/// are parallel but offset by the setback within the face.
#[derive(Debug)]
pub struct BevelFaceRunOut {
    /// The part.
    pub mesh: Arc<Mesh>,
    /// Walking the surface, stopped at `surface_edges`.
    pub walk: BevelSurfaceWalk,
    /// Which mesh edges end a face, at the run-out's tolerance - shared with `walk`.
    pub surface_edges: Rc<IndexSet<i32>>,
    /// Each of those edges' convexity, by mesh edge index.
    pub convexity_by_mesh_edge: HashMap<i32, EdgeConvexity>,
    /// `mesh`'s edge graph.
    pub graph: Rc<MeshEdgeGraph>,
    /// Which mesh edge joins a pair of vertices, keyed `(low, high)`. The graph indexes edges but
    /// not the pair, and the flood has only the pair to go on.
    pub edges_by_vertex_pair: HashMap<(i32, i32), i32>,
    /// The diagonal of the part's own bounding box, which is the scale a POSITION tolerance about
    /// this mesh belongs to (C#'s `Diagonal` property).
    pub diagonal: f64,
}

/// C# `InsideTheTriangle`: how far inside a triangle's edges a point still counts as in it.
pub const INSIDE_THE_TRIANGLE: f64 = 1e-6;

impl BevelFaceRunOut {
    /// C#'s (private) constructor: the walk over `surface_edges`, the part's diagonal and the
    /// vertex-pair index of every graph edge.
    pub fn new(
        mesh: Arc<Mesh>,
        graph: Rc<MeshEdgeGraph>,
        surface_edges: Rc<IndexSet<i32>>,
        convexity_by_mesh_edge: HashMap<i32, EdgeConvexity>,
    ) -> Self {
        let walk = BevelSurfaceWalk::new(mesh.clone(), graph.clone(), surface_edges.clone());
        let diagonal = mesh.get_axis_aligned_bounding_box().size().length();

        let mut edges_by_vertex_pair = HashMap::with_capacity(graph.edge_count().max(0) as usize);

        for edge in 0..graph.edge_count() {
            let low = graph.get_vertex0(edge).min(graph.get_vertex1(edge));
            let high = graph.get_vertex0(edge).max(graph.get_vertex1(edge));

            edges_by_vertex_pair.insert((low, high), edge);
        }

        BevelFaceRunOut {
            mesh,
            walk,
            surface_edges,
            convexity_by_mesh_edge,
            graph,
            edges_by_vertex_pair,
            diagonal,
        }
    }

    /// C# `Create`: reads the part's surface once at `surface_tolerance_degrees` (C#'s default is
    /// `bevel_face_fit::DEFAULT_SURFACE_TOLERANCE_DEGREES`), so a run-out and a width refusal never
    /// disagree about where a face ends. `graph` is the mesh's edge graph where the caller already
    /// has one; `None` builds it.
    pub fn create(
        mesh: Arc<Mesh>,
        graph: Option<Rc<MeshEdgeGraph>>,
        surface_tolerance_degrees: f64,
    ) -> Self {
        let graph = graph.unwrap_or_else(|| Rc::new(mesh.get_mesh_edge_graph()));

        let surfaces = FeatureEdgeSet::create_with_graph(&mesh, &graph, surface_tolerance_degrees);
        let mut surface_edges = IndexSet::new();
        let mut convexity = HashMap::new();

        for edge in surfaces.edges() {
            surface_edges.insert(edge.mesh_edge_index);
            convexity.insert(edge.mesh_edge_index, edge.convexity);
        }

        BevelFaceRunOut::new(mesh, graph, Rc::new(surface_edges), convexity)
    }
}

/// C# `BevelFaceRunOut.TryMeasure`'s answer: how far the line stays on the face, what stopped it,
/// and the mesh edge it stopped at (`-1` for none).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct BevelRunOutMeasure {
    /// How far along the direction the line stayed on the face.
    pub distance: f64,
    /// What stopped it.
    pub bound: BevelRunOutBound,
    /// The mesh edge it stopped at, `-1` for none.
    pub bounding_edge: i32,
}

impl BevelFaceRunOut {
    /// C# `TryMeasure`: how far the line from `from` along `direction` stays on the face
    /// `riding_face` belongs to, up to `limit`, and what stops it. `None` where `from` is not on
    /// that face at all - a caller's mistake rather than a shape with no run-out.
    pub fn try_measure(
        &self,
        riding_face: i32,
        from: Vector3,
        direction: Vector3,
        limit: f64,
    ) -> Option<BevelRunOutMeasure> {
        let start_face = self.try_find_start(riding_face, from)?;

        let stop = self
            .walk
            .walk(start_face, from, direction, -1, -1, limit, None);
        let mut bound = BevelRunOutBound::OpenBoundary;

        if stop.blocking_edge >= 0 {
            if let Some(convexity) = self.convexity_by_mesh_edge.get(&stop.blocking_edge) {
                bound = if *convexity == EdgeConvexity::Concave {
                    BevelRunOutBound::ConcaveEdge
                } else {
                    BevelRunOutBound::ConvexEdge
                };
            }
        }

        Some(BevelRunOutMeasure {
            distance: stop.travelled,
            bound,
            bounding_edge: stop.blocking_edge,
        })
    }

    /// C# `TryEdgeEnds`: a mesh edge's two end positions, `None` for an edge out of range or of no
    /// length.
    pub fn try_edge_ends(&self, mesh_edge: i32) -> Option<(Vector3, Vector3)> {
        if mesh_edge < 0 || mesh_edge >= self.graph.edge_count() {
            return None;
        }

        let from = self.position(self.graph.get_vertex0(mesh_edge));
        let to = self.position(self.graph.get_vertex1(mesh_edge));

        if (to - from).length() > 0.0 {
            Some((from, to))
        } else {
            None
        }
    }

    /// C# `TryEdgeFaceNormals`: the unit normals of a manifold mesh edge's two faces.
    pub fn try_edge_face_normals(&self, mesh_edge: i32) -> Option<(Vector3, Vector3)> {
        if mesh_edge < 0
            || mesh_edge >= self.graph.edge_count()
            || self.graph.get_face_count(mesh_edge) != 2
        {
            return None;
        }

        let normal0 = self.try_face_normal(self.graph.get_face(mesh_edge, 0))?;
        let normal1 = self.try_face_normal(self.graph.get_face(mesh_edge, 1))?;

        Some((normal0, normal1))
    }

    /// C# `TryFaceNormal`: a face's unit normal from its corners, `None` for a degenerate face.
    #[allow(clippy::neg_cmp_op_on_partial_ord)]
    pub fn try_face_normal(&self, face: i32) -> Option<Vector3> {
        let (a, b, c) = self.corners(face);
        let normal = (b - a).cross(c - a);

        if !(normal.length() > 0.0) {
            return None;
        }

        Some(normal.get_normal())
    }

    /// C# `TryFindStart`: the face of `riding_face`'s surface (flooded across every edge that is
    /// not a surface edge) that holds `from`.
    pub fn try_find_start(&self, riding_face: i32, from: Vector3) -> Option<i32> {
        let mut seen = HashSet::from([riding_face]);
        let mut queue = VecDeque::from([riding_face]);

        while let Some(face) = queue.pop_front() {
            if self.contains(face, from) {
                return Some(face);
            }

            let mesh_face = self.mesh.faces[face as usize];

            for side in 0..3 {
                let (vertex_a, vertex_b) = match side {
                    0 => (mesh_face.v0, mesh_face.v1),
                    1 => (mesh_face.v1, mesh_face.v2),
                    _ => (mesh_face.v2, mesh_face.v0),
                };

                let Some(edge) = self.try_edge_between(vertex_a, vertex_b) else {
                    continue;
                };

                if self.surface_edges.contains(&edge) || self.graph.get_face_count(edge) != 2 {
                    continue;
                }

                let first = self.graph.get_face(edge, 0);
                let across = if first == face {
                    self.graph.get_face(edge, 1)
                } else {
                    first
                };

                if seen.insert(across) {
                    queue.push_back(across);
                }
            }
        }

        None
    }

    /// C# `TryEdgeBetween`: the mesh edge joining two vertices.
    pub fn try_edge_between(&self, vertex_a: i32, vertex_b: i32) -> Option<i32> {
        self.edges_by_vertex_pair
            .get(&(vertex_a.min(vertex_b), vertex_a.max(vertex_b)))
            .copied()
    }

    /// C# `Contains`: whether `point`, dropped onto `face`'s plane, lies inside the triangle to
    /// [`INSIDE_THE_TRIANGLE`] of its doubled area.
    #[allow(clippy::neg_cmp_op_on_partial_ord)]
    pub fn contains(&self, face: i32, point: Vector3) -> bool {
        let (a, b, c) = self.corners(face);

        let mut normal = (b - a).cross(c - a);
        let twice_area = normal.length();

        if !(twice_area > 0.0) {
            return false;
        }

        normal /= twice_area;

        let on_the_plane = point - (normal * (point - a).dot(normal));
        let slack = -INSIDE_THE_TRIANGLE * twice_area;

        (b - a).cross(on_the_plane - a).dot(normal) >= slack
            && (c - b).cross(on_the_plane - b).dot(normal) >= slack
            && (a - c).cross(on_the_plane - c).dot(normal) >= slack
    }

    fn position(&self, vertex: i32) -> Vector3 {
        Vector3::from_vector3_float(self.mesh.vertices[vertex as usize])
    }

    fn corners(&self, face: i32) -> (Vector3, Vector3, Vector3) {
        let mesh_face = self.mesh.faces[face as usize];

        (
            self.position(mesh_face.v0),
            self.position(mesh_face.v1),
            self.position(mesh_face.v2),
        )
    }
}
