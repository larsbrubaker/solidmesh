//! How far a run may carry on PAST the end of its crease, and what stops it there
//! (`Bevel/BevelFaceRunOut.cs`): what stopped a run's tangent line ([`BevelRunOutBound`]), and
//! the walk along that line over the face the bead rides ([`BevelFaceRunOut`]). Creating it and
//! measuring land with the core's packages as `impl` blocks.

use std::collections::HashMap;
use std::rc::Rc;
use std::sync::Arc;

use indexmap::IndexSet;
use polygon_mesh::{Mesh, MeshEdgeGraph};

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
