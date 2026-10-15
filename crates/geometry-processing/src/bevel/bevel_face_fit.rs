//! Whether a group's bevel has FACE to land on ([`BevelFaceFit`]): the plan's other size limit -
//! not how far the tool runs along the chain but how far it reaches across the two faces beside it,
//! measured by walking the surface ([`BevelSurfaceWalk`]) square to each selected edge. Creating
//! the check, the walk per sample and the exemptions land with the core's packages as `impl`
//! blocks.

use std::cell::RefCell;
use std::collections::{HashMap, HashSet};
use std::rc::Rc;
use std::sync::Arc;

use indexmap::IndexSet;
use polygon_mesh::{Mesh, MeshEdgeGraph};
use vector_math::ieee_math::max_f64;
use vector_math::math_helper::degrees_to_radians;
use vector_math::Vector3;

use super::bevel_edge_tool_builder::profile_wedge_angle_degrees;
use super::{BevelEndCap, BevelSurfaceWalk};
use crate::topology::{
    BevelEdgeGroupResolution, EdgeConvexity, FaceGroup, FeatureEdge, FeatureEdgeSet,
};

/// One group's canonical free-end caps, by `(vertex, mesh edge)` of the run end they cap.
pub type BevelFaceFitEndCaps = HashMap<(i32, i32), BevelEndCap>;

/// One rebuild's face-width check. Every selected edge sets the cut back from itself along each of
/// its two faces; the walk measures how much face there IS to set back into, and a group whose
/// setback does not fit is refused with the largest size that would have.
#[derive(Debug)]
pub struct BevelFaceFit {
    /// The welded working mesh the groups were resolved against.
    pub mesh: Arc<Mesh>,
    /// `mesh`'s edge graph.
    pub graph: Rc<MeshEdgeGraph>,
    /// Every group of the rebuild, in the operation's own order. Unresolved ones are ignored.
    pub resolutions: Rc<Vec<BevelEdgeGroupResolution>>,
    /// True for a fillet's radius, false for a chamfer's setback.
    pub round_profile: bool,
    /// The operation's one bevel size, which every group of it is measured at.
    pub size: f64,
    /// Which mesh edges end a face, by mesh edge index, at the walk's own tolerance - the set
    /// `walk` stops at, shared with it.
    pub surface_edges: Rc<IndexSet<i32>>,
    /// The concave ones of those, which are the only stops a run's end cap can be argued to cover.
    pub concave_surface_edges: HashSet<i32>,
    /// Every selected mesh edge at each of its two vertices, so a run's END can be told from a
    /// corner two selected runs turn. A set rather than a count because one edge picked into two
    /// groups is one edge.
    pub selected_edges_at_vertex: HashMap<i32, HashSet<i32>>,
    /// Canonical free-end caps, read only for groups whose face width needs a cap exemption: by the
    /// group's index in `resolutions`, then by `(vertex, mesh edge)`. Filled lazily the first time
    /// a group asks, hence the `RefCell`.
    pub free_end_caps: RefCell<HashMap<usize, BevelFaceFitEndCaps>>,
    /// How far off a plane a vertex may lie and still be called ON it, scaled to the part because
    /// mesh vertices are float.
    pub plane_tolerance: f64,
    /// Which mesh edges are themselves being cut, and by how much.
    pub setbacks: HashMap<i32, f64>,
    /// Walking the surface to find where a face ends - the measurement, split from the policy
    /// here.
    pub walk: BevelSurfaceWalk,
    /// The largest of `setbacks`.
    pub largest_setback: f64,
}

/// The "is this still one surface" reading a face ends at.
pub const DEFAULT_SURFACE_TOLERANCE_DEGREES: f64 = FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES;

/// Where along each selected edge the width is sampled.
pub const SAMPLE_FRACTIONS: [f64; 3] = [0.1, 0.5, 0.9];

/// Below this a wedge builds no tool.
pub const MINIMUM_WEDGE_ANGLE_DEGREES: f64 = 1.0;

/// Above this a wedge builds no tool.
pub const MAXIMUM_WEDGE_ANGLE_DEGREES: f64 = 179.0;

/// The plane tolerance as a fraction of the part's diagonal.
pub const PLANE_TOLERANCE_OF_THE_PARTICULAR_PART: f64 = 1e-5;

/// Two cap planes within half a degree are the same.
pub fn same_cap_plane_agreement() -> f64 {
    degrees_to_radians(0.5).cos()
}

impl BevelFaceFit {
    /// A check from its readings, with an empty free-end-cap memo.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        mesh: Arc<Mesh>,
        graph: Rc<MeshEdgeGraph>,
        resolutions: Rc<Vec<BevelEdgeGroupResolution>>,
        round_profile: bool,
        size: f64,
        surface_edges: Rc<IndexSet<i32>>,
        concave_surface_edges: HashSet<i32>,
        selected_edges_at_vertex: HashMap<i32, HashSet<i32>>,
        plane_tolerance: f64,
        setbacks: HashMap<i32, f64>,
        largest_setback: f64,
    ) -> Self {
        let walk = BevelSurfaceWalk::new(mesh.clone(), graph.clone(), surface_edges.clone());

        BevelFaceFit {
            mesh,
            graph,
            resolutions,
            round_profile,
            size,
            surface_edges,
            concave_surface_edges,
            selected_edges_at_vertex,
            free_end_caps: RefCell::new(HashMap::new()),
            plane_tolerance,
            setbacks,
            walk,
            largest_setback,
        }
    }

    /// Reads the part's surface at `surface_tolerance_degrees` (normally
    /// [`DEFAULT_SURFACE_TOLERANCE_DEGREES`]) and every resolved group's setbacks. `graph` is the
    /// mesh's edge graph where the caller already has one; `None` builds it.
    pub fn create(
        mesh: Arc<Mesh>,
        resolutions: Rc<Vec<BevelEdgeGroupResolution>>,
        round_profile: bool,
        size: f64,
        surface_tolerance_degrees: f64,
        graph: Option<Rc<MeshEdgeGraph>>,
    ) -> Self {
        let graph = graph.unwrap_or_else(|| Rc::new(mesh.get_mesh_edge_graph()));

        let mut surface_edges = IndexSet::new();
        let mut concave_surface_edges = HashSet::new();

        // Built here rather than shared with the resolver's: the resolver's sets are one per
        // group's OWN picking tolerance, and this one is at the "is this still one surface"
        // reading a face ends at.
        let surfaces = FeatureEdgeSet::create_with_graph(&mesh, &graph, surface_tolerance_degrees);

        for edge in surfaces.edges() {
            surface_edges.insert(edge.mesh_edge_index);

            if edge.convexity == EdgeConvexity::Concave {
                concave_surface_edges.insert(edge.mesh_edge_index);
            }
        }

        // Every group's reservation, not just the group being checked: the all-edges gesture
        // makes one group per edge.
        let mut setbacks = HashMap::new();
        let mut selected_edges_at_vertex = HashMap::new();
        let mut largest_setback = 0.0;

        for resolution in resolutions.iter() {
            if !resolution.is_resolved() {
                continue;
            }

            for edge in &resolution.edges {
                // Recorded before the setback is, and whether or not one can be computed: an edge
                // no tool could be built in still occupies its vertices.
                add_selected_edge_at(
                    &mut selected_edges_at_vertex,
                    edge.start_vertex,
                    edge.mesh_edge_index,
                );
                add_selected_edge_at(
                    &mut selected_edges_at_vertex,
                    edge.end_vertex,
                    edge.mesh_edge_index,
                );

                let Some(setback) = try_get_setback(edge, size, round_profile) else {
                    continue;
                };

                // An edge picked into two groups is reserved once.
                if setbacks.contains_key(&edge.mesh_edge_index) {
                    continue;
                }

                setbacks.insert(edge.mesh_edge_index, setback);
                largest_setback = max_f64(largest_setback, setback);
            }
        }

        let plane_tolerance = mesh.get_axis_aligned_bounding_box().size().length()
            * PLANE_TOLERANCE_OF_THE_PARTICULAR_PART;

        BevelFaceFit::new(
            mesh,
            graph,
            resolutions,
            round_profile,
            size,
            Rc::new(surface_edges),
            concave_surface_edges,
            selected_edges_at_vertex,
            plane_tolerance,
            setbacks,
            largest_setback,
        )
    }

    /// Whether a fillet's walk on `face` stopped at a concave, unselected obstruction standing ON
    /// the face's own plane rather than at the face's end - `Some((planar_within, seed_face))`, the
    /// seed being the obstruction's own face across `blocking_edge`. `crossed` is every triangle
    /// the walk crossed.
    pub fn stands_on_the_face_rather_than_ending_it(
        &self,
        face: i32,
        edge: &FeatureEdge,
        blocking_edge: i32,
        crossed: &[i32],
    ) -> Option<(f64, i32)> {
        // A stop at an edge that is itself being bevelled stays a collision between two tools.
        if !self.round_profile
            || blocking_edge < 0
            || !self.concave_surface_edges.contains(&blocking_edge)
            || self.setbacks.contains_key(&blocking_edge)
        {
            return None;
        }

        let normal = self.walk.try_get_face_normal(face)?;
        let plane = normal.dot(edge.start);

        if !self.on_the_plane(normal, plane, edge.end)
            || !self.on_the_plane(
                normal,
                plane,
                self.position(self.graph.get_vertex0(blocking_edge)),
            )
            || !self.on_the_plane(
                normal,
                plane,
                self.position(self.graph.get_vertex1(blocking_edge)),
            )
        {
            return None;
        }

        // And every triangle the walk actually crossed, which the four points above do NOT stand
        // in for: a shallow dome has its rim back on the plane at both ends.
        for &walked in crossed {
            if !self.lies_in_the_plane(walked, normal, plane) {
                return None;
            }
        }

        if self.graph.get_face_count(blocking_edge) != 2 {
            return None;
        }

        // Which of the blocking edge's two faces is the OBSTRUCTION's: the one NOT lying in the
        // obstructed face's own plane. Where the plane cannot tell them apart, the group keeps its
        // refusal.
        let first_face = self.graph.get_face(blocking_edge, 0);
        let second_face = self.graph.get_face(blocking_edge, 1);
        let first_is_the_face = self.lies_in_the_plane(first_face, normal, plane);

        if first_is_the_face == self.lies_in_the_plane(second_face, normal, plane) {
            return None;
        }

        let seed_face = if first_is_the_face {
            second_face
        } else {
            first_face
        };

        Some((self.plane_tolerance, seed_face))
    }

    /// Whether `point` lies on the plane, within the part's plane tolerance.
    pub fn on_the_plane(&self, normal: Vector3, plane_distance: f64, point: Vector3) -> bool {
        (normal.dot(point) - plane_distance).abs() <= self.plane_tolerance
    }

    /// All three of `face`'s corners on the plane.
    pub fn lies_in_the_plane(&self, face: i32, normal: Vector3, plane_distance: f64) -> bool {
        let mesh_face = self.mesh.faces[face as usize];

        self.on_the_plane(normal, plane_distance, self.position(mesh_face.v0))
            && self.on_the_plane(normal, plane_distance, self.position(mesh_face.v1))
            && self.on_the_plane(normal, plane_distance, self.position(mesh_face.v2))
    }

    /// The walk's normal of `face`, zero for a degenerate face.
    pub fn face_normal(&self, face: i32) -> Vector3 {
        self.walk.try_get_face_normal(face).unwrap_or_default()
    }

    fn position(&self, vertex: i32) -> Vector3 {
        Vector3::from_vector3_float(self.mesh.vertices[vertex as usize])
    }
}

/// Records `edge` as selected at `vertex`.
pub fn add_selected_edge_at(
    selected_edges_at_vertex: &mut HashMap<i32, HashSet<i32>>,
    vertex: i32,
    mesh_edge_index: i32,
) {
    selected_edges_at_vertex
        .entry(vertex)
        .or_default()
        .insert(mesh_edge_index);
}

/// How far `edge`'s bevel of `size` sets back along each face, or `None` where no tool could be
/// built (a non-positive size, or a wedge outside 1..179 degrees).
#[allow(clippy::neg_cmp_op_on_partial_ord)]
pub fn try_get_setback(edge: &FeatureEdge, size: f64, round_profile: bool) -> Option<f64> {
    if !(size > 0.0) {
        return None;
    }

    let wedge_angle_degrees = profile_wedge_angle_degrees(edge);

    if wedge_angle_degrees < MINIMUM_WEDGE_ANGLE_DEGREES
        || wedge_angle_degrees > MAXIMUM_WEDGE_ANGLE_DEGREES
    {
        return None;
    }

    let setback = if round_profile {
        size / (degrees_to_radians(wedge_angle_degrees) / 2.0).tan()
    } else {
        size
    };

    if setback > 0.0 && !setback.is_infinite() {
        Some(setback)
    } else {
        None
    }
}
