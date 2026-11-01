//! The logical surface a face belongs to, and every face's surface label at once (fenced or not).
//! Flooded over a
//! [`MeshEdgeGraph`] with
//! [`FeatureEdge::try_classify`]; cached per mesh by [`super::FaceRegionCache`].

use std::collections::VecDeque;

use polygon_mesh::{Mesh, MeshEdgeGraph};

use super::error::TopologyError;
use super::feature_edge::{EdgeConvexity, FeatureEdge};

const EDGES_PER_FACE: usize = 3;

/// The "logical surface" a face belongs to: everything reachable from it without crossing an
/// edge the surface visibly turns at, together with the feature edges that bound it. Selecting a
/// face group is how a user says "bevel the top of this box" (four rim edges) or "fillet where
/// this boss meets the plate" (one circular loop).
///
/// Smoothness is about how far the surface turns, not which way it folds: the fill compares
/// `|turn_angle_degrees|` against the curve tolerance, so a gentle concave blend is crossed just
/// as a gentle convex one is.
///
/// The fill only crosses manifold edges. An open rim or a non-manifold edge stops it but is not a
/// feature edge, so those are reported separately in `open_boundary_mesh_edges` rather than
/// dropped silently, which lets a caller warn that part of the boundary cannot be beveled.
///
/// Like [`super::FeatureEdgeSet`], a group is a snapshot of one state of one mesh.
#[derive(Debug, Clone, PartialEq)]
pub struct FaceGroup {
    faces: Vec<i32>,
    boundary_edges: Vec<FeatureEdge>,
    open_boundary_mesh_edges: Vec<i32>,
    seed_face: i32,
    curve_tolerance_degrees: f64,
}

impl FaceGroup {
    /// The default curve tolerance in degrees, the plan's resolved value: Blender's auto-smooth
    /// default. At this tolerance a genuinely flat face collects its coplanar region and a
    /// tessellated cylinder wall collects the whole wall.
    pub const DEFAULT_CURVE_TOLERANCE_DEGREES: f64 = 30.0;

    /// The faces in the group, in ascending face index order. Always contains the seed.
    pub fn faces(&self) -> &[i32] {
        &self.faces
    }

    /// The feature edges bounding the group - the manifold edges the fill refused to cross - in
    /// ascending mesh edge order. Classified at the group's own curve tolerance, so none is
    /// `Flat` - except where a FENCE bounded the group ([`Self::create_fenced`]).
    pub fn boundary_edges(&self) -> &[FeatureEdge] {
        &self.boundary_edges
    }

    /// The mesh edges bounding the group that are not bevelable: boundary edges of an open mesh
    /// and non-manifold edges, as indices into the edge graph the group was built from.
    pub fn open_boundary_mesh_edges(&self) -> &[i32] {
        &self.open_boundary_mesh_edges
    }

    /// The face the fill started from.
    pub fn seed_face(&self) -> i32 {
        self.seed_face
    }

    /// How far the surface was allowed to turn and still count as the same surface, in degrees.
    pub fn curve_tolerance_degrees(&self) -> f64 {
        self.curve_tolerance_degrees
    }

    /// Flood fills from a face at the default curve tolerance.
    pub fn create(mesh: &Mesh, seed_face_index: i32) -> Result<FaceGroup, TopologyError> {
        Self::create_with_tolerance(mesh, seed_face_index, Self::DEFAULT_CURVE_TOLERANCE_DEGREES)
    }

    /// Flood fills from a face across every edge the surface turns less than
    /// `curve_tolerance_degrees` at.
    pub fn create_with_tolerance(
        mesh: &Mesh,
        seed_face_index: i32,
        curve_tolerance_degrees: f64,
    ) -> Result<FaceGroup, TopologyError> {
        Self::create_with_graph(
            mesh,
            &mesh.get_mesh_edge_graph(),
            seed_face_index,
            curve_tolerance_degrees,
        )
    }

    /// Flood fills using an edge graph the caller already has. That graph must be `mesh`'s own:
    /// nothing checks. Errors when `seed_face_index` is not a face of `mesh`.
    pub fn create_with_graph(
        mesh: &Mesh,
        graph: &MeshEdgeGraph,
        seed_face_index: i32,
        curve_tolerance_degrees: f64,
    ) -> Result<FaceGroup, TopologyError> {
        Self::create_fenced(mesh, graph, seed_face_index, curve_tolerance_degrees, None)
    }

    /// The same flood with a FENCE round part of the mesh: the fill never enters a barrier face,
    /// and the edge between a face of the group and a barrier face is a boundary edge of the
    /// group exactly as a crease is. `None` (or empty) is no fence and answers exactly what the
    /// unfenced overload does.
    ///
    /// This is for a bevel over a bevel. A blend's facets turn LESS than a pick's flat angle, so
    /// an unfenced flood crosses the fillet the way it crosses a tessellated wall; fencing the
    /// blend's own faces is how the second pass reads the SAME region on the built mesh that it
    /// read on the source. A fenced edge keeps whatever convexity `try_classify` gives it, flat
    /// included, because both sides of it are real faces.
    pub fn create_fenced(
        mesh: &Mesh,
        graph: &MeshEdgeGraph,
        seed_face_index: i32,
        curve_tolerance_degrees: f64,
        barrier_faces: Option<&[i32]>,
    ) -> Result<FaceGroup, TopologyError> {
        let face_count = mesh.faces.len();
        if seed_face_index < 0 || seed_face_index as usize >= face_count {
            return Err(TopologyError::FaceIndexOutOfRange {
                parameter: "seedFaceIndex",
                index: seed_face_index,
                face_count,
            });
        }

        let face_edges = build_face_to_edge_map(face_count, graph);

        let mut fenced: Option<Vec<bool>> = None;

        if let Some(barriers) = barrier_faces.filter(|b| !b.is_empty()) {
            let mut fence = vec![false; face_count];

            for &barrier in barriers {
                if barrier >= 0 && (barrier as usize) < fence.len() {
                    fence[barrier as usize] = true;
                }
            }

            // The seed is always in its own group - a caller that seeds inside the fence is
            // asking for the region round THAT face.
            fence[seed_face_index as usize] = false;
            fenced = Some(fence);
        }

        let mut in_group = vec![false; face_count];
        let mut edge_seen = vec![false; graph.edge_count().max(0) as usize];
        let mut faces = Vec::new();
        let mut boundary_edges = Vec::new();
        let mut open_boundary_mesh_edges = Vec::new();

        let mut to_visit = VecDeque::new();
        in_group[seed_face_index as usize] = true;
        to_visit.push_back(seed_face_index);

        while let Some(face_index) = to_visit.pop_front() {
            faces.push(face_index);

            for slot in 0..EDGES_PER_FACE {
                let edge_index = face_edges[(face_index as usize * EDGES_PER_FACE) + slot];
                if edge_index < 0 {
                    continue;
                }

                let seen = &mut edge_seen[edge_index as usize];

                let Some(edge) =
                    FeatureEdge::try_classify(mesh, graph, edge_index, curve_tolerance_degrees)
                else {
                    // an open rim or a non-manifold junction: the fill cannot cross it and a
                    // bevel cannot use it
                    if !*seen {
                        *seen = true;
                        open_boundary_mesh_edges.push(edge_index);
                    }

                    continue;
                };

                if edge.convexity != EdgeConvexity::Flat {
                    if !*seen {
                        *seen = true;
                        boundary_edges.push(edge);
                    }

                    continue;
                }

                let across_face = if edge.face0 == face_index {
                    edge.face1
                } else {
                    edge.face0
                };

                if fenced.as_ref().is_some_and(|f| f[across_face as usize]) {
                    // The fence stops the fill where a crease would, and bounds the group the
                    // same way.
                    if !*seen {
                        *seen = true;
                        boundary_edges.push(edge);
                    }

                    continue;
                }

                if !in_group[across_face as usize] {
                    in_group[across_face as usize] = true;
                    to_visit.push_back(across_face);
                }
            }
        }

        // Mesh order rather than discovery order, so the same mesh and seed always describe the
        // group the same way - callers persist these and compare them across rebuilds. Indices
        // are unique, so the unstable sort has no ties to order.
        faces.sort_unstable();
        boundary_edges.sort_by_key(|e: &FeatureEdge| e.mesh_edge_index);
        open_boundary_mesh_edges.sort_unstable();

        Ok(FaceGroup {
            faces,
            boundary_edges,
            open_boundary_mesh_edges,
            seed_face: seed_face_index,
            curve_tolerance_degrees,
        })
    }

    /// Every face's logical surface at one tolerance, as an id per face: the same flood
    /// [`Self::create`] makes, run from every face at once so two faces can be asked whether they
    /// are the same surface without either being a seed. Ids are dense from zero in ascending
    /// face order and mean nothing across meshes, tolerances or rebuilds.
    ///
    /// Edges that cannot be classified - an open rim, a non-manifold junction - are not crossed,
    /// exactly as the group fill does not cross them.
    pub fn label_surfaces(
        mesh: &Mesh,
        graph: &MeshEdgeGraph,
        curve_tolerance_degrees: f64,
    ) -> Vec<i32> {
        Self::label_surfaces_fenced(mesh, graph, curve_tolerance_degrees, None)
    }

    /// [`Self::label_surfaces`] with a FENCE: every face outside the fence gets the label of the
    /// group [`Self::create_fenced`] floods from it (the fill never enters a barrier face), and
    /// every barrier face gets a label of its own. A flood seeded on a barrier face lifts the
    /// fence for that seed alone and so runs out into its neighbours' surface; that group is the
    /// barrier face's answer and nobody else's, so in a labelling, where every face has exactly
    /// one id, the barrier face stands alone. `None` (or empty) is no fence and labels exactly
    /// as [`Self::label_surfaces`] does.
    ///
    /// Ids are dense from zero in ascending face order, a barrier face's id where its index
    /// comes. One pass: every face is visited once and each edge classified once per face beside
    /// it, where flooding a group a seed at a time rebuilds the face-to-edge map per group.
    pub fn label_surfaces_fenced(
        mesh: &Mesh,
        graph: &MeshEdgeGraph,
        curve_tolerance_degrees: f64,
        barrier_faces: Option<&[i32]>,
    ) -> Vec<i32> {
        let face_count = mesh.faces.len();
        let face_edges = build_face_to_edge_map(face_count, graph);
        let mut surface_of_face = vec![-1; face_count];

        let mut fence = vec![false; face_count];
        for &barrier in barrier_faces.unwrap_or(&[]) {
            if barrier >= 0 && (barrier as usize) < face_count {
                fence[barrier as usize] = true;
            }
        }

        let mut to_visit = VecDeque::new();
        let mut surfaces = 0;

        for seed in 0..surface_of_face.len() {
            if surface_of_face[seed] >= 0 {
                continue;
            }

            let surface = surfaces;
            surfaces += 1;
            surface_of_face[seed] = surface;

            // A barrier face is a surface of its own: nothing floods out of it.
            if fence[seed] {
                continue;
            }

            to_visit.push_back(seed as i32);

            while let Some(face_index) = to_visit.pop_front() {
                for slot in 0..EDGES_PER_FACE {
                    let edge_index = face_edges[(face_index as usize * EDGES_PER_FACE) + slot];

                    if edge_index < 0 {
                        continue;
                    }

                    let Some(edge) =
                        FeatureEdge::try_classify(mesh, graph, edge_index, curve_tolerance_degrees)
                    else {
                        continue;
                    };

                    if edge.convexity != EdgeConvexity::Flat {
                        continue;
                    }

                    let across_face = if edge.face0 == face_index {
                        edge.face1
                    } else {
                        edge.face0
                    };

                    if fence[across_face as usize] {
                        continue;
                    }

                    if surface_of_face[across_face as usize] < 0 {
                        surface_of_face[across_face as usize] = surface;
                        to_visit.push_back(across_face);
                    }
                }
            }
        }

        surface_of_face
    }
}

/// The other half of the edge graph's adjacency: which edges bound each face. A triangle has
/// three edges, so this is a fixed stride array; unused slots hold -1, which is what a degenerate
/// face repeating a vertex leaves behind.
fn build_face_to_edge_map(face_count: usize, graph: &MeshEdgeGraph) -> Vec<i32> {
    let mut face_edges = vec![-1; face_count * EDGES_PER_FACE];

    let mut filled = vec![0usize; face_count];
    for edge_index in 0..graph.edge_count() {
        for &face_index in graph.get_faces(edge_index) {
            let face = face_index as usize;
            if filled[face] < EDGES_PER_FACE {
                face_edges[(face * EDGES_PER_FACE) + filled[face]] = edge_index;
                filled[face] += 1;
            }
        }
    }

    face_edges
}
