//! A port of MIConvexHull's 3D convex hull (`ConvexHull.Create`), the path agg-sharp's
//! `MeshConvexHull` uses. Source: github.com/DesignEngrLab/MIConvexHull at d5f3a77 (2019-10-19),
//! the code NuGet `MIConvexHull 1.1.19.1019` (agg-sharp's `PolygonMesh.csproj`) was built from;
//! its `FractionalNegligibleVolume` is still 1e-8 as in the shipped DLL. MIT, Copyright (c) 2010
//! David Sehnal, Matthew Campbell (see THIRD_PARTY_NOTICES.md).
//!
//! Only what the 3D, non-lifted path runs is ported: the 2D hull, Delaunay/Voronoi lifting and
//! the general N-D normal are not. The quickhull keeps C#'s pooling, list orders and seeded
//! `Random(1)` so the same input selects the same hull vertices and emits the same faces in the
//! same order. Neighbors: `data_model` (faces, connectors, lists, pools), `math_helper` (planes,
//! volumes), `initialize` (bounding box points and the initial simplex), `main_loop` (cone
//! building); .NET's seeded `Random` is `vector_math::dotnet_random`.

mod data_model;
mod initialize;
mod main_loop;
mod math_helper;

use std::collections::HashSet;

use data_model::{ConnectorList, DeferredFace, FaceConnector, FaceInternal, FaceList};
use math_helper::DIM;

/// Where C# throws (`ConvexHullGenerationException`, or an index/null exception inside the
/// algorithm), which `ConvexHull.Create` turns into a null result.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct HullFailed;

/// `ConvexHull.Create<TVertex, TFace>(data, tolerance)` for 3D points: the hull faces as
/// triples of indices into `points`, wound as MIConvexHull returns them (flipped faces have
/// vertices 0 and 2 swapped), in its face order. `None` where C#'s `Result` is null.
pub(crate) fn create(points: &[[f64; 3]], tolerance: f64) -> Option<Vec<[usize; 3]>> {
    let mut hull = ConvexHullAlgorithm::new(points, tolerance).ok()?;
    hull.get_convex_hull().ok()?;
    Some(hull.get_convex_faces())
}

/// `ConvexHullAlgorithm`, its fields named as in C#.
pub(super) struct ConvexHullAlgorithm {
    plane_distance_tolerance: f64,
    positions: Vec<f64>,
    vertex_visited: Vec<bool>,
    number_of_vertices: usize,
    face_pool: Vec<FaceInternal>,
    affected_face_flags: Vec<bool>,
    unprocessed_faces: FaceList,
    convex_faces: Vec<usize>,
    current_vertex: usize,
    max_distance: f64,
    furthest_vertex: usize,
    inside_point: [f64; DIM],
    update_buffer: [usize; DIM],
    update_indices: [usize; DIM],
    traverse_stack: Vec<usize>,
    /// Vertex buffers (`IndexBuffer`s) by id; faces and the fields below hold ids.
    buffers: Vec<Vec<i32>>,
    empty_buffer: usize,
    beyond_buffer: usize,
    affected_face_buffer: Vec<usize>,
    cone_face_buffer: Vec<DeferredFace>,
    singular_vertices: HashSet<usize>,
    connector_table: Vec<ConnectorList>,
    connectors: Vec<FaceConnector>,
    bounding_box_points: Vec<i32>,
    minima: [f64; DIM],
    maxima: [f64; DIM],
    // ObjectManager
    connector_stack: Option<usize>,
    empty_buffer_stack: Vec<usize>,
    free_face_indices: Vec<usize>,
}

impl ConvexHullAlgorithm {
    /// The constructor: every input is 3D, so `DetermineDimension` is 3.
    fn new(points: &[[f64; 3]], tolerance: f64) -> Result<Self, HullFailed> {
        let number_of_vertices = points.len();
        if number_of_vertices <= DIM {
            // NotEnoughVerticesForDimension
            return Err(HullFailed);
        }
        let mut positions = Vec::with_capacity(number_of_vertices * DIM);
        for p in points {
            positions.extend_from_slice(p);
        }
        Ok(ConvexHullAlgorithm {
            plane_distance_tolerance: tolerance,
            positions,
            vertex_visited: vec![false; number_of_vertices],
            number_of_vertices,
            face_pool: Vec::new(),
            affected_face_flags: Vec::new(),
            unprocessed_faces: FaceList::default(),
            convex_faces: Vec::new(),
            current_vertex: 0,
            max_distance: 0.0,
            furthest_vertex: 0,
            inside_point: [0.0; DIM],
            update_buffer: [0; DIM],
            update_indices: [0; DIM],
            traverse_stack: Vec::new(),
            // EmptyBuffer (0) and BeyondBuffer (1)
            buffers: vec![Vec::new(), Vec::new()],
            empty_buffer: 0,
            beyond_buffer: 1,
            affected_face_buffer: Vec::new(),
            cone_face_buffer: Vec::new(),
            singular_vertices: HashSet::new(),
            connector_table: vec![
                ConnectorList::default();
                data_model::CONNECTOR_TABLE_SIZE as usize
            ],
            connectors: Vec::new(),
            bounding_box_points: Vec::new(),
            minima: [0.0; DIM],
            maxima: [0.0; DIM],
            connector_stack: None,
            empty_buffer_stack: Vec::new(),
            free_face_indices: Vec::new(),
        })
    }

    /// `GetConvexHull`: the quickhull main loop.
    fn get_convex_hull(&mut self) -> Result<(), HullFailed> {
        // accessing a 1D array is quicker than a jagged array (SerializeVerticesToPositions is
        // done in `new`)
        // next the bounding box extremes are found. This is used to shift, scale and find the
        // starting simplex.
        self.find_bounding_box_points();
        // the positions are shifted to avoid divide by zero problems
        self.shift_and_scale_positions();
        // Find the (dimension+1) initial points and create the simplexes.
        self.create_initial_simplex()?;

        // Now, the main loop. These initial faces of a simplex are replaced and expanded
        // outwards to make the convex hull and faces.
        while let Some(current_face) = self.unprocessed_faces.first {
            self.current_vertex = self.face_pool[current_face].furthest_vertex;
            // The affected faces get tagged
            self.tag_affected_faces(current_face)?;
            // Create the cone from the currentVertex and the affected faces horizon.
            if !self.singular_vertices.contains(&self.current_vertex) && self.create_cone()? {
                self.commit_cone();
            } else {
                self.handle_singular();
            }
            // Need to reset the tags
            for i in 0..self.affected_face_buffer.len() {
                let f = self.affected_face_buffer[i];
                self.affected_face_flags[f] = false;
            }
        }
        Ok(())
    }

    /// `GetConvexFaces`: each hull face's vertices, flipped faces with vertices 0 and 2 swapped.
    fn get_convex_faces(&self) -> Vec<[usize; 3]> {
        self.convex_faces
            .iter()
            .map(|&f| {
                let face = &self.face_pool[f];
                let mut v = face.vertices.map(|i| i as usize);
                if face.is_normal_flipped {
                    v.swap(0, DIM - 1);
                }
                v
            })
            .collect()
    }

    // ObjectManager

    /// `DepositFace`.
    fn deposit_face(&mut self, face_index: usize) {
        self.face_pool[face_index].adjacent_faces = [-1; DIM];
        self.free_face_indices.push(face_index);
    }

    /// `GetFace`: a freed face index, else a new face.
    fn get_face(&mut self) -> usize {
        if let Some(index) = self.free_face_indices.pop() {
            return index;
        }
        let index = self.face_pool.len();
        let buffer = self.get_vertex_buffer();
        self.face_pool.push(FaceInternal::new(index, buffer));
        self.affected_face_flags.push(false);
        index
    }

    /// `DepositConnector`.
    fn deposit_connector(&mut self, connector: usize) {
        self.connectors[connector].next = self.connector_stack;
        self.connector_stack = Some(connector);
    }

    /// `GetConnector`.
    fn get_connector(&mut self) -> usize {
        match self.connector_stack {
            None => {
                self.connectors.push(FaceConnector::default());
                self.connectors.len() - 1
            }
            Some(ret) => {
                self.connector_stack = self.connectors[ret].next;
                self.connectors[ret].next = None;
                ret
            }
        }
    }

    /// `DepositVertexBuffer`.
    fn deposit_vertex_buffer(&mut self, buffer: usize) {
        self.buffers[buffer].clear();
        self.empty_buffer_stack.push(buffer);
    }

    /// `GetVertexBuffer`.
    fn get_vertex_buffer(&mut self) -> usize {
        match self.empty_buffer_stack.pop() {
            Some(b) => b,
            None => {
                self.buffers.push(Vec::new());
                self.buffers.len() - 1
            }
        }
    }
}

#[cfg(test)]
mod tests;
