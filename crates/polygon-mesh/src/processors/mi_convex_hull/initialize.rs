//! The quickhull's initialization (derived from MIConvexHull), 3D non-lifted path: the
//! bounding box points, the shift to positive coordinates, and the initial simplex chosen from
//! seeded random draws (seed 1). Three details decide which vertices start the hull and are
//! pinned by the hull tests: the bounding-box point count is read before the points are
//! padded, the "distance" from the centroid keeps only the last axis's square, and a padding
//! vertex of -1 (no candidate found) fails the hull on first use.

use std::collections::HashSet;

use super::data_model::FaceInternal;
use super::math_helper::{calculate_face_plane, get_vertex_distance, volume_of_simplex, DIM};
use super::{ConvexHullAlgorithm, HullFailed};
use vector_math::seeded_random::SeededRandom;

/// `Constants.FractionalNegligibleVolume` in the 1.1.19.1019 package.
const FRACTIONAL_NEGLIGIBLE_VOLUME: f64 = 1e-8;
/// `Constants.DefaultPlaneDistanceTolerance`.
const DEFAULT_PLANE_DISTANCE_TOLERANCE: f64 = 1e-10;
const NUMBER_OF_INIT_SIMPLICES_TO_TEST: usize = 5;

impl ConvexHullAlgorithm {
    fn get_coordinate(&self, v_index: usize, dimension: usize) -> f64 {
        self.positions[v_index * DIM + dimension]
    }

    /// Every vertex at a minimum or maximum on some axis, distinct, in
    /// first-seen order.
    pub(super) fn find_bounding_box_points(&mut self) {
        let mut points: Vec<i32> = Vec::new();
        for i in 0..DIM {
            let mut min_indices: Vec<i32> = Vec::new();
            let mut max_indices: Vec<i32> = Vec::new();
            let mut min = f64::INFINITY;
            let mut max = f64::NEG_INFINITY;
            for j in 0..self.number_of_vertices {
                let v = self.get_coordinate(j, i);
                if v < min {
                    // you found a better solution than before, clear out the list and store new value
                    min = v;
                    min_indices.clear();
                    min_indices.push(j as i32);
                } else if v == min {
                    //same or almost as good as current limit, so store it
                    min_indices.push(j as i32);
                }
                if v > max {
                    max = v;
                    max_indices.clear();
                    max_indices.push(j as i32);
                } else if v == max {
                    max_indices.push(j as i32);
                }
            }
            self.minima[i] = min;
            self.maxima[i] = max;
            points.extend_from_slice(&min_indices);
            points.extend_from_slice(&max_indices);
        }
        // Distinct, first occurrences in order, through a hash set: a linear scan here is
        // quadratic when a dense part has many vertices on a flat extreme.
        let mut seen = HashSet::with_capacity(points.len());
        points.retain(|&p| seen.insert(p));
        self.bounding_box_points = points;
    }

    /// Shift so the minimum on each axis is half the maximum (no
    /// points at zero, no huge ratios).
    pub(super) fn shift_and_scale_positions(&mut self) {
        let mut shift_amount = [0.0; DIM];
        for (i, shift) in shift_amount.iter_mut().enumerate() {
            *shift = if self.maxima[i] == self.minima[i] {
                0.0
            } else {
                (self.maxima[i] - self.minima[i]) - self.minima[i]
            };
        }
        for (i, p) in self.positions.iter_mut().enumerate() {
            *p += shift_amount[i % DIM];
        }
    }

    /// The first hull faces, from the initial points, with the initial points marked visited;
    /// no vertex is filed beyond them yet.
    pub(super) fn create_initial_faces(&mut self) -> Result<[usize; DIM + 1], HullFailed> {
        let initial_points = self.find_initial_points()?;

        // Create the first faces from (dimension + 1) vertices.
        let mut faces = [0usize; DIM + 1];
        for (i, face_slot) in faces.iter_mut().enumerate() {
            let mut vertices = [0i32; DIM];
            let mut k = 0;
            for (j, &p) in initial_points.iter().enumerate().take(DIM + 1) {
                if i != j {
                    vertices[k] = p;
                    k += 1;
                }
            }
            let new_face = self.get_face();
            vertices.sort_unstable();
            self.face_pool[new_face].vertices = vertices;
            let inside = self.inside_point;
            calculate_face_plane(&self.positions, &mut self.face_pool[new_face], &inside);
            *face_slot = self.face_pool[new_face].index;
        }
        // update the adjacency (check all pairs of faces)
        for i in 0..DIM {
            for j in (i + 1)..(DIM + 1) {
                self.update_adjacency(faces[i], faces[j]);
            }
        }
        for &item in initial_points.iter() {
            self.vertex_visited[item as usize] = true;
        }
        Ok(faces)
    }

    /// Starts filing the vertices beyond an initial face ([`Self::find_beyond_vertices_in`]
    /// over every vertex, in order, then [`Self::end_beyond_vertices`]).
    pub(super) fn begin_beyond_vertices(&mut self) {
        self.max_distance = f64::NEG_INFINITY;
        self.furthest_vertex = 0;
    }

    /// Every unvisited vertex of `range` beyond the face, into its own buffer.
    pub(super) fn find_beyond_vertices_in(&mut self, face: usize, range: std::ops::Range<usize>) {
        let beyond_vertices = self.face_pool[face].vertices_beyond;
        for i in range {
            if self.vertex_visited[i] {
                continue;
            }
            self.is_beyond(face, beyond_vertices, i);
        }
    }

    /// Ends filing an initial face's beyond vertices: it is a hull face when none is beyond it,
    /// else it waits for the main loop.
    pub(super) fn end_beyond_vertices(&mut self, face_index: usize) {
        self.face_pool[face_index].furthest_vertex = self.furthest_vertex;
        let face = &self.face_pool[face_index];
        if self.buffers[face.vertices_beyond].is_empty() {
            // The face is on the hull
            self.convex_faces.push(face.index);
        } else {
            self.unprocessed_faces
                .add(&mut self.face_pool, &self.buffers, face_index);
        }
    }

    /// Dimension + 1 points spanning a non-negligible volume.
    fn find_initial_points(&mut self) -> Result<Vec<i32>, HullFailed> {
        let mut negligible_volume = FRACTIONAL_NEGLIGIBLE_VOLUME;
        for i in 0..DIM {
            negligible_volume *= self.maxima[i] - self.minima[i];
        }
        let mut best_vertex_indices: Option<Vec<i32>> = None;
        // The count is read once, before the padding below.
        let num_bb_points = self.bounding_box_points.len();
        let mut degenerate = true;
        if (DIM + 1) > num_bb_points {
            // if there are fewer bounding box points than what is needed for the simplex (this
            // is quite rare), then add the ones farthest form the centroid of the current
            // bounding box points.
            let padded = self
                .add_n_vertices_farthest_to_centroid(self.bounding_box_points.clone(), DIM + 1)?;
            self.bounding_box_points = padded;
        }
        if (DIM + 1) == num_bb_points {
            // if the number of points is the same then just go with these
            best_vertex_indices = Some(self.bounding_box_points.clone());
            degenerate =
                volume_of_simplex(&self.positions, &self.bounding_box_points) <= negligible_volume;
        } else if (DIM + 1) < num_bb_points {
            // if there are more bounding box points than needed, find a random one that has a
            // large volume.
            let bb = self.bounding_box_points.clone();
            let (best, volume) = self.find_largest_random_simplex(&bb, &bb)?;
            best_vertex_indices = best;
            degenerate = volume <= negligible_volume;
        }
        if degenerate {
            // if it turns out to still be degenerate, then increase the check to include all
            // vertices. this is potentially expensive, but we don't have a choice.
            let bb = self.bounding_box_points.clone();
            let all: Vec<i32> = (0..self.number_of_vertices as i32).collect();
            let (best, volume) = self.find_largest_random_simplex(&bb, &all)?;
            best_vertex_indices = best;
            degenerate = volume <= negligible_volume;
        }
        if degenerate {
            // DegenerateData: the data is all co-planar (or collinear).
            return Err(HullFailed);
        }
        // no simplex was found: the hull fails
        let best = best_vertex_indices.ok_or(HullFailed)?;
        self.inside_point = self.calculate_vertex_centroid(&best)?;
        Ok(best)
    }

    /// Five seeded draws of a base triangle from `bb_points`, each
    /// completed by the farthest of `other_points`; the largest volume wins.
    fn find_largest_random_simplex(
        &self,
        bb_points: &[i32],
        other_points: &[i32],
    ) -> Result<(Option<Vec<i32>>, f64), HullFailed> {
        let mut random = SeededRandom::new(1);
        let mut best_vertex_indices: Option<Vec<i32>> = None;
        let mut max_volume = DEFAULT_PLANE_DISTANCE_TOLERANCE;
        let num_bb_points = bb_points.len();
        for _ in 0..NUMBER_OF_INIT_SIMPLICES_TO_TEST {
            let mut vertex_indices: Vec<i32> = Vec::new();
            // a HashSet<int> that is only added to enumerates in insertion order
            let mut already_chosen_indices: Vec<usize> = Vec::new();
            let num_random_indices = if 2 * DIM <= num_bb_points {
                DIM
            } else {
                num_bb_points - DIM
            };
            while already_chosen_indices.len() < num_random_indices {
                let index = random.next(num_bb_points as i32) as usize;
                if already_chosen_indices.contains(&index) {
                    continue;
                }
                already_chosen_indices.push(index);
            }
            if 2 * DIM <= num_bb_points {
                for &index in already_chosen_indices.iter() {
                    vertex_indices.push(bb_points[index]);
                }
            } else {
                for (j, &p) in bb_points.iter().enumerate() {
                    if !already_chosen_indices.contains(&j) {
                        vertex_indices.push(p);
                    }
                }
            }
            if vertex_indices.iter().any(|&v| v < 0) {
                // a -1 padding vertex has no position: the hull fails
                return Err(HullFailed);
            }
            let mut plane = FaceInternal::new(0, usize::MAX);
            plane.vertices = [vertex_indices[0], vertex_indices[1], vertex_indices[2]];
            calculate_face_plane(&self.positions, &mut plane, &[0.0; DIM]);
            // this next line is the only difference between this subroutine and the one
            let new_vertex = self.find_farthest_point(other_points, &plane);
            if new_vertex == -1 {
                continue;
            }
            vertex_indices.push(new_vertex);
            let volume = volume_of_simplex(&self.positions, &vertex_indices);
            if max_volume < volume {
                max_volume = volume;
                best_vertex_indices = Some(vertex_indices);
            }
        }
        Ok((best_vertex_indices, max_volume))
    }

    /// The vertex farthest from the plane, either side; -1 if none.
    fn find_farthest_point(&self, vertex_indices: &[i32], plane: &FaceInternal) -> i32 {
        let mut max_distance = 0.0;
        let mut farthest_vertex_index = -1;
        for &v in vertex_indices {
            let distance = get_vertex_distance(&self.positions, v as usize, plane).abs();
            if max_distance < distance {
                max_distance = distance;
                farthest_vertex_index = v;
            }
        }
        farthest_vertex_index
    }

    /// Pads `vertex_indices` to `n` vertices, each time adding the one farthest from the
    /// current centroid.
    fn add_n_vertices_farthest_to_centroid(
        &self,
        vertex_indices: Vec<i32>,
        n: usize,
    ) -> Result<Vec<i32>, HullFailed> {
        let mut new_verts_list = vertex_indices;
        while new_verts_list.len() < n {
            let centroid = self.calculate_vertex_centroid(&new_verts_list)?;
            let mut max_distance = 0.0;
            let mut new_vert = -1;
            for v in 0..self.number_of_vertices {
                if new_verts_list.contains(&(v as i32)) {
                    continue;
                }
                let mut distance_squared = 0.0;
                for (i, c) in centroid.iter().enumerate() {
                    let d = c - self.positions[i + DIM * v];
                    // Assigned rather than accumulated: only the last axis counts.
                    distance_squared = d * d;
                }
                if max_distance < distance_squared {
                    max_distance = distance_squared;
                    new_vert = v as i32;
                }
            }
            new_verts_list.push(new_vert);
        }
        Ok(new_verts_list)
    }

    /// The centroid of the listed vertices; a -1 padding vertex fails the hull.
    fn calculate_vertex_centroid(&self, vertex_indices: &[i32]) -> Result<[f64; DIM], HullFailed> {
        if vertex_indices.iter().any(|&v| v < 0) {
            // a -1 padding vertex has no position: the hull fails
            return Err(HullFailed);
        }
        let num_points = vertex_indices.len();
        let mut centroid = [0.0; DIM];
        for (i, c) in centroid.iter_mut().enumerate() {
            for &v in vertex_indices {
                *c += self.positions[i + DIM * v as usize];
            }
            *c /= num_points as f64;
        }
        Ok(centroid)
    }

    /// Links two faces of the initial simplex that share an edge.
    fn update_adjacency(&mut self, l: usize, r: usize) {
        let lv = self.face_pool[l].vertices.map(|v| v as usize);
        let rv = self.face_pool[r].vertices.map(|v| v as usize);
        // reset marks on the 1st face
        for &v in lv.iter() {
            self.vertex_visited[v] = false;
        }
        // mark all vertices on the 2nd face
        for &v in rv.iter() {
            self.vertex_visited[v] = true;
        }
        // find the 1st false index
        let mut i = 0;
        while i < DIM {
            if !self.vertex_visited[lv[i]] {
                break;
            }
            i += 1;
        }
        // no vertex was marked
        if i == DIM {
            return;
        }
        // check if only 1 vertex wasn't marked
        for &v in lv.iter().skip(i + 1) {
            if !self.vertex_visited[v] {
                return;
            }
        }
        // if we are here, the two faces share an edge
        self.face_pool[l].adjacent_faces[i] = self.face_pool[r].index as i32;
        // update the adj. face on the other face - find the vertex that remains marked
        for &v in lv.iter() {
            self.vertex_visited[v] = false;
        }
        let mut i = 0;
        while i < DIM {
            if self.vertex_visited[rv[i]] {
                break;
            }
            i += 1;
        }
        self.face_pool[r].adjacent_faces[i] = self.face_pool[l].index as i32;
    }
}
