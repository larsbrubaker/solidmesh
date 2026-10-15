//! The quickhull's main loop (derived from MIConvexHull), 3D path: for the furthest
//! vertex beyond the first unprocessed face, tag every face it sees, replace them with a cone
//! from the vertex to their horizon, and hand the old faces' beyond vertices to the new faces.
//!
//! An adjacency of -1 (a deposited face's link) means the hull's topology broke; it fails the
//! whole hull with [`HullFailed`].

use super::data_model::{DeferredFace, FaceConnector, CONNECTOR_TABLE_SIZE};
use super::math_helper::{calculate_face_plane, get_vertex_distance, DIM};
use super::{ConvexHullAlgorithm, HullFailed};

/// An adjacency entry as a face index; a negative one fails the hull.
fn face_index(adj: i32) -> Result<usize, HullFailed> {
    usize::try_from(adj).map_err(|_| HullFailed)
}

impl ConvexHullAlgorithm {
    /// Tags every face the current vertex is beyond, starting from `current_face`.
    pub(super) fn tag_affected_faces(&mut self, current_face: usize) -> Result<(), HullFailed> {
        self.affected_face_buffer.clear();
        self.affected_face_buffer
            .push(self.face_pool[current_face].index);
        self.traverse_affected_faces(self.face_pool[current_face].index)
    }

    /// Flood the faces the current vertex is beyond.
    fn traverse_affected_faces(&mut self, current_face: usize) -> Result<(), HullFailed> {
        self.traverse_stack.clear();
        self.traverse_stack.push(current_face);
        self.affected_face_flags[current_face] = true;

        while let Some(top) = self.traverse_stack.pop() {
            for i in 0..DIM {
                let adj_face = face_index(self.face_pool[top].adjacent_faces[i])?;
                if !self.affected_face_flags[adj_face]
                    && get_vertex_distance(
                        &self.positions,
                        self.current_vertex,
                        &self.face_pool[adj_face],
                    ) >= self.plane_distance_tolerance
                {
                    self.affected_face_buffer.push(adj_face);
                    self.affected_face_flags[adj_face] = true;
                    self.traverse_stack.push(adj_face);
                }
            }
        }
        Ok(())
    }

    /// Pair the connector with its twin in the hash table, or file it there.
    fn connect_face(&mut self, connector: usize) {
        let index = (self.connectors[connector].hash_code % CONNECTOR_TABLE_SIZE) as usize;
        let mut current = self.connector_table[index].first;
        while let Some(c) = current {
            if FaceConnector::are_connectable(&self.connectors[connector], &self.connectors[c]) {
                self.connector_table[index].remove(&mut self.connectors, c);
                // FaceConnector.Connect(current, connector)
                let (a_face, a_edge) = (self.connectors[c].face, self.connectors[c].edge_index);
                let (b_face, b_edge) = (
                    self.connectors[connector].face,
                    self.connectors[connector].edge_index,
                );
                if let (Some(a_face), Some(b_face)) = (a_face, b_face) {
                    self.face_pool[a_face].adjacent_faces[a_edge] = b_face as i32;
                    self.face_pool[b_face].adjacent_faces[b_edge] = a_face as i32;
                }
                self.connectors[c].face = None;
                self.connectors[connector].face = None;
                self.deposit_connector(c);
                self.deposit_connector(connector);
                return;
            }
            current = self.connectors[c].next;
        }
        self.connector_table[index].add(&mut self.connectors, connector);
    }

    /// A new face per horizon edge. False when one is degenerate (then the vertex
    /// is handled as singular).
    pub(super) fn create_cone(&mut self) -> Result<bool, HullFailed> {
        let current_vertex_index = self.current_vertex as i32;
        self.cone_face_buffer.clear();

        for f_index in 0..self.affected_face_buffer.len() {
            let old_face_index = self.affected_face_buffer[f_index];

            // Find the faces that need to be updated
            let mut update_count = 0;
            for i in 0..DIM {
                let af = face_index(self.face_pool[old_face_index].adjacent_faces[i])?;
                if !self.affected_face_flags[af] {
                    // Tag == false when oldFaces does not contain af
                    self.update_buffer[update_count] = af;
                    self.update_indices[update_count] = i;
                    update_count += 1;
                }
            }

            for i in 0..update_count {
                let adjacent_face = self.update_buffer[i];

                let mut old_face_adjacent_index = 0;
                for (j, &adj) in self.face_pool[adjacent_face]
                    .adjacent_faces
                    .iter()
                    .enumerate()
                {
                    if old_face_index as i32 == adj {
                        old_face_adjacent_index = j;
                        break;
                    }
                }

                // Index of the face that corresponds to this adjacent face
                let forbidden = self.update_indices[i];

                let new_face_index = self.get_face();
                let mut vertices = self.face_pool[old_face_index].vertices;
                let old_vertex_index = vertices[forbidden];

                let ordered_pivot_index = if current_vertex_index < old_vertex_index {
                    let mut pivot = 0;
                    let mut j = forbidden as isize - 1;
                    while j >= 0 {
                        let ju = j as usize;
                        if vertices[ju] > current_vertex_index {
                            vertices[ju + 1] = vertices[ju];
                        } else {
                            pivot = ju + 1;
                            break;
                        }
                        j -= 1;
                    }
                    pivot
                } else {
                    let mut pivot = DIM - 1;
                    for j in (forbidden + 1)..DIM {
                        if vertices[j] < current_vertex_index {
                            vertices[j - 1] = vertices[j];
                        } else {
                            pivot = j - 1;
                            break;
                        }
                    }
                    pivot
                };

                vertices[ordered_pivot_index] = current_vertex_index;
                self.face_pool[new_face_index].vertices = vertices;

                let inside = self.inside_point;
                if !calculate_face_plane(
                    &self.positions,
                    &mut self.face_pool[new_face_index],
                    &inside,
                ) {
                    return Ok(false);
                }

                self.cone_face_buffer.push(DeferredFace {
                    face: new_face_index,
                    face_index: ordered_pivot_index,
                    pivot: adjacent_face,
                    pivot_index: old_face_adjacent_index,
                    old_face: old_face_index,
                });
            }
        }
        Ok(true)
    }

    /// Link the cone faces, distribute beyond vertices, retire the old faces.
    pub(super) fn commit_cone(&mut self) {
        for i in 0..self.cone_face_buffer.len() {
            let face = self.cone_face_buffer[i];
            let new_face = face.face;
            let adjacent_face = face.pivot;
            let old_face = face.old_face;
            let ordered_pivot_index = face.face_index;

            self.face_pool[new_face].adjacent_faces[ordered_pivot_index] =
                self.face_pool[adjacent_face].index as i32;
            self.face_pool[adjacent_face].adjacent_faces[face.pivot_index] =
                self.face_pool[new_face].index as i32;

            // let there be a connection.
            for j in 0..DIM {
                if j == ordered_pivot_index {
                    continue;
                }
                let connector = self.get_connector();
                let nf = &self.face_pool[new_face];
                self.connectors[connector].update(nf, j);
                self.connect_face(connector);
            }

            let adjacent_beyond = self.face_pool[adjacent_face].vertices_beyond;
            let old_beyond = self.face_pool[old_face].vertices_beyond;
            let adjacent_count = self.buffers[adjacent_beyond].len();
            let old_count = self.buffers[old_beyond].len();
            if adjacent_count == 0 {
                self.find_beyond_vertices_one(new_face, old_beyond);
            } else if adjacent_count < old_count {
                self.find_beyond_vertices_two(new_face, adjacent_beyond, old_beyond);
            } else {
                self.find_beyond_vertices_two(new_face, old_beyond, adjacent_beyond);
            }

            let new_beyond = self.face_pool[new_face].vertices_beyond;
            if self.buffers[new_beyond].is_empty() {
                self.convex_faces.push(self.face_pool[new_face].index);
                self.unprocessed_faces.remove(&mut self.face_pool, new_face);
                self.deposit_vertex_buffer(new_beyond);
                self.face_pool[new_face].vertices_beyond = self.empty_buffer;
            } else {
                // Add the face to the list
                self.unprocessed_faces
                    .add(&mut self.face_pool, &self.buffers, new_face);
            }
        }

        for f_index in 0..self.affected_face_buffer.len() {
            let face = self.affected_face_buffer[f_index];
            self.unprocessed_faces.remove(&mut self.face_pool, face);
            self.deposit_face(face);
        }
    }

    /// Files `v` in `beyond_vertices` if it is beyond the face, tracking the
    /// furthest (ties within tolerance go to the lexicographically larger position).
    pub(super) fn is_beyond(&mut self, face: usize, beyond_vertices: usize, v: usize) {
        let distance = get_vertex_distance(&self.positions, v, &self.face_pool[face]);
        if distance >= self.plane_distance_tolerance {
            if distance > self.max_distance {
                // If it's within the tolerance distance, use the lex. larger point
                if distance - self.max_distance < self.plane_distance_tolerance {
                    if self.lex_compare(v, self.furthest_vertex) == std::cmp::Ordering::Greater {
                        self.max_distance = distance;
                        self.furthest_vertex = v;
                    }
                } else {
                    self.max_distance = distance;
                    self.furthest_vertex = v;
                }
            }
            self.buffers[beyond_vertices].push(v as i32);
        }
    }

    /// Compares two vertices coordinate by coordinate (NaN sorts first, so the order is total).
    fn lex_compare(&self, u: usize, v: usize) -> std::cmp::Ordering {
        let (u_offset, v_offset) = (u * DIM, v * DIM);
        for i in 0..DIM {
            let (x, y) = (self.positions[u_offset + i], self.positions[v_offset + i]);
            let comp = cs_compare(x, y);
            if comp != std::cmp::Ordering::Equal {
                return comp;
            }
        }
        std::cmp::Ordering::Equal
    }

    /// The union of two beyond sets. Buffers are
    /// read by index with a live length, because they can alias (see data_model).
    fn find_beyond_vertices_two(&mut self, face: usize, beyond: usize, beyond1: usize) {
        let beyond_vertices = self.beyond_buffer;
        self.max_distance = f64::NEG_INFINITY;
        self.furthest_vertex = 0;

        let mut i = 0;
        while i < self.buffers[beyond1].len() {
            let v = self.buffers[beyond1][i] as usize;
            self.vertex_visited[v] = true;
            i += 1;
        }
        self.vertex_visited[self.current_vertex] = false;
        let mut i = 0;
        while i < self.buffers[beyond].len() {
            let v = self.buffers[beyond][i] as usize;
            i += 1;
            if v == self.current_vertex {
                continue;
            }
            self.vertex_visited[v] = false;
            self.is_beyond(face, beyond_vertices, v);
        }
        let mut i = 0;
        while i < self.buffers[beyond1].len() {
            let v = self.buffers[beyond1][i] as usize;
            i += 1;
            if self.vertex_visited[v] {
                self.is_beyond(face, beyond_vertices, v);
            }
        }

        self.swap_in_beyond_buffer(face, beyond_vertices);
    }

    /// The vertices of one beyond set that are beyond `face`.
    fn find_beyond_vertices_one(&mut self, face: usize, beyond: usize) {
        let beyond_vertices = self.beyond_buffer;
        self.max_distance = f64::NEG_INFINITY;
        self.furthest_vertex = 0;
        let mut i = 0;
        while i < self.buffers[beyond].len() {
            let v = self.buffers[beyond][i] as usize;
            i += 1;
            if v == self.current_vertex {
                continue;
            }
            self.is_beyond(face, beyond_vertices, v);
        }
        self.swap_in_beyond_buffer(face, beyond_vertices);
    }

    /// The tail both beyond-vertex searches share: the filled working buffer becomes
    /// the face's, and the face's old buffer (cleared) becomes the working buffer.
    fn swap_in_beyond_buffer(&mut self, face: usize, beyond_vertices: usize) {
        self.face_pool[face].furthest_vertex = self.furthest_vertex;
        let temp = self.face_pool[face].vertices_beyond;
        self.face_pool[face].vertices_beyond = beyond_vertices;
        self.buffers[temp].clear();
        self.beyond_buffer = temp;
    }

    /// The vertex (and everything beyond its faces) cannot extend the hull;
    /// the affected faces are kept as hull faces.
    pub(super) fn handle_singular(&mut self) {
        self.singular_vertices.insert(self.current_vertex);
        for f_index in 0..self.affected_face_buffer.len() {
            let face = self.affected_face_buffer[f_index];
            let vb = self.face_pool[face].vertices_beyond;
            for i in 0..self.buffers[vb].len() {
                self.singular_vertices.insert(self.buffers[vb][i] as usize);
            }
            self.convex_faces.push(self.face_pool[face].index);
            self.unprocessed_faces.remove(&mut self.face_pool, face);
            self.deposit_vertex_buffer(vb);
            self.face_pool[face].vertices_beyond = self.empty_buffer;
        }
    }
}

/// A total order with NaN below everything, NaN equal to NaN.
fn cs_compare(x: f64, y: f64) -> std::cmp::Ordering {
    use std::cmp::Ordering;
    if x < y {
        Ordering::Less
    } else if x > y {
        Ordering::Greater
    } else if x == y {
        Ordering::Equal
    } else if x.is_nan() {
        if y.is_nan() {
            Ordering::Equal
        } else {
            Ordering::Less
        }
    } else {
        Ordering::Greater
    }
}
