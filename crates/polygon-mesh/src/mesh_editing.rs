//! Mesh editing: the vertex BVH, `UnionFind` and vertex merging, splitting a face, splitting a
//! mesh by a plane, and `split_on_planes`.
//!
//! Each rebuild records which source face every new face came from and calls
//! `Mesh::remap_face_data` (`face_data.rs`) so face colors and textures follow the faces; cut
//! pieces get their UVs interpolated from the face they were cut out of.

use std::collections::{HashMap, HashSet};

use vector_math::bvh::{BvhTree, BvhTreeItemData, TradeOffBvhConstructor};
use vector_math::cs_math::{max_f64, min_f64};
use vector_math::{AxisAlignedBoundingBox, Plane, Vector3, Vector3Float};

use crate::face::{Face, FaceList};
use crate::face_split::{clip_edge_at_plane, SplitData};
use crate::mesh::Mesh;

/// A progress callback: the fraction done and a status message.
pub type ProgressReporter<'a> = &'a mut dyn FnMut(f64, &str);

/// Disjoint sets with path compression and union by size.
#[derive(Debug, Clone)]
pub struct UnionFind {
    parent: Vec<usize>,
    size: Vec<usize>,
}

impl UnionFind {
    /// `n` singleton sets.
    pub fn new(n: usize) -> Self {
        UnionFind {
            parent: (0..n).collect(),
            size: vec![1; n],
        }
    }

    /// The representative of `i`'s set.
    pub fn find(&mut self, i: usize) -> usize {
        // iterative path compression; same final parents as the recursive form
        let mut root = i;
        while self.parent[root] != root {
            root = self.parent[root];
        }
        let mut node = i;
        while self.parent[node] != root {
            let next = self.parent[node];
            self.parent[node] = root;
            node = next;
        }
        root
    }

    /// Merges the sets of `a` and `b`; the larger set's root wins, `a`'s on a tie.
    pub fn union(&mut self, a: usize, b: usize) {
        let root_a = self.find(a);
        let root_b = self.find(b);
        if root_a == root_b {
            return;
        }

        if self.size[root_a] < self.size[root_b] {
            self.parent[root_a] = root_b;
            self.size[root_b] += self.size[root_a];
        } else {
            self.parent[root_b] = root_a;
            self.size[root_a] += self.size[root_b];
        }
    }
}

impl Mesh {
    /// A BVH over the vertex indices, each a .001 box around its vertex; `None` for a mesh
    /// with no vertices.
    pub fn get_vertex_bvh_tree(&self) -> Option<BvhTree<i32>> {
        let tiny_distance = Vector3Float::new(0.001, 0.001, 0.001);
        let items: Vec<BvhTreeItemData<i32>> = self
            .vertices
            .iter()
            .enumerate()
            .map(|(i, &v)| {
                BvhTreeItemData::new(
                    i as i32,
                    AxisAlignedBoundingBox::from_vector3_float(
                        v - tiny_distance,
                        v + tiny_distance,
                    ),
                )
            })
            .collect();
        TradeOffBvhConstructor::create_new_hierarchy(
            items,
            TradeOffBvhConstructor::DEFAULT_MAX_RECURSION,
            0,
            None,
            10,
        )
    }

    /// Welds every group of vertices within `treat_as_same_distance` of each other (box
    /// distance, transitively) to their average, then drops faces that collapse or fall below
    /// `min_face_area`. `reporter` gets progress every 256 vertices.
    pub fn merge_vertices(
        &mut self,
        treat_as_same_distance: f64,
        min_face_area: f64,
        mut reporter: Option<ProgressReporter<'_>>,
    ) {
        let Some(vertex_bvh_tree) = self.get_vertex_bvh_tree() else {
            return;
        };
        let vertex_count = self.vertices.len();

        let mut uf = UnionFind::new(vertex_count);
        let mut search_results: Vec<&i32> = Vec::new();
        let merge_distance = treat_as_same_distance as f32;
        let offset = Vector3Float::new_f32(merge_distance, merge_distance, merge_distance);

        // First pass: Find all vertices that should be merged
        for i in 0..vertex_count {
            // Skip if this vertex is already merged into another group
            if uf.find(i) != i {
                continue;
            }

            if i % 256 == 0 {
                if let Some(reporter) = reporter.as_mut() {
                    reporter(i as f64 / vertex_count as f64, "Merge Vertices");
                }
            }

            search_results.clear();
            let vertex = self.vertices[i];
            let edge_aabb =
                AxisAlignedBoundingBox::from_vector3_float(vertex - offset, vertex + offset);
            vertex_bvh_tree.search_bounds(&edge_aabb, &mut search_results);

            if search_results.len() > 1 {
                for &&result in &search_results {
                    uf.union(i, result as usize);
                }
            }
        }

        // Accumulate positions for each representative
        let mut x_sum = vec![0.0f64; vertex_count];
        let mut y_sum = vec![0.0f64; vertex_count];
        let mut z_sum = vec![0.0f64; vertex_count];
        let mut count = vec![0i32; vertex_count];
        for i in 0..vertex_count {
            let rep = uf.find(i);
            let vertex = self.vertices[i];
            x_sum[rep] += f64::from(vertex.x);
            y_sum[rep] += f64::from(vertex.y);
            z_sum[rep] += f64::from(vertex.z);
            count[rep] += 1;
        }

        // Create new vertices for each representative
        let mut rep_to_new_index: HashMap<usize, i32> = HashMap::new();
        let mut new_vertices: Vec<Vector3Float> = Vec::new();
        for i in 0..vertex_count {
            if count[i] > 0 {
                rep_to_new_index.insert(i, new_vertices.len() as i32);
                let c = f64::from(count[i]);
                new_vertices.push(Vector3Float::new_f32(
                    (x_sum[i] / c) as f32,
                    (y_sum[i] / c) as f32,
                    (z_sum[i] / c) as f32,
                ));
            }
        }

        // Create new faces, skipping degenerate ones
        let mut new_faces = FaceList::new();
        let mut source_face_index: Vec<i32> = Vec::with_capacity(self.faces.len());
        for face_idx in 0..self.faces.len() {
            let face = self.faces[face_idx];
            let new_v0 = rep_to_new_index[&uf.find(face.v0 as usize)];
            let new_v1 = rep_to_new_index[&uf.find(face.v1 as usize)];
            let new_v2 = rep_to_new_index[&uf.find(face.v2 as usize)];

            if new_v0 != new_v1 && new_v1 != new_v2 && new_v2 != new_v0 {
                let area = new_vertices[new_v0 as usize]
                    .get_area(new_vertices[new_v1 as usize], new_vertices[new_v2 as usize]);
                if f64::from(area) >= min_face_area {
                    new_faces.add(new_v0, new_v1, new_v2, &new_vertices);
                    source_face_index.push(face_idx as i32);
                }
            }
        }

        // Update the mesh if changes were made
        if self.faces.len() != new_faces.len() || self.vertices.len() != new_vertices.len() {
            self.vertices = new_vertices;
            self.faces = new_faces;

            // Dropping the collapsed faces shifts every face after them down, so the colors
            // and textures have to follow.
            self.remap_face_data(&source_face_index, None);

            self.mark_as_changed();
        }
    }

    /// Splits face `face_index` at `plane`: the face is removed, its pieces appended, then the
    /// mesh is cleaned and merged. Returns whether the face was actually split.
    pub fn split_face(&mut self, face_index: usize, plane: Plane, on_plane_distance: f64) -> bool {
        let mut new_vertices: Vec<Vector3Float> = Vec::new();
        let mut new_faces: Vec<Face> = Vec::new();
        if self.faces[face_index].split(
            &self.vertices,
            plane,
            &mut new_faces,
            &mut new_vertices,
            on_plane_distance,
            None,
            false,
        ) {
            let vertex_count = self.vertices.len() as i32;
            let source_faces = self.faces.clone();
            let mut source_face_index: Vec<i32> =
                Vec::with_capacity(self.faces.len() + new_faces.len());
            source_face_index
                .extend((0..self.faces.len() as i32).filter(|&i| i != face_index as i32));
            // removing the face shifts everything after it down, and the pieces go on the end
            self.faces.remove(face_index);
            self.vertices.extend_from_slice(&new_vertices);
            for mut face in new_faces {
                face.v0 += vertex_count;
                face.v1 += vertex_count;
                face.v2 += vertex_count;
                self.faces.push(face);
                source_face_index.push(face_index as i32);
            }

            // the pieces are cut out of the face that was split, so their UVs get interpolated
            let vertices = self.vertices.clone();
            self.remap_face_data(&source_face_index, Some((&source_faces, &vertices)));

            // Faces and Vertices both changed, so invalidate here rather than leaning on the
            // mark_as_changed inside clean_and_merge - which returns early for an already
            // manifold result.
            self.mark_as_changed();
            self.clean_and_merge();
            return true;
        }

        false
    }

    /// Splits every face that crosses `plane` (see [`Face::split`]), keeping the uncut faces in
    /// order and appending the pieces; optionally cleans and merges afterwards. Always true.
    pub fn split(
        &mut self,
        plane: Plane,
        on_plane_distance: f64,
        clip_face: Option<&dyn Fn(&SplitData) -> bool>,
        clean_and_merge: bool,
        discard_faces_on_negative_side: bool,
    ) -> bool {
        let mut new_vertices: Vec<Vector3Float> = Vec::new();
        let mut new_faces: Vec<Face> = Vec::new();
        let mut faces_to_remove: HashSet<usize> = HashSet::new();
        // which face each entry of new_faces was cut out of, so the per face data can follow
        let mut new_face_source: Vec<i32> = Vec::new();

        for i in 0..self.faces.len() {
            let face = self.faces[i];
            let pieces_before = new_faces.len();
            if face.split(
                &self.vertices,
                plane,
                &mut new_faces,
                &mut new_vertices,
                on_plane_distance,
                clip_face,
                discard_faces_on_negative_side,
            ) {
                // record the face for removal
                faces_to_remove.insert(i);
                new_face_source.extend(std::iter::repeat_n(
                    i as i32,
                    new_faces.len() - pieces_before,
                ));
            }
        }

        let mut kept_faces = FaceList::new();
        let mut source_face_index: Vec<i32> =
            Vec::with_capacity(self.faces.len() + new_faces.len());
        for (i, face) in self.faces.iter().enumerate() {
            if !faces_to_remove.contains(&i) {
                kept_faces.push(*face);
                source_face_index.push(i as i32);
            }
        }

        let vertex_count = self.vertices.len() as i32;
        self.vertices.extend_from_slice(&new_vertices);
        for (i, mut face) in new_faces.into_iter().enumerate() {
            face.v0 += vertex_count;
            face.v1 += vertex_count;
            face.v2 += vertex_count;
            kept_faces.push(face);
            source_face_index.push(new_face_source[i]);
        }

        let source_faces = std::mem::replace(&mut self.faces, kept_faces);

        // the added faces are pieces cut out of the faces they replaced, so their UVs get
        // interpolated
        let vertices = self.vertices.clone();
        self.remap_face_data(&source_face_index, Some((&source_faces, &vertices)));

        // Faces and Vertices both changed. clean_and_merge is the only other thing here that
        // would invalidate, and callers can switch it off, so do it unconditionally.
        self.mark_as_changed();

        if clean_and_merge {
            self.clean_and_merge();
        }

        true
    }

    /// Cuts every face along each of the parallel planes `normal . p = d` for `d` in
    /// `distances_from_origin`, so no face crosses any of them. Edge
    /// crossings are computed once per edge so the two faces on an edge agree bit for bit.
    pub fn split_on_planes(
        &mut self,
        plane_normal: Vector3,
        distances_from_origin: &[f64],
        on_plane_distance: f64,
    ) {
        if distances_from_origin.is_empty() {
            return;
        }
        let mut on_plane_distance = on_plane_distance;

        // Plane(normal, distance) normalizes, so match it or the distances mean something else.
        let normal = plane_normal.get_normal();

        let mut sorted_planes = distances_from_origin.to_vec();
        sorted_planes.sort_by(f64::total_cmp);

        // drop exact duplicates
        let mut planes: Vec<f64> = vec![sorted_planes[0]];
        for i in 1..sorted_planes.len() {
            if sorted_planes[i] != sorted_planes[i - 1] {
                planes.push(sorted_planes[i]);
            }
        }

        // keep the tolerance well inside the plane spacing
        for i in 1..planes.len() {
            on_plane_distance = min_f64(on_plane_distance, (planes[i] - planes[i - 1]) * 0.45);
        }

        let source_vertices = std::mem::take(&mut self.vertices);
        let source_faces = std::mem::take(&mut self.faces);

        let vertex_offset: Vec<f64> = source_vertices
            .iter()
            .map(|v| {
                (normal.x * f64::from(v.x))
                    + (normal.y * f64::from(v.y))
                    + (normal.z * f64::from(v.z))
            })
            .collect();

        let mut new_vertices: Vec<Vector3Float> = source_vertices.clone();
        let mut new_faces = FaceList::new();

        let mut ring: Vec<i32> = Vec::with_capacity(8);
        let mut ring_offset: Vec<f64> = Vec::with_capacity(8);
        let mut below: Vec<i32> = Vec::with_capacity(8);
        let mut above: Vec<i32> = Vec::with_capacity(8);
        let mut above_offset: Vec<f64> = Vec::with_capacity(8);
        let mut edge_crossings: Vec<usize> = Vec::with_capacity(8);

        // which source face each emitted triangle came out of; the colors and textures are
        // moved onto the pieces from this one map once the new face list is in place
        let mut source_face_index: Vec<i32> = Vec::with_capacity(source_faces.len());

        fn add_triangle(
            v0: i32,
            v1: i32,
            v2: i32,
            new_vertices: &[Vector3Float],
            out: (&mut FaceList, &mut Vec<i32>, i32),
        ) {
            let (new_faces, source_face_index, source_face) = out;
            if v0 == v1 || v1 == v2 || v2 == v0 {
                return;
            }
            let p0 = Vector3::from_vector3_float(new_vertices[v0 as usize]);
            let p1 = Vector3::from_vector3_float(new_vertices[v1 as usize]);
            let p2 = Vector3::from_vector3_float(new_vertices[v2 as usize]);
            if (p1 - p0).cross(p2 - p0).length_squared() == 0.0 {
                return;
            }
            new_faces.add(v0, v1, v2, new_vertices);
            source_face_index.push(source_face);
        }

        fn emit_fan(
            polygon: &[i32],
            new_vertices: &[Vector3Float],
            new_faces: &mut FaceList,
            source_face_index: &mut Vec<i32>,
            source_face: i32,
        ) {
            let mut i = 1;
            while i + 1 < polygon.len() {
                add_triangle(
                    polygon[0],
                    polygon[i],
                    polygon[i + 1],
                    new_vertices,
                    (&mut *new_faces, &mut *source_face_index, source_face),
                );
                i += 1;
            }
        }

        // Index of the first plane strictly above offset.
        let first_plane_above = |offset: f64| -> usize {
            let mut low = 0;
            let mut high = planes.len();
            while low < high {
                let mid = (low + high) / 2;
                if planes[mid] > offset {
                    high = mid;
                } else {
                    low = mid + 1;
                }
            }
            low
        };

        for (face_index, face) in source_faces.iter().enumerate() {
            let source_face = face_index as i32;
            let corners = [face.v0, face.v1, face.v2];

            ring.clear();
            ring_offset.clear();

            for edge in 0..3 {
                let start_index = corners[edge];
                let end_index = corners[(edge + 1) % 3];
                ring.push(start_index);
                ring_offset.push(vertex_offset[start_index as usize]);

                let start_offset = vertex_offset[start_index as usize];
                let end_offset = vertex_offset[end_index as usize];
                let low_offset = min_f64(start_offset, end_offset);
                let high_offset = max_f64(start_offset, end_offset);

                edge_crossings.clear();
                let mut plane = first_plane_above(low_offset);
                while plane < planes.len() && planes[plane] < high_offset {
                    let start_dist = start_offset - planes[plane];
                    let end_dist = end_offset - planes[plane];
                    if start_dist.abs() > on_plane_distance
                        && end_dist.abs() > on_plane_distance
                        && ((start_dist < 0.0 && end_dist > 0.0)
                            || (start_dist > 0.0 && end_dist < 0.0))
                    {
                        new_vertices.push(clip_edge_at_plane(
                            source_vertices[start_index as usize],
                            source_vertices[end_index as usize],
                            start_dist,
                            end_dist,
                        ));
                        edge_crossings.push(plane);
                    }
                    plane += 1;
                }

                if start_offset > end_offset {
                    edge_crossings.reverse();
                }

                let crossing_vertex = (new_vertices.len() - edge_crossings.len()) as i32;
                let crossing_count = edge_crossings.len() as i32;
                for (i, &crossing) in edge_crossings.iter().enumerate() {
                    let i = i as i32;
                    ring.push(if start_offset > end_offset {
                        crossing_vertex + crossing_count - 1 - i
                    } else {
                        crossing_vertex + i
                    });
                    ring_offset.push(planes[crossing]);
                }
            }

            if ring.len() == 3 {
                // No plane touched this face - keep it exactly as it was, normal included.
                new_faces.add_with_normal(face.v0, face.v1, face.v2, face.normal);
                source_face_index.push(source_face);
                continue;
            }

            let o = |v: i32| vertex_offset[v as usize];
            let face_low = min_f64(o(face.v0), min_f64(o(face.v1), o(face.v2)));
            let face_high = max_f64(o(face.v0), max_f64(o(face.v1), o(face.v2)));

            let mut plane = first_plane_above(face_low);
            while plane < planes.len() && planes[plane] < face_high {
                let plane_distance = planes[plane];
                plane += 1;

                below.clear();
                above.clear();
                above_offset.clear();

                let mut any_below = false;
                let mut any_above = false;
                for &offset in &ring_offset {
                    let side = offset - plane_distance;
                    if side < -on_plane_distance {
                        any_below = true;
                    } else if side > on_plane_distance {
                        any_above = true;
                    }
                }

                if !any_below || !any_above {
                    continue;
                }

                for i in 0..ring.len() {
                    let next = (i + 1) % ring.len();
                    let side = side_of(ring_offset[i], plane_distance, on_plane_distance);
                    let next_side = side_of(ring_offset[next], plane_distance, on_plane_distance);

                    if side <= 0 {
                        below.push(ring[i]);
                    }

                    if side >= 0 {
                        above.push(ring[i]);
                        above_offset.push(ring_offset[i]);
                    }

                    if side != 0 && next_side != 0 && side != next_side {
                        // Should be unreachable: every crossing was inserted per edge.
                        debug_assert!(
                            false,
                            "SplitOnPlanes: ring segment straddles a plane; crossings should all have been inserted per edge."
                        );
                        let point = clip_edge_at_plane(
                            new_vertices[ring[i] as usize],
                            new_vertices[ring[next] as usize],
                            ring_offset[i] - plane_distance,
                            ring_offset[next] - plane_distance,
                        );
                        new_vertices.push(point);
                        let point_index = new_vertices.len() as i32 - 1;
                        below.push(point_index);
                        above.push(point_index);
                        above_offset.push(plane_distance);
                    }
                }

                emit_fan(
                    &below,
                    &new_vertices,
                    &mut new_faces,
                    &mut source_face_index,
                    source_face,
                );

                // Carry on with what is above the plane.
                std::mem::swap(&mut ring, &mut above);
                std::mem::swap(&mut ring_offset, &mut above_offset);
            }

            emit_fan(
                &ring,
                &new_vertices,
                &mut new_faces,
                &mut source_face_index,
                source_face,
            );
        }

        self.vertices = new_vertices;
        self.faces = new_faces;

        // Every emitted triangle is a piece of the face it came from, so its corners get the UVs
        // the source face's mapping gives at those positions.
        self.remap_face_data(&source_face_index, Some((&source_faces, &source_vertices)));

        self.clean_and_merge();
        self.mark_as_changed();
    }
}

/// Which side of a plane a point sits on, anything within `on_plane_distance` counting as on
/// it.
fn side_of(offset: f64, plane_distance: f64, on_plane_distance: f64) -> i32 {
    let distance = offset - plane_distance;
    if distance < -on_plane_distance {
        return -1;
    }
    if distance > on_plane_distance {
        1
    } else {
        0
    }
}
