//! T-junction repair: agg-sharp `PolygonMesh/Mesh.cs` `RepairTJunctions` and
//! `RepairTJunctionsPass`. Every vertex lying on the interior of an edge it is not connected to
//! gets that edge split at it, fanning the edge's faces from their opposite corners.

use std::cmp::Ordering;
use std::collections::{HashMap, HashSet};

use vector_math::cs_math::{max_f32, min_f32};
use vector_math::cs_sort::introspective_sort;
use vector_math::{AxisAlignedBoundingBox, Vector3Float};

use crate::face::{Face, FaceList};
use crate::mesh::Mesh;
use crate::mesh_editing::ProgressReporter;

/// Where a vertex splits an edge (C# `VertexSplitInfo`): the vertex to use (-1 for a new one at
/// `projection_point`) and its distance along the edge.
#[derive(Debug, Clone, Copy)]
struct VertexSplitInfo {
    vertex_index: i32,
    projection_scalar: f32,
    projection_point: Vector3Float,
}

/// C# `float.CompareTo`: NaN sorts before everything and equals itself.
fn float_compare_to(a: f32, b: f32) -> Ordering {
    match (a.is_nan(), b.is_nan()) {
        (true, true) => Ordering::Equal,
        (true, false) => Ordering::Less,
        (false, true) => Ordering::Greater,
        _ => a.partial_cmp(&b).unwrap_or(Ordering::Equal),
    }
}

impl Mesh {
    /// For every T-junction adds a vertex to the edge that needs one, in up to three passes
    /// (four with `aggressive_non_manifold_fix`, whose first pass also moves vertices within
    /// `aggressive_max_distance` onto non-manifold edges), with merges and cleanups around them.
    pub fn repair_t_junctions(
        &mut self,
        reporter: Option<ProgressReporter<'_>>,
        aggressive_non_manifold_fix: bool,
        aggressive_max_distance: f64,
    ) {
        let mut no_report = |_: f64, _: &str| {};
        let reporter: &mut dyn FnMut(f64, &str) = match reporter {
            Some(r) => r,
            None => &mut no_report,
        };

        const EPSILON: f32 = 1e-3;
        const EPSILON_SQ: f32 = EPSILON * EPSILON;

        let aggressive_distance_threshold = aggressive_max_distance as f32;
        let aggressive_distance_threshold_sq =
            aggressive_distance_threshold * aggressive_distance_threshold;

        if self.vertices.is_empty() || self.faces.is_empty() {
            reporter(1.0, "Empty mesh - no repair needed.");
            return;
        }

        reporter(0.0, "Initial Cleanup");
        self.remove_degenerate_faces(f64::from(EPSILON_SQ));
        self.remove_unused_vertices();
        self.merge_vertices(
            f64::from(EPSILON),
            f64::from(EPSILON_SQ),
            Some(&mut |r, _| reporter(r * 0.05, "Initial Merge")),
        );

        let mut repair_made = true;
        let mut pass_count = 0;
        let max_passes = if aggressive_non_manifold_fix { 4 } else { 3 };

        while repair_made && pass_count < max_passes {
            pass_count += 1;
            let pass_base = 0.05 + f64::from(pass_count - 1) * 0.3 / f64::from(max_passes);
            reporter(pass_base, &format!("T-Junction Pass {pass_count}"));

            let use_aggressive_mode = aggressive_non_manifold_fix && pass_count == 1;

            repair_made = repair_t_junctions_pass(
                self,
                EPSILON,
                EPSILON_SQ,
                use_aggressive_mode,
                aggressive_distance_threshold,
                aggressive_distance_threshold_sq,
                &mut |r, m| reporter(pass_base + r * 0.25, &format!("{m} (Pass {pass_count})")),
            );

            if repair_made {
                self.remove_degenerate_faces(f64::from(EPSILON_SQ));
                self.remove_unused_vertices();
                self.calculate_normals();
            }
        }

        reporter(0.9, "Final Cleanup");
        self.merge_vertices(
            f64::from(EPSILON),
            f64::from(EPSILON_SQ),
            Some(&mut |r, _| reporter(0.9 + r * 0.05, "Final Merge")),
        );
        self.remove_degenerate_faces(f64::from(EPSILON_SQ));
        self.remove_unused_vertices();
        self.calculate_normals();

        reporter(
            1.0,
            &format!("T-Junction repair complete ({pass_count} passes)."),
        );
    }
}

/// One pass of T-junction repair; true when the mesh was modified.
fn repair_t_junctions_pass(
    mesh: &mut Mesh,
    epsilon: f32,
    epsilon_sq: f32,
    aggressive_mode: bool,
    aggressive_threshold: f32,
    aggressive_threshold_sq: f32,
    reporter: &mut dyn FnMut(f64, &str),
) -> bool {
    reporter(0.0, "Get Mesh Edges");
    let edges = mesh.get_mesh_edges();

    reporter(0.1, "Get Vertex Tree");
    let Some(vertex_bvh_tree) = mesh.get_vertex_bvh_tree() else {
        return false;
    };
    let mut search_results: Vec<&i32> = Vec::new();
    // insertion ordered, as the C# Dictionary<MeshEdge, ...> enumerates (no removals)
    let mut edges_to_split: Vec<(usize, Vec<VertexSplitInfo>)> = Vec::new();
    let mut vertices_to_move: Vec<(i32, Vector3Float)> = Vec::new();
    let mut vertices_to_move_slot: HashMap<i32, usize> = HashMap::new();

    let vertex_face_lists = if aggressive_mode {
        Some(mesh.get_vertex_face_lists())
    } else {
        None
    };

    reporter(0.15, "Finding T-Junctions");

    // non-manifold edges first; aggressive mode processes only those
    let mut process_queue: Vec<usize> = (0..edges.len())
        .filter(|&e| edges[e].faces().len() != 2)
        .collect();
    if !aggressive_mode {
        process_queue.extend((0..edges.len()).filter(|&e| edges[e].faces().len() == 2));
    }

    let total_edges = process_queue.len();
    let mut processed_edges = 0;

    for &edge_index in &process_queue {
        let edge = &edges[edge_index];
        processed_edges += 1;

        if edge.faces().is_empty() {
            continue;
        }

        let start = mesh.vertices[edge.vertex0_index as usize];
        let end = mesh.vertices[edge.vertex1_index as usize];
        let edge_vector = end - start;
        let edge_length_squared = edge_vector.length_squared();

        if edge_length_squared < epsilon_sq {
            continue;
        }

        let edge_length = edge_length_squared.sqrt();
        let edge_direction = edge_vector / f64::from(edge_length);

        let search_radius = if aggressive_mode {
            aggressive_threshold
        } else {
            epsilon * 2.0
        };

        let mut edge_aabb = AxisAlignedBoundingBox::from_vector3_float(start, end);
        edge_aabb.expand(f64::from(search_radius));

        search_results.clear();
        vertex_bvh_tree.search_bounds(&edge_aabb, &mut search_results);

        let mut edge_split_infos: Vec<VertexSplitInfo> = Vec::new();

        for &&vertex_index in &search_results {
            if vertex_index == edge.vertex0_index || vertex_index == edge.vertex1_index {
                continue;
            }

            let vertex = mesh.vertices[vertex_index as usize];
            let vertex_vector = vertex - start;

            let projection_scalar = vertex_vector.dot(edge_direction);

            let projection_is_on_edge =
                projection_scalar >= -epsilon && projection_scalar <= edge_length + epsilon;
            if !projection_is_on_edge && !aggressive_mode {
                continue;
            }

            let clamped_projection = max_f32(0.0, min_f32(edge_length, projection_scalar));
            let point_on_line = start + edge_direction * f64::from(clamped_projection);
            let dist_sq = (vertex - point_on_line).length_squared();

            let threshold_sq = if aggressive_mode {
                aggressive_threshold_sq
            } else {
                epsilon_sq
            };

            if dist_sq <= threshold_sq {
                let is_vertex_connected_to_edge = edge.faces().iter().any(|&face_index| {
                    let face = mesh.faces[face_index as usize];
                    face.v0 == vertex_index || face.v1 == vertex_index || face.v2 == vertex_index
                });

                if !is_vertex_connected_to_edge {
                    let near_start = projection_scalar <= epsilon;
                    let near_end = projection_scalar >= edge_length - epsilon;
                    if near_start || near_end {
                        // handled by MergeVertices
                        continue;
                    }

                    if aggressive_mode && dist_sq > epsilon_sq {
                        let mut can_move_vertex = true;

                        if let Some(lists) = &vertex_face_lists {
                            if (vertex_index as usize) < lists.len() {
                                let vertex_faces = lists[vertex_index as usize].faces();
                                if vertex_faces.len() > 4 {
                                    can_move_vertex = false;
                                }

                                for &connected_face_index in vertex_faces {
                                    let connected_face = mesh.faces[connected_face_index as usize];
                                    let (v1, v2) = if connected_face.v0 == vertex_index {
                                        (connected_face.v1, connected_face.v2)
                                    } else if connected_face.v1 == vertex_index {
                                        (connected_face.v0, connected_face.v2)
                                    } else {
                                        (connected_face.v0, connected_face.v1)
                                    };
                                    let area_with_new_point = point_on_line.get_area(
                                        mesh.vertices[v1 as usize],
                                        mesh.vertices[v2 as usize],
                                    );
                                    if area_with_new_point < epsilon_sq {
                                        can_move_vertex = false;
                                        break;
                                    }
                                }
                            }
                        }

                        if can_move_vertex {
                            match vertices_to_move_slot.get(&vertex_index) {
                                Some(&slot) => vertices_to_move[slot].1 = point_on_line,
                                None => {
                                    vertices_to_move_slot
                                        .insert(vertex_index, vertices_to_move.len());
                                    vertices_to_move.push((vertex_index, point_on_line));
                                }
                            }
                            edge_split_infos.push(VertexSplitInfo {
                                vertex_index,
                                projection_scalar: clamped_projection,
                                projection_point: point_on_line,
                            });
                        } else {
                            edge_split_infos.push(VertexSplitInfo {
                                vertex_index: -1,
                                projection_scalar: clamped_projection,
                                projection_point: point_on_line,
                            });
                        }
                    } else {
                        edge_split_infos.push(VertexSplitInfo {
                            vertex_index,
                            projection_scalar: clamped_projection,
                            projection_point: Vector3Float::ZERO,
                        });
                    }
                }
            }
        }

        if !edge_split_infos.is_empty() {
            edges_to_split.push((edge_index, edge_split_infos));
        }

        if total_edges > 0 {
            reporter(
                0.15 + (processed_edges as f64 / total_edges as f64 * 0.5),
                &format!(
                    "Finding {}T-Junctions",
                    if aggressive_mode { "Aggressive " } else { "" }
                ),
            );
        }
    }

    if edges_to_split.is_empty() && vertices_to_move.is_empty() {
        reporter(1.0, "No T-Junctions found to repair in this pass.");
        return false;
    }

    if !vertices_to_move.is_empty() {
        reporter(
            0.65,
            &format!("Moving {} vertices to edges", vertices_to_move.len()),
        );
        for &(index, position) in &vertices_to_move {
            mesh.vertices[index as usize] = position;
        }
        mesh.mark_as_changed();
    }

    if !edges_to_split.is_empty() {
        reporter(0.7, "Splitting Edges");
        let mut edges_split = 0;
        let edges_to_split_count = edges_to_split.len();

        let mut faces_to_remove: HashSet<i32> = HashSet::new();
        let mut new_faces_to_add: Vec<Face> = Vec::new();
        // which face each new face is a piece of, so the face colors and textures can follow
        let mut new_face_source: Vec<i32> = Vec::new();
        let mut new_vertices_to_add: Vec<Vector3Float> = Vec::new();
        let mut vertex_index_mapping: HashMap<i32, i32> = HashMap::new();

        for (edge_index, split_infos) in edges_to_split.iter_mut() {
            let edge_to_split = &edges[*edge_index];
            if split_infos.is_empty() {
                continue;
            }

            let v_start_index = edge_to_split.vertex0_index;
            let v_end_index = edge_to_split.vertex1_index;

            introspective_sort(split_infos, &mut |a, b| {
                float_compare_to(a.projection_scalar, b.projection_scalar)
            });

            let mut split_sequence: Vec<i32> = vec![v_start_index];
            for split_info in split_infos.iter() {
                let vertex_index = if split_info.vertex_index >= 0 {
                    split_info.vertex_index
                } else {
                    let temporary = -(new_vertices_to_add.len() as i32) - 1;
                    new_vertices_to_add.push(split_info.projection_point);
                    vertex_index_mapping.insert(
                        temporary,
                        (mesh.vertices.len() + new_vertices_to_add.len() - 1) as i32,
                    );
                    temporary
                };

                if !split_sequence.contains(&vertex_index) {
                    split_sequence.push(vertex_index);
                }
            }

            if !split_sequence.contains(&v_end_index) {
                split_sequence.push(v_end_index);
            }

            if split_sequence.len() < 3 {
                continue;
            }

            for &face_index in edge_to_split.faces() {
                if faces_to_remove.contains(&face_index) {
                    continue;
                }

                let original_face = mesh.faces[face_index as usize];
                faces_to_remove.insert(face_index);

                let not_on_edge = |v: i32| v != v_start_index && v != v_end_index;
                let v_opposite_index = if not_on_edge(original_face.v0) {
                    original_face.v0
                } else if not_on_edge(original_face.v1) {
                    original_face.v1
                } else if not_on_edge(original_face.v2) {
                    original_face.v2
                } else {
                    continue;
                };

                let (f0, f1, f2) = (original_face.v0, original_face.v1, original_face.v2);
                let edge_forward = if (f0 == v_start_index && f1 == v_end_index)
                    || (f1 == v_start_index && f2 == v_end_index)
                    || (f2 == v_start_index && f0 == v_end_index)
                {
                    true
                } else if (f0 == v_end_index && f1 == v_start_index)
                    || (f1 == v_end_index && f2 == v_start_index)
                    || (f2 == v_end_index && f0 == v_start_index)
                {
                    false
                } else {
                    continue;
                };

                let get_actual_index = |idx: i32| -> i32 {
                    if idx < 0 {
                        vertex_index_mapping.get(&idx).copied().unwrap_or(idx)
                    } else {
                        idx
                    }
                };

                // the position of an index that may be one of the not yet added vertices
                let position_of = |idx: i32| -> Vector3Float {
                    let vertex_count = mesh.vertices.len() as i32;
                    if idx < 0 {
                        new_vertices_to_add[(-idx - 1) as usize]
                    } else if idx >= vertex_count {
                        new_vertices_to_add[(idx - vertex_count) as usize]
                    } else {
                        mesh.vertices[idx as usize]
                    }
                };

                for i in 0..split_sequence.len() - 1 {
                    let seg_start = get_actual_index(split_sequence[i]);
                    let seg_end = get_actual_index(split_sequence[i + 1]);

                    if seg_start == seg_end
                        || seg_start == v_opposite_index
                        || seg_end == v_opposite_index
                    {
                        continue;
                    }

                    let p0 = position_of(seg_start);
                    let p1 = position_of(seg_end);
                    let p2 = mesh.vertices[v_opposite_index as usize];

                    let area = p0.get_area(p1, p2);
                    if area < epsilon_sq {
                        continue;
                    }

                    // normals are calculated once the new vertices are in (divergence 100)
                    if edge_forward {
                        new_faces_to_add.push(Face::with_normal(
                            seg_start,
                            seg_end,
                            v_opposite_index,
                            Vector3Float::ZERO,
                        ));
                    } else {
                        new_faces_to_add.push(Face::with_normal(
                            seg_end,
                            seg_start,
                            v_opposite_index,
                            Vector3Float::ZERO,
                        ));
                    }
                    new_face_source.push(face_index);
                }
            }

            edges_split += 1;
            reporter(
                0.7 + (edges_split as f64 / edges_to_split_count as f64 * 0.25),
                &format!("Splitting Edges ({edges_split}/{edges_to_split_count})"),
            );
        }

        if faces_to_remove.is_empty()
            && new_faces_to_add.is_empty()
            && new_vertices_to_add.is_empty()
        {
            return !vertices_to_move.is_empty();
        }

        reporter(0.95, "Updating Mesh");

        mesh.vertices.extend_from_slice(&new_vertices_to_add);

        for face in new_faces_to_add.iter_mut() {
            if face.v0 < 0 {
                face.v0 = vertex_index_mapping[&face.v0];
            }
            if face.v1 < 0 {
                face.v1 = vertex_index_mapping[&face.v1];
            }
            if face.v2 < 0 {
                face.v2 = vertex_index_mapping[&face.v2];
            }
        }

        let mut final_faces = FaceList::new();
        let mut source_face_index: Vec<i32> =
            Vec::with_capacity(mesh.faces.len() + new_faces_to_add.len());
        for (i, face) in mesh.faces.iter().enumerate() {
            if !faces_to_remove.contains(&(i as i32)) {
                final_faces.push(*face);
                source_face_index.push(i as i32);
            }
        }

        for face in new_faces_to_add.iter_mut() {
            face.calculate_normal(&mesh.vertices);
        }

        final_faces.extend(new_faces_to_add);
        source_face_index.extend(new_face_source);
        let source_faces = std::mem::replace(&mut mesh.faces, final_faces);

        // the added faces are fans cut out of the faces they replaced, so their UVs come from
        // interpolating the source face's mapping at the new corners
        let vertices = mesh.vertices.clone();
        mesh.remap_face_data(&source_face_index, Some((&source_faces, &vertices)));
        mesh.mark_as_changed();
    }

    true
}
