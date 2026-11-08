//! Face and mesh queries as `Face` and `Mesh` methods: copying, planes, coplanar faces, edges,
//! manifoldness and whether the faces close into one consistently wound surface, unused-vertex
//! and degenerate-face removal.
//!
//! Splitting a face and its cut line live in `face_split.rs`, `split_on_planes` in
//! `mesh_editing.rs` and T-junction repair in `t_junctions.rs`; the texture helpers (placing
//! and removing textures, the max-plane projection, and the single-face projection
//! `place_texture_on_face_projected`) are in `face_data.rs`.

use std::collections::{HashMap, HashSet};

use solidmesh_math::Color;
use solidmesh_math::{AxisAlignedBoundingBox, Plane, Vector3, Vector3Float};

use crate::face::{Face, FaceList};
use crate::mesh::Mesh;
use crate::mesh_edge_graph::{MeshEdge, MeshEdgeGraph, VertexFaceList};

/// A face's first corner and its normal.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct PositionNormal {
    pub position: Vector3Float,
    pub normal: Vector3Float,
}

impl Face {
    /// The bounds of the face's three corners in `mesh`.
    pub fn get_axis_aligned_bounding_box(&self, mesh: &Mesh) -> AxisAlignedBoundingBox {
        let mut bounds = AxisAlignedBoundingBox::empty();
        for v in [self.v0, self.v1, self.v2] {
            bounds.expand_to_include_point(Vector3::from_vector3_float(mesh.vertices[v as usize]));
        }
        bounds
    }

    /// The face's area as `a * c * sin(B) / 2`, from two edge lengths and the angle between
    /// them.
    pub fn get_area(&self, mesh: &Mesh) -> f64 {
        let p0 = mesh.vertices[self.v0 as usize];
        let p1 = mesh.vertices[self.v1 as usize];
        let p2 = mesh.vertices[self.v2 as usize];
        let xy = Vector3::new(
            f64::from(p1.x - p0.x),
            f64::from(p1.y - p0.y),
            f64::from(p1.z - p0.z),
        );
        let xz = Vector3::new(
            f64::from(p2.x - p0.x),
            f64::from(p2.y - p0.y),
            f64::from(p2.z - p0.z),
        );

        let a = f64::from((p0 - p1).length());
        let c = f64::from((p0 - p2).length());
        let b = Vector3::calculate_angle(xy, xz);

        (a * c * libm::sin(b)) / 2.0
    }
}

impl Mesh {
    /// Appends `copy_from`'s vertices as they are and its faces with
    /// their normals, unwelded, with its face textures and colors at their new indices.
    pub fn copy_faces(&mut self, copy_from: &Mesh) {
        let v_start = self.vertices.len() as i32;
        let f_start = self.faces.len();
        self.vertices.extend_from_slice(&copy_from.vertices);
        for face in copy_from.faces.iter() {
            self.faces.add_with_normal(
                face.v0 + v_start,
                face.v1 + v_start,
                face.v2 + v_start,
                face.normal,
            );
        }

        // the copied faces land at f_start and up, so their texture keys move with them
        for (key, value) in &copy_from.face_textures {
            self.face_textures
                .insert(f_start as i32 + key, value.clone());
        }

        let total_faces = f_start + copy_from.faces.len();
        if let Some(source_colors) = &copy_from.face_colors {
            let mut colors = self.face_colors.take().unwrap_or_default();
            if colors.len() < total_faces {
                colors.resize(total_faces, Color::default());
            }
            let count = source_colors.len().min(copy_from.faces.len());
            colors[f_start..f_start + count].copy_from_slice(&source_colors[..count]);
            self.face_colors = Some(colors);
        } else if let Some(colors) = &mut self.face_colors {
            // target has face colors but source doesn't - extend with transparent (resize
            // also truncates a longer array)
            colors.resize(total_faces, Color::default());
        }
    }

    /// An independent copy of the mesh - its own vertices, faces and per-face data - so that
    /// editing the copy can never reach back into the source.
    pub fn copy(&self) -> Mesh {
        let mut copy = Mesh::from_vertices_and_faces(&self.vertices, &self.faces);
        copy.face_colors = self.face_colors.clone();
        // A new map so texturing one mesh does not texture the other. The images are shared,
        // which is safe because nothing edits one in place.
        copy.face_textures = self.face_textures.clone();
        copy
    }

    /// The plane through the face's three corners.
    pub fn get_plane(&self, face_index: usize) -> Plane {
        let face = self.faces[face_index];
        let verts = &self.vertices;
        Plane::from_points(
            Vector3::from_vector3_float(verts[face.v0 as usize]),
            Vector3::from_vector3_float(verts[face.v1 as usize]),
            Vector3::from_vector3_float(verts[face.v2 as usize]),
        )
    }

    /// Each face's first corner and normal.
    pub fn to_position_normal_array(&self) -> Vec<PositionNormal> {
        self.faces
            .iter()
            .map(|face| PositionNormal {
                position: self.vertices[face.v0 as usize],
                normal: face.normal,
            })
            .collect()
    }

    /// The faces lying in `plane`: normal within .001 and distance from the origin within .001.
    pub fn get_coplanar_faces(&self, plane: Plane) -> Vec<usize> {
        let normal_tolerance = 0.001;
        let distance_tolerance = 0.001;

        let normal_tolerance_squared = normal_tolerance * normal_tolerance;
        let mut found = Vec::new();
        for (face_index, face) in self.faces.iter().enumerate() {
            let face_normal = face.normal;
            let distance_from_origin = face_normal.dot(self.vertices[face.v0 as usize]);

            if (plane.distance_from_origin - f64::from(distance_from_origin)).abs()
                <= distance_tolerance
                && (plane.normal - Vector3::from_vector3_float(face_normal)).length_squared()
                    <= normal_tolerance_squared
            {
                found.push(face_index);
            }
        }
        found
    }

    /// The faces coplanar with face `face_index` (including itself).
    pub fn get_coplanar_faces_of_face(&self, face_index: usize) -> Vec<usize> {
        self.get_coplanar_faces(self.get_plane(face_index))
    }

    /// One face list per vertex.
    pub fn get_vertex_face_lists(&self) -> Vec<VertexFaceList> {
        VertexFaceList::create_vertex_face_list(self)
    }

    /// One [`MeshEdge`] per unique edge.
    pub fn get_mesh_edges(&self) -> Vec<MeshEdge> {
        MeshEdge::create_mesh_edge_list(self)
    }

    /// The unique edges and their adjacent faces as flat arrays. Much cheaper than
    /// [`Mesh::get_mesh_edges`] - prefer it wherever the edges are only walked, not retained.
    pub fn get_mesh_edge_graph(&self) -> MeshEdgeGraph {
        MeshEdgeGraph::create(self)
    }

    /// One face's area by Heron's formula.
    pub fn get_face_surface_area(&self, face_index: usize) -> f64 {
        let face = self.faces[face_index];
        let verts = &self.vertices;
        let a = f64::from((verts[face.v0 as usize] - verts[face.v1 as usize]).length());
        let b = f64::from((verts[face.v1 as usize] - verts[face.v2 as usize]).length());
        let c = f64::from((verts[face.v2 as usize] - verts[face.v0 as usize]).length());
        let p = 0.5 * (a + b + c);
        (p * (p - a) * (p - b) * (p - c)).sqrt()
    }

    /// Every edge that does not have exactly two faces.
    pub fn get_non_manifold_edges(&self) -> Vec<MeshEdge> {
        // Walk the flat graph and only materialize MeshEdge objects for the (usually very few)
        // bad edges.
        let graph = self.get_mesh_edge_graph();
        (0..graph.edge_count())
            .filter(|&e| graph.get_face_count(e) != 2)
            .map(|e| {
                MeshEdge::with_faces(
                    graph.get_vertex0(e),
                    graph.get_vertex1(e),
                    graph.get_faces(e).to_vec(),
                )
            })
            .collect()
    }

    /// True when every edge has exactly two faces (vacuously true with no faces).
    pub fn is_manifold(&self) -> bool {
        let graph = self.get_mesh_edge_graph();
        (0..graph.edge_count()).all(|e| graph.get_face_count(e) == 2)
    }

    /// True when the faces close up into one consistently wound surface: every edge, on shared
    /// vertex indices, is walked exactly once in each direction by the faces around it, and no
    /// face repeats a vertex. Stricter than [`Self::is_manifold`], which counts faces per edge
    /// but not which way they walk it, so a face flipped against its neighbours passes there.
    /// False with no faces, which bound nothing.
    pub fn is_closed_consistently_wound(&self) -> bool {
        if self.faces.is_empty() {
            return false;
        }
        let mut walks: HashMap<(i32, i32), u32> = HashMap::new();
        for face in self.faces.iter() {
            if face.v0 == face.v1 || face.v1 == face.v2 || face.v2 == face.v0 {
                return false;
            }
            for edge in [(face.v0, face.v1), (face.v1, face.v2), (face.v2, face.v0)] {
                *walks.entry(edge).or_insert(0) += 1;
            }
        }
        // Only lookups: the map's order never reaches an output.
        walks
            .iter()
            .all(|(&(a, b), &count)| count == 1 && walks.get(&(b, a)) == Some(&1))
    }

    /// Drops the vertices no face uses and renumbers the faces (normals recomputed).
    pub fn remove_unused_vertices(&mut self) {
        let mut used_vertices = HashSet::new();
        for face in self.faces.iter() {
            used_vertices.insert(face.v0);
            used_vertices.insert(face.v1);
            used_vertices.insert(face.v2);
        }

        let mut new_vertices = Vec::new();
        let mut old_to_new_index = vec![-1; self.vertices.len()];
        for (i, &vertex) in self.vertices.iter().enumerate() {
            if used_vertices.contains(&(i as i32)) {
                old_to_new_index[i] = new_vertices.len() as i32;
                new_vertices.push(vertex);
            }
        }

        let mut new_faces = FaceList::new();
        for face in self.faces.iter() {
            new_faces.add(
                old_to_new_index[face.v0 as usize],
                old_to_new_index[face.v1 as usize],
                old_to_new_index[face.v2 as usize],
                &new_vertices,
            );
        }

        self.vertices = new_vertices;
        self.faces = new_faces;
        self.mark_as_changed();
    }

    /// Drops faces with a repeated vertex index or an area below `min_face_area` (compared in
    /// f32, against the face's f32 area).
    pub fn remove_degenerate_faces(&mut self, min_face_area: f64) {
        let mut new_faces = FaceList::new();
        let mut source_face_index: Vec<i32> = Vec::with_capacity(self.faces.len());
        let min_area_f = min_face_area as f32;

        for (face_idx, face) in self.faces.iter().enumerate() {
            if face.v0 != face.v1 && face.v1 != face.v2 && face.v2 != face.v0 {
                let p0 = self.vertices[face.v0 as usize];
                let p1 = self.vertices[face.v1 as usize];
                let p2 = self.vertices[face.v2 as usize];
                if p0.get_area(p1, p2) >= min_area_f {
                    new_faces.add(face.v0, face.v1, face.v2, &self.vertices);
                    source_face_index.push(face_idx as i32);
                }
            }
        }

        if self.faces.len() != new_faces.len() {
            self.faces = new_faces;
            // the surviving faces keep their geometry, they just move down the list
            self.remap_face_data(&source_face_index, None);
            self.mark_as_changed();
        }
    }
}
