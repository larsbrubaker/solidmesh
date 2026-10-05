//! A mesh's unique edges and the faces on each, plus the per-vertex face lists. Ports agg-sharp
//! `PolygonMesh/MeshEdgeGraph.cs`, `MeshEdge.cs` and `VertexFaceList.cs`.
//!
//! [`MeshEdgeGraph`] exists because the object-per-edge view ([`MeshEdge`]) is far too
//! expensive for the outline render path, which walks every edge once and keeps nothing.
//! Measured in C# on a 5,113,368 face mesh (7,670,061 unique edges), the old
//! dictionary-of-lists build allocated 2,279 MB and retained 824 MB. Here the whole thing is
//! four arrays and one sort: three (edgeKey, faceIndex) slots per face sorted by key, then a
//! walk over equal-key runs. Adjacency is stored CSR style: the faces on edge e are
//! `face_indices[face_starts[e] .. face_starts[e + 1]]`.

use crate::mesh::Mesh;

/// The unique edges of a mesh, and the faces adjacent to each edge, held as flat arrays.
#[derive(Debug, Clone, Default)]
pub struct MeshEdgeGraph {
    edge_vertex0: Vec<i32>,
    edge_vertex1: Vec<i32>,
    // Length edge_count + 1, so the run for the last edge needs no special case.
    face_starts: Vec<i32>,
    face_indices: Vec<i32>,
}

impl MeshEdgeGraph {
    /// The number of unique edges in the mesh.
    pub fn edge_count(&self) -> i32 {
        self.edge_vertex0.len() as i32
    }

    /// The lower of the two vertex indices of the given edge.
    pub fn get_vertex0(&self, edge_index: i32) -> i32 {
        self.edge_vertex0[edge_index as usize]
    }

    /// The higher of the two vertex indices of the given edge.
    pub fn get_vertex1(&self, edge_index: i32) -> i32 {
        self.edge_vertex1[edge_index as usize]
    }

    /// How many faces share the given edge. 2 is manifold, 1 is a boundary edge, more is
    /// non-manifold.
    pub fn get_face_count(&self, edge_index: i32) -> i32 {
        let e = edge_index as usize;
        self.face_starts[e + 1] - self.face_starts[e]
    }

    /// One of the faces sharing the given edge, `face_offset` in `0..get_face_count()`. Faces
    /// are in ascending face index order.
    pub fn get_face(&self, edge_index: i32, face_offset: i32) -> i32 {
        self.face_indices[(self.face_starts[edge_index as usize] + face_offset) as usize]
    }

    /// All the faces sharing the given edge, in ascending face index order, without copying.
    pub fn get_faces(&self, edge_index: i32) -> &[i32] {
        let e = edge_index as usize;
        &self.face_indices[self.face_starts[e] as usize..self.face_starts[e + 1] as usize]
    }

    /// Builds the edge graph for a mesh. O(n log n) in the face count, with no per-edge heap
    /// objects.
    pub fn create(mesh: &Mesh) -> MeshEdgeGraph {
        let face_count = mesh.faces.len();
        let slot_count = face_count * 3;

        // One slot per face-edge. The key packs the edge's two vertex indices low-first into a
        // single u64 so a plain numeric sort brings every use of the same edge together. The
        // face index rides along as the tie-break, which also leaves every edge's faces in
        // ascending order (C# sorts unstably and then insertion-sorts each run to get there).
        let mut slots: Vec<(u64, i32)> = Vec::with_capacity(slot_count);
        for (face_index, face) in mesh.faces.iter().enumerate() {
            let face_index = face_index as i32;
            slots.push((edge_key(face.v0, face.v1), face_index));
            slots.push((edge_key(face.v1, face.v2), face_index));
            slots.push((edge_key(face.v2, face.v0), face_index));
        }
        slots.sort_unstable();

        let mut graph = MeshEdgeGraph {
            face_indices: Vec::with_capacity(slot_count),
            ..MeshEdgeGraph::default()
        };
        for (slot, &(key, face)) in slots.iter().enumerate() {
            if slot == 0 || key != slots[slot - 1].0 {
                graph.edge_vertex0.push((key >> 32) as i32);
                graph.edge_vertex1.push((key & 0xFFFF_FFFF) as i32);
                graph.face_starts.push(slot as i32);
            }
            graph.face_indices.push(face);
        }
        graph.face_starts.push(slot_count as i32);
        graph
    }
}

/// The two vertex indices packed low-first (C# casts through `uint`).
fn edge_key(vertex_a: i32, vertex_b: i32) -> u64 {
    let (low, high) = if vertex_a < vertex_b {
        (vertex_a, vertex_b)
    } else {
        (vertex_b, vertex_a)
    };
    (u64::from(low as u32) << 32) | u64::from(high as u32)
}

/// One unique edge of a mesh, together with the faces that share it.
///
/// This is the object-per-edge view of a mesh's edges. It costs a heap allocation per edge, so
/// code that only walks the edges once should use [`MeshEdgeGraph`] instead.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MeshEdge {
    faces: Vec<i32>,
    pub vertex0_index: i32,
    pub vertex1_index: i32,
}

impl MeshEdge {
    /// An edge with no faces.
    pub fn new(vertex0_index: i32, vertex1_index: i32) -> Self {
        Self::with_faces(vertex0_index, vertex1_index, Vec::new())
    }

    pub(crate) fn with_faces(vertex0_index: i32, vertex1_index: i32, faces: Vec<i32>) -> Self {
        MeshEdge {
            faces,
            vertex0_index,
            vertex1_index,
        }
    }

    /// The indices of all the faces that share this edge.
    pub fn faces(&self) -> &[i32] {
        &self.faces
    }

    /// A `MeshEdge` for every unique edge of the mesh, built on [`MeshEdgeGraph`].
    pub fn create_mesh_edge_list(mesh: &Mesh) -> Vec<MeshEdge> {
        let graph = MeshEdgeGraph::create(mesh);
        (0..graph.edge_count())
            .map(|e| {
                MeshEdge::with_faces(
                    graph.get_vertex0(e),
                    graph.get_vertex1(e),
                    graph.get_faces(e).to_vec(),
                )
            })
            .collect()
    }
}

/// The faces that use one vertex.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct VertexFaceList {
    faces: Vec<i32>,
}

impl VertexFaceList {
    /// The indices of all the faces that share this vertex.
    pub fn faces(&self) -> &[i32] {
        &self.faces
    }

    /// One list per vertex of `mesh`, each holding the faces that use it, in face order.
    pub fn create_vertex_face_list(mesh: &Mesh) -> Vec<VertexFaceList> {
        let mut mesh_vertex = vec![VertexFaceList::default(); mesh.vertices.len()];
        for (i, face) in mesh.faces.iter().enumerate() {
            mesh_vertex[face.v0 as usize].faces.push(i as i32);
            mesh_vertex[face.v1 as usize].faces.push(i as i32);
            mesh_vertex[face.v2 as usize].faces.push(i as i32);
        }
        mesh_vertex
    }
}
