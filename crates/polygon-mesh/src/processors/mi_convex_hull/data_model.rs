//! The quickhull's data model and pooled collections (derived from MIConvexHull), for the 3D
//! hull path: faces, the connectors that pair new cone edges, deferred faces, the unprocessed
//! face list and the connector hash buckets.
//!
//! Faces and connectors live in arenas, linked by index, and are reused through free stacks
//! popped last-in first-out, because the face index decides the connector hash table order and
//! the order the hull faces come out in. Vertex buffers are arena ids too: finished faces share
//! one empty buffer, which can be swapped into the working beyond buffer, so two ids can alias.

/// The number of connector hash buckets (a prime).
pub(super) const CONNECTOR_TABLE_SIZE: u32 = 2017;

/// One hull face: its vertices, neighbours, plane and beyond vertices.
#[derive(Debug, Clone)]
pub(super) struct FaceInternal {
    pub adjacent_faces: [i32; 3],
    pub furthest_vertex: usize,
    pub index: usize,
    pub in_list: bool,
    pub is_normal_flipped: bool,
    pub next: Option<usize>,
    pub previous: Option<usize>,
    pub normal: [f64; 3],
    pub offset: f64,
    pub vertices: [i32; 3],
    /// Id of the buffer of vertices beyond this face.
    pub vertices_beyond: usize,
}

impl FaceInternal {
    pub(super) fn new(index: usize, beyond_list: usize) -> Self {
        FaceInternal {
            adjacent_faces: [0; 3],
            furthest_vertex: 0,
            index,
            in_list: false,
            is_normal_flipped: false,
            next: None,
            previous: None,
            normal: [0.0; 3],
            offset: 0.0,
            vertices: [0; 3],
            vertices_beyond: beyond_list,
        }
    }
}

/// One edge of a new cone face, waiting for its twin.
#[derive(Debug, Clone, Default)]
pub(super) struct FaceConnector {
    pub edge_index: usize,
    pub face: Option<usize>,
    pub hash_code: u32,
    pub next: Option<usize>,
    pub previous: Option<usize>,
    pub vertices: [i32; 2],
}

impl FaceConnector {
    /// The face's vertices without `edge_index`, and their hash.
    pub(super) fn update(&mut self, face: &FaceInternal, edge_index: usize) {
        self.face = Some(face.index);
        self.edge_index = edge_index;
        let mut hash_code: u32 = 23;
        let mut c = 0;
        for (i, &v) in face.vertices.iter().enumerate() {
            if i == edge_index {
                continue;
            }
            self.vertices[c] = v;
            c += 1;
            // wrapping arithmetic: the hash only has to agree for the same vertices
            hash_code =
                hash_code.wrapping_add(31u32.wrapping_mul(hash_code).wrapping_add(v as u32));
        }
        self.hash_code = hash_code;
    }

    /// Whether two connectors are twins: the same hash and the same two vertices.
    pub(super) fn are_connectable(a: &FaceConnector, b: &FaceConnector) -> bool {
        a.hash_code == b.hash_code && a.vertices == b.vertices
    }
}

/// A cone face whose adjacency is committed after the whole cone is built.
#[derive(Debug, Clone, Copy)]
pub(super) struct DeferredFace {
    pub face: usize,
    pub face_index: usize,
    pub pivot: usize,
    pub pivot_index: usize,
    pub old_face: usize,
}

/// The unprocessed faces, an intrusive list through the faces' next/previous.
#[derive(Debug, Default)]
pub(super) struct FaceList {
    last: Option<usize>,
    pub first: Option<usize>,
}

impl FaceList {
    fn add_first(&mut self, faces: &mut [FaceInternal], face: usize) {
        faces[face].in_list = true;
        if let Some(first) = self.first {
            faces[first].previous = Some(face);
        }
        faces[face].next = self.first;
        self.first = Some(face);
    }

    /// A face with more beyond vertices than the first goes to the front.
    pub(super) fn add(&mut self, faces: &mut [FaceInternal], buffers: &[Vec<i32>], face: usize) {
        let count = |faces: &[FaceInternal], f: usize| buffers[faces[f].vertices_beyond].len();
        if faces[face].in_list {
            // in the list, so `first` is set
            if let Some(first) = self.first {
                if count(faces, first) < count(faces, face) {
                    self.remove(faces, face);
                    self.add_first(faces, face);
                }
            }
            return;
        }
        faces[face].in_list = true;
        match self.first {
            Some(first) if count(faces, first) < count(faces, face) => {
                faces[first].previous = Some(face);
                faces[face].next = Some(first);
                self.first = Some(face);
            }
            _ => {
                if let Some(last) = self.last {
                    faces[last].next = Some(face);
                }
                faces[face].previous = self.last;
                self.last = Some(face);
                if self.first.is_none() {
                    self.first = Some(face);
                }
            }
        }
    }

    /// Unlinks `face` from the list (nothing when it is not in it).
    pub(super) fn remove(&mut self, faces: &mut [FaceInternal], face: usize) {
        if !faces[face].in_list {
            return;
        }
        faces[face].in_list = false;
        let (previous, next) = (faces[face].previous, faces[face].next);
        match previous {
            Some(p) => faces[p].next = next,
            None => self.first = next,
        }
        match next {
            Some(n) => faces[n].previous = previous,
            None => self.last = previous,
        }
        faces[face].next = None;
        faces[face].previous = None;
    }
}

/// One bucket of the connector hash table.
#[derive(Debug, Default, Clone)]
pub(super) struct ConnectorList {
    last: Option<usize>,
    pub first: Option<usize>,
}

impl ConnectorList {
    /// Appends at the end.
    pub(super) fn add(&mut self, connectors: &mut [FaceConnector], element: usize) {
        if let Some(last) = self.last {
            connectors[last].next = Some(element);
        }
        connectors[element].previous = self.last;
        self.last = Some(element);
        if self.first.is_none() {
            self.first = Some(element);
        }
    }

    /// Unlinks `connector` from the bucket.
    pub(super) fn remove(&mut self, connectors: &mut [FaceConnector], connector: usize) {
        let (previous, next) = (connectors[connector].previous, connectors[connector].next);
        match previous {
            Some(p) => connectors[p].next = next,
            None => self.first = next,
        }
        match next {
            Some(n) => connectors[n].previous = previous,
            None => self.last = previous,
        }
        connectors[connector].next = None;
        connectors[connector].previous = None;
    }
}
