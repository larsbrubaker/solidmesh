//! Contouring a dual contouring octree into a [`Mesh`], clean-room from Ju et al., "Dual
//! Contouring of Hermite Data" (SIGGRAPH 2002), section 3.2: the cell / face / edge recursion
//! visits every minimal edge of the octree once with the (up to four distinct) leaves around it,
//! and each edge whose ends differ in sign gets a quad joining those leaves' vertices. The
//! minimal edge is the one of the smallest leaf around it; where a larger leaf appears twice the
//! quad degenerates to a triangle. Quads are wound so the face points from inside to outside.
//! See `octree` for how the tree and its vertices are built.

use std::collections::HashMap;

use vector_math::Vector3Float;

use super::octree::{OctreeNode, OctreeNodeKind};
use crate::Mesh;

/// The four cells around an edge along axis `e`, by their side (0 low, 1 high) on the axes
/// `p = (e + 1) % 3` and `q = (e + 2) % 3`: counter-clockwise seen from `+e`.
const AROUND_EDGE: [[usize; 2]; 4] = [[0, 0], [1, 0], [1, 1], [0, 1]];

/// The child at `bits` (per axis), or the node itself when it has no children.
fn child(node: &OctreeNode, bits: [usize; 3]) -> &OctreeNode {
    match &node.kind {
        OctreeNodeKind::Internal(children) => &children[bits[0] | (bits[1] << 1) | (bits[2] << 2)],
        _ => node,
    }
}

fn is_internal(node: &OctreeNode) -> bool {
    matches!(node.kind, OctreeNodeKind::Internal(_))
}

/// The contour of the octree as triangles. Each leaf vertex used by a face appears once.
pub fn generate_mesh_from_octree(root: &OctreeNode) -> Mesh {
    let mut contour = Contour {
        mesh: Mesh::new(),
        vertex_ids: HashMap::new(),
    };
    contour.cell_proc(root);
    contour.mesh
}

struct Contour {
    mesh: Mesh,
    /// Each leaf's vertex index, keyed by the node's address (the tree is not moved while
    /// contouring).
    vertex_ids: HashMap<*const OctreeNode, i32>,
}

impl Contour {
    /// Every minimal edge inside `node`.
    fn cell_proc(&mut self, node: &OctreeNode) {
        let OctreeNodeKind::Internal(children) = &node.kind else {
            return;
        };
        for c in children.iter() {
            self.cell_proc(c);
        }
        for a in 0..3 {
            let (u, v) = ((a + 1) % 3, (a + 2) % 3);
            for i in 0..2 {
                for j in 0..2 {
                    let mut low = [0; 3];
                    low[u] = i;
                    low[v] = j;
                    let mut high = low;
                    high[a] = 1;
                    self.face_proc([child(node, low), child(node, high)], a);
                }
            }
        }
        for e in 0..3 {
            let (p, q) = ((e + 1) % 3, (e + 2) % 3);
            for i in 0..2 {
                let cells = AROUND_EDGE.map(|[sp, sq]| {
                    let mut bits = [0; 3];
                    bits[e] = i;
                    bits[p] = sp;
                    bits[q] = sq;
                    child(node, bits)
                });
                self.edge_proc(cells, e);
            }
        }
    }

    /// Every minimal edge on the face between `nodes[0]` (low) and `nodes[1]` (high) along `a`.
    fn face_proc(&mut self, nodes: [&OctreeNode; 2], a: usize) {
        if !nodes.iter().any(|n| is_internal(n)) {
            return;
        }
        let (u, v) = ((a + 1) % 3, (a + 2) % 3);
        for i in 0..2 {
            for j in 0..2 {
                let mut low = [0; 3];
                low[u] = i;
                low[v] = j;
                let mut high = low;
                low[a] = 1;
                high[a] = 0;
                self.face_proc([child(nodes[0], low), child(nodes[1], high)], a);
            }
        }
        for e in [u, v] {
            let w = if e == u { v } else { u };
            let (p, q) = ((e + 1) % 3, (e + 2) % 3);
            for i in 0..2 {
                let cells = AROUND_EDGE.map(|[sp, sq]| {
                    let mut side = [0; 3];
                    side[p] = sp;
                    side[q] = sq;
                    let mut bits = [0; 3];
                    bits[a] = 1 - side[a];
                    bits[w] = side[w];
                    bits[e] = i;
                    child(nodes[side[a]], bits)
                });
                self.edge_proc(cells, e);
            }
        }
    }

    /// Every minimal edge along the line shared by the four `nodes` (ordered as `AROUND_EDGE`).
    fn edge_proc(&mut self, nodes: [&OctreeNode; 4], e: usize) {
        if !nodes.iter().any(|n| is_internal(n)) {
            self.process_edge(nodes, e);
            return;
        }
        let (p, q) = ((e + 1) % 3, (e + 2) % 3);
        for i in 0..2 {
            let mut k = 0;
            let cells = nodes.map(|node| {
                let [sp, sq] = AROUND_EDGE[k];
                k += 1;
                let mut bits = [0; 3];
                bits[e] = i;
                bits[p] = 1 - sp;
                bits[q] = 1 - sq;
                child(node, bits)
            });
            self.edge_proc(cells, e);
        }
    }

    /// The quad (or triangle) for one minimal edge, if its ends differ in sign.
    fn process_edge(&mut self, nodes: [&OctreeNode; 4], e: usize) {
        if nodes
            .iter()
            .any(|n| !matches!(n.kind, OctreeNodeKind::Leaf { .. }))
        {
            return;
        }
        // The smallest cell's edge is the minimal edge.
        let mut smallest = 0;
        for k in 1..4 {
            if nodes[k].depth > nodes[smallest].depth {
                smallest = k;
            }
        }
        let (p, q) = ((e + 1) % 3, (e + 2) % 3);
        let [sp, sq] = AROUND_EDGE[smallest];
        let corner = |end: usize| {
            let mut bits = [0; 3];
            bits[e] = end;
            bits[p] = 1 - sp;
            bits[q] = 1 - sq;
            bits[0] | (bits[1] << 1) | (bits[2] << 2)
        };
        let signs = nodes[smallest].signs;
        let low_inside = signs & (1 << corner(0)) != 0;
        let high_inside = signs & (1 << corner(1)) != 0;
        if low_inside == high_inside {
            return;
        }

        let mut ids = nodes.map(|n| self.vertex_id(n));
        if !low_inside {
            ids.reverse();
        }
        // Drop repeats (a larger leaf around the edge twice).
        // A leaf covers adjacent quadrants, so its repeats are neighbours around the ring.
        let mut ring: Vec<i32> = Vec::with_capacity(4);
        for id in ids {
            if ring.last() != Some(&id) {
                ring.push(id);
            }
        }
        if ring.len() > 1 && ring.first() == ring.last() {
            ring.pop();
        }
        match ring.len() {
            4 => {
                self.add_face(ring[0], ring[1], ring[2]);
                self.add_face(ring[0], ring[2], ring[3]);
            }
            3 => self.add_face(ring[0], ring[1], ring[2]),
            _ => {}
        }
    }

    fn add_face(&mut self, a: i32, b: i32, c: i32) {
        self.mesh.faces.add(a, b, c, &self.mesh.vertices);
    }

    /// The mesh index of a leaf's vertex, added on first use.
    fn vertex_id(&mut self, node: &OctreeNode) -> i32 {
        let key = node as *const OctreeNode;
        if let Some(&id) = self.vertex_ids.get(&key) {
            return id;
        }
        let v = node.vertex().unwrap_or(node.min);
        let id = self.mesh.vertices.len() as i32;
        self.mesh.vertices.push(Vector3Float::new(v.x, v.y, v.z));
        self.vertex_ids.insert(key, id);
        id
    }
}
