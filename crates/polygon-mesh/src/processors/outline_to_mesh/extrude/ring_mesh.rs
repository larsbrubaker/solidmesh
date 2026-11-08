//! The indexed mesh a bevelled extrusion is built in. Every bevel ring is placed once, as one
//! vertex per distinct point (one per pass where the ring's loops pass a point more than once:
//! `processors/ring_vertices.rs`), and every face - the caps, the straight walls and the bands
//! between rings (`band.rs`) - is made of those vertex indices. Faces that meet along a ring meet on the
//! same vertices by construction, so the solid is closed without any welding of equal positions
//! afterwards.

use std::rc::Rc;

use clipper2_rust::Paths64;
use solidmesh_math::{Vector3, Vector3Float};

use crate::mesh::Mesh;
use crate::processors::ring_vertices::RingVertices;

/// Clipper units per mm, the scale every ring's integer points are in.
const UNITS_PER_MM: f64 = solidmesh_clipper::DEFAULT_SCALING;

/// One ring placed in the mesh: its loops and the vertex of every loop point.
pub(super) struct PlacedRing {
    pub loops: Rc<Paths64>,
    pub vertices: RingVertices,
}

/// The mesh being built and the rings placed in it.
pub(super) struct RingMesh {
    pub mesh: Mesh,
    /// Added to every ring's height: where z = 0 of the rings is in the mesh.
    base: f64,
}

impl RingMesh {
    /// A mesh whose rings are all placed `base` higher than the heights they are given.
    pub fn with_base(base: f64) -> Self {
        Self {
            mesh: Mesh::new(),
            base,
        }
    }

    /// Creates the ring's vertices: each distinct point once, or once per pass where the loops
    /// pass it more than once, at `height` (plus the base). The position is computed in f64 and
    /// rounded to the mesh's f32 once.
    pub fn place(&mut self, loops: &Rc<Paths64>, height: f64) -> PlacedRing {
        let vertices = RingVertices::place(loops, |l, i, _| {
            let point = loops[l][i];
            self.push_vertex(point.x, point.y, height)
        });
        PlacedRing {
            loops: Rc::clone(loops),
            vertices,
        }
    }

    /// Adds the vertex at a ring point, `height` (plus the base) up.
    fn push_vertex(&mut self, x: i64, y: i64, height: f64) -> i32 {
        self.mesh
            .vertices
            .push(Vector3Float::from_vector3(Vector3::new(
                x as f64 / UNITS_PER_MM,
                y as f64 / UNITS_PER_MM,
                height + self.base,
            )));
        self.mesh.vertices.len() as i32 - 1
    }

    /// Adds a triangle unless two of its corners are the same vertex.
    pub fn add(&mut self, a: i32, b: i32, c: i32) {
        if a != b && b != c && c != a {
            self.mesh.faces.add(a, b, c, &self.mesh.vertices);
        }
    }

    /// The straight wall between two placements of the same loops: a quad per edge, walking the
    /// lower ring's edges forward and the upper ring's backward, which is how every band meets
    /// the rings it joins (see [`RingMesh::cap`]). Each quad is split from its lower start corner
    /// to its upper end corner. Either diagonal makes the same flat wall, but a mesh built from
    /// the solid (a boolean, a bevel) follows the triangles it is given, and this is the split
    /// the walls of saved designs were made with.
    pub fn straight_wall(&mut self, lower: &PlacedRing, upper: &PlacedRing) {
        for (lower_loop, upper_loop) in lower.vertices.indices.iter().zip(&upper.vertices.indices) {
            let count = lower_loop.len();
            for i in 0..count {
                let next = (i + 1) % count;
                self.add(lower_loop[i], lower_loop[next], upper_loop[next]);
                self.add(lower_loop[i], upper_loop[next], upper_loop[i]);
            }
        }
    }

    /// Fills `loops` (some or all of `ring`'s loops, given as loop numbers) flat at the ring's
    /// height. Every surface above a ring walks its edges in loop order and every surface below
    /// walks them backward, so a cap that closes the solid below the ring (`walk_forward`, a top)
    /// walks them forward and one that closes the solid above it (a bottom) walks them backward.
    /// The fill joins only the ring's own vertices (see [`RingVertices::cap_faces`]).
    pub fn cap(&mut self, ring: &PlacedRing, loops: &[usize], walk_forward: bool) {
        let faces = ring.vertices.cap_faces(&ring.loops, loops, walk_forward);
        self.add_all(&faces);
    }

    pub fn add_all(&mut self, faces: &[[i32; 3]]) {
        for &[a, b, c] in faces {
            self.add(a, b, c);
        }
    }
}
