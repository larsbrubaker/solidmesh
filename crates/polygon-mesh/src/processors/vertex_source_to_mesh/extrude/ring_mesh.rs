//! The indexed mesh a bevelled extrusion is built in. Every bevel ring is placed once, as one
//! vertex per distinct point (one per pass where the ring's loops pass a point more than once, as
//! `ring_loops.rs` chains them: one per filled wedge there), and every face - the caps, the straight walls and the bands between
//! rings (`band.rs`) - is made of those vertex indices. Faces that meet along a ring meet on the
//! same vertices by construction, so the solid is closed without any welding of equal positions
//! afterwards.

use std::collections::HashMap;
use std::rc::Rc;

use clipper2_rust::{Path64, Paths64};
use vector_math::{Vector3, Vector3Float};

use crate::mesh::Mesh;
use crate::processors::cap_fill::{cap_faces_by_corner, Triangle};

/// Clipper units per mm, the scale every ring's integer points are in.
const UNITS_PER_MM: f64 = agg_path::data_converters_2d::DEFAULT_SCALING;

/// One ring placed in the mesh: its loops and the vertex of every loop point.
pub(super) struct PlacedRing {
    pub loops: Rc<Paths64>,
    /// `indices[loop][point]` is the vertex of that point.
    pub indices: Vec<Vec<i32>>,
    /// The vertex at each 2D position the ring passes once.
    pub by_position: HashMap<(i64, i64), i32>,
    /// At each 2D position the ring passes more than once, every pass's vertex and the filled
    /// wedge it bounds there.
    wedges: HashMap<(i64, i64), Vec<Wedge>>,
}

/// One pass through a point the ring passes more than once: its vertex, and the wedge of filled
/// area it bounds, from the direction it leaves in counter clockwise to the direction it arrived
/// from.
struct Wedge {
    vertex: i32,
    leave: (i128, i128),
    arrive: (i128, i128),
}

/// The z component of `a x b`.
fn turn(a: (i128, i128), b: (i128, i128)) -> i128 {
    a.0 * b.1 - a.1 * b.0
}

impl Wedge {
    /// Whether direction `v` points into the wedge, strictly between its sides.
    fn holds(&self, v: (i128, i128)) -> bool {
        let (a, b) = (self.leave, self.arrive);
        let sweep = turn(a, b);
        if sweep > 0 {
            turn(a, v) > 0 && turn(v, b) > 0
        } else if sweep == 0 {
            // a straight pass: the half plane on the left
            turn(a, v) > 0
        } else {
            // a wedge wider than a half turn is everything outside the closed one from b to a
            !(turn(b, v) >= 0 && turn(v, a) >= 0)
        }
    }
}

impl PlacedRing {
    /// The vertex at a 2D position, if the ring passes through it (where it passes more than
    /// once, the first pass's).
    pub fn vertex_at(&self, x: i64, y: i64) -> Option<i32> {
        self.by_position.get(&(x, y)).copied().or_else(|| {
            self.wedges
                .get(&(x, y))
                .and_then(|passes| passes.first())
                .map(|pass| pass.vertex)
        })
    }

    /// The vertex of corner `k` of a fill triangle: where the ring passes the corner's point more
    /// than once, the vertex of the wedge the triangle lies in.
    fn corner_vertex(&self, triangle: Triangle, k: usize) -> Option<i32> {
        let (x, y) = triangle[k];
        if let Some(&vertex) = self.by_position.get(&(x, y)) {
            return Some(vertex);
        }
        let passes = self.wedges.get(&(x, y))?;
        // the sum of the directions to the other two corners points into the triangle's corner,
        // which lies inside one wedge
        let toward = |p: (i64, i64)| ((p.0 - x) as i128, (p.1 - y) as i128);
        let (b, c) = (toward(triangle[(k + 1) % 3]), toward(triangle[(k + 2) % 3]));
        let inside = (b.0 + c.0, b.1 + c.1);
        passes
            .iter()
            .find(|pass| pass.holds(inside))
            .map(|pass| pass.vertex)
    }
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
        let mut passes: HashMap<(i64, i64), usize> = HashMap::new();
        for point in loops.iter().flatten() {
            *passes.entry((point.x, point.y)).or_insert(0) += 1;
        }
        let mut by_position = HashMap::new();
        let mut wedges: HashMap<(i64, i64), Vec<Wedge>> = HashMap::new();
        let mut indices = Vec::with_capacity(loops.len());
        for path in loops.iter() {
            let count = path.len();
            let mut loop_indices = Vec::with_capacity(count);
            for (i, point) in path.iter().enumerate() {
                let key = (point.x, point.y);
                let vertex = if passes[&key] > 1 {
                    let vertex = self.push_vertex(point.x, point.y, height);
                    let (prev, next) = (path[(i + count - 1) % count], path[(i + 1) % count]);
                    wedges.entry(key).or_default().push(Wedge {
                        vertex,
                        leave: ((next.x - point.x) as i128, (next.y - point.y) as i128),
                        arrive: ((prev.x - point.x) as i128, (prev.y - point.y) as i128),
                    });
                    vertex
                } else {
                    let vertex = self.push_vertex(point.x, point.y, height);
                    by_position.insert(key, vertex);
                    vertex
                };
                loop_indices.push(vertex);
            }
            indices.push(loop_indices);
        }
        PlacedRing {
            loops: Rc::clone(loops),
            indices,
            by_position,
            wedges,
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
        for (lower_loop, upper_loop) in lower.indices.iter().zip(&upper.indices) {
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
    /// The fill joins only the ring's own vertices (see [`cap_faces_by_corner`]).
    pub fn cap(&mut self, ring: &PlacedRing, loops: &[usize], walk_forward: bool) {
        let paths: Vec<&Path64> = loops.iter().map(|&i| &ring.loops[i]).collect();
        let indices: Vec<&[i32]> = loops.iter().map(|&i| ring.indices[i].as_slice()).collect();
        let faces = cap_faces_by_corner(
            &paths,
            &indices,
            |triangle, k| ring.corner_vertex(triangle, k),
            walk_forward,
        );
        self.add_all(&faces);
    }

    pub fn add_all(&mut self, faces: &[[i32; 3]]) {
        for &[a, b, c] in faces {
            self.add(a, b, c);
        }
    }
}
