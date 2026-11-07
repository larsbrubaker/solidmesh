//! The vertices of one placement of a ring's loops (`ring_loops.rs` prepares the loops): one
//! vertex per loop point, so a point the loops pass more than once has a vertex per pass, one per
//! filled wedge there. Every solid built from a path - an extrusion's rings, a revolve's columns,
//! a sweep's stations - places its rings this way and builds its walls and caps on these vertices,
//! so faces that meet along a ring meet on the same vertices and the solid is closed by
//! construction. [`RingVertices::cap_faces`] fills the loops flat over `cap_fill`, choosing at a
//! point passed more than once the vertex of the wedge each fill triangle lies in.

use std::collections::HashMap;

use clipper2_rust::{Path64, Paths64};

use super::cap_fill::{cap_faces_by_corner, Triangle};

/// The vertices of one placement of a ring's loops.
pub struct RingVertices {
    /// `indices[loop][point]` is the vertex of that point.
    pub indices: Vec<Vec<i32>>,
    /// The vertex at each 2D position the ring passes once.
    by_position: HashMap<(i64, i64), i32>,
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

impl RingVertices {
    /// Places `loops`: `vertex_for(loop, point, passed_more_than_once)` gives the vertex of each
    /// loop point, called once per point in loop order, and says whether the loops pass that
    /// point's position more than once (where each pass must have a vertex of its own).
    pub fn place(loops: &Paths64, mut vertex_for: impl FnMut(usize, usize, bool) -> i32) -> Self {
        let mut passes: HashMap<(i64, i64), usize> = HashMap::new();
        for point in loops.iter().flatten() {
            *passes.entry((point.x, point.y)).or_insert(0) += 1;
        }
        let mut by_position = HashMap::new();
        let mut wedges: HashMap<(i64, i64), Vec<Wedge>> = HashMap::new();
        let mut indices = Vec::with_capacity(loops.len());
        for (l, path) in loops.iter().enumerate() {
            let count = path.len();
            let mut loop_indices = Vec::with_capacity(count);
            for (i, point) in path.iter().enumerate() {
                let key = (point.x, point.y);
                let shared = passes[&key] > 1;
                let vertex = vertex_for(l, i, shared);
                if shared {
                    let (prev, next) = (path[(i + count - 1) % count], path[(i + 1) % count]);
                    wedges.entry(key).or_default().push(Wedge {
                        vertex,
                        leave: ((next.x - point.x) as i128, (next.y - point.y) as i128),
                        arrive: ((prev.x - point.x) as i128, (prev.y - point.y) as i128),
                    });
                } else {
                    by_position.insert(key, vertex);
                }
                loop_indices.push(vertex);
            }
            indices.push(loop_indices);
        }
        RingVertices {
            indices,
            by_position,
            wedges,
        }
    }

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
    pub fn corner_vertex(&self, triangle: Triangle, k: usize) -> Option<i32> {
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

    /// The faces filling `which` of the ring's loops (`loops`, the loops it was placed from) flat.
    /// Every surface on the far side of the ring from the cap walks its edges one way: a cap that
    /// walks them in loop order (`walk_forward`) closes the solid lying on the side the loops'
    /// +z normal points away from. The fill joins only the ring's own vertices (see
    /// [`cap_faces_by_corner`]).
    pub fn cap_faces(&self, loops: &Paths64, which: &[usize], walk_forward: bool) -> Vec<[i32; 3]> {
        let paths: Vec<&Path64> = which.iter().map(|&i| &loops[i]).collect();
        let indices: Vec<&[i32]> = which.iter().map(|&i| self.indices[i].as_slice()).collect();
        cap_faces_by_corner(
            &paths,
            &indices,
            |triangle, k| self.corner_vertex(triangle, k),
            walk_forward,
        )
    }
}
