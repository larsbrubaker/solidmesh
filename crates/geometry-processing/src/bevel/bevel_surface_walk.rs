//! Walking a mesh surface in a straight line from an edge until it turns away - the measurement
//! the face fit asks "how much face is there to set back into" with, split out from the policy
//! that reads the answer. Ports `Bevel/BevelSurfaceWalk.cs`.
//!
//! Everything here is geometry that could be asked of any mesh - where a face ends, which way is
//! across an edge, what a triangle's normal is - and nothing here knows what a bevel or a setback
//! is. What faces STOP the walk is the one thing it is told rather than decides, because "is this
//! still one surface" is a reading the caller takes at its own tolerance.

use std::collections::HashMap;
use std::rc::Rc;
use std::sync::Arc;

use indexmap::IndexSet;
use polygon_mesh::{Mesh, MeshEdgeGraph};
use vector_math::Vector3;

use crate::topology::FeatureEdge;

/// How near a walk's exit has to be to a vertex before it counts as having landed ON it. Mesh
/// vertices are float, so an exit computed from two of them lands within a few ulps of the corner
/// rather than exactly on it.
const VERTEX_COINCIDENCE: f64 = 1e-6;

/// How nearly two faces have to point the same way before one is taken to lie in the other's
/// plane - about five degrees. Deliberately tight: a candidate that does not clear it is simply
/// not offered, and the walk falls back to the behaviour it had before this test existed.
const SAME_SURFACE: f64 = 0.996;

/// Where a [`BevelSurfaceWalk::walk`] stopped (C#'s return value and two `out` parameters).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SurfaceWalkStop {
    /// How far the walk travelled.
    pub travelled: f64,
    /// The mesh edge the walk stopped at, or -1 where it ran to the limit or fell off the mesh.
    pub blocking_edge: i32,
    /// The triangle the walk was in when it stopped, so the caller can ask what is on the OTHER
    /// side of `blocking_edge` - the surface the face turns onto. Meaningless (-1 or stale) when
    /// there is no blocking edge.
    pub stopped_in_face: i32,
}

/// C# `BevelSurfaceWalk`. Holds its mesh, graph and stopping edges through shared handles so a
/// type can own its walk as the C# does (`BevelFaceFit`, `BevelFaceRunOut`); the walk only reads
/// them.
#[derive(Debug)]
pub struct BevelSurfaceWalk {
    mesh: Arc<Mesh>,
    graph: Rc<MeshEdgeGraph>,
    // Which mesh edges end a face, by mesh edge index, at the caller's own tolerance. An ordered
    // set: C# hands in a `HashSet<int>` whose iteration (insertion) order decides ties in
    // `try_blocking_edge_at` through `surface_edges_at_vertex`.
    surface_edges: Rc<IndexSet<i32>>,
    // The vertex pair a triangle side is, to the edge index the graph knows it by. The walk
    // crosses sides and has to ask the graph what is on the other one. Only looked up.
    edges_by_vertex_pair: HashMap<(i32, i32), i32>,
    // Which feature edges meet at each vertex, built from `surface_edges` alone because that is
    // the only set a walk can be stopped by. Needed when a walk exits through a VERTEX rather
    // than through the middle of a side - see `try_blocking_edge_at`.
    surface_edges_at_vertex: HashMap<i32, Vec<i32>>,
}

impl BevelSurfaceWalk {
    /// A walker over `mesh` (with its own edge graph) that stops at `surface_edges`.
    pub fn new(
        mesh: Arc<Mesh>,
        graph: Rc<MeshEdgeGraph>,
        surface_edges: Rc<IndexSet<i32>>,
    ) -> Self {
        let mut edges_by_vertex_pair = HashMap::with_capacity(graph.edge_count().max(0) as usize);

        for edge_index in 0..graph.edge_count() {
            edges_by_vertex_pair.insert(
                vertex_pair(graph.get_vertex0(edge_index), graph.get_vertex1(edge_index)),
                edge_index,
            );
        }

        let mut surface_edges_at_vertex: HashMap<i32, Vec<i32>> = HashMap::new();

        for &edge_index in surface_edges.iter() {
            for vertex in [graph.get_vertex0(edge_index), graph.get_vertex1(edge_index)] {
                surface_edges_at_vertex
                    .entry(vertex)
                    .or_default()
                    .push(edge_index);
            }
        }

        BevelSurfaceWalk {
            mesh,
            graph,
            surface_edges,
            edges_by_vertex_pair,
            surface_edges_at_vertex,
        }
    }

    fn position(&self, vertex: i32) -> Vector3 {
        Vector3::from_vector3_float(self.mesh.vertices[vertex as usize])
    }

    /// The direction that leaves an edge square to it and heads INTO one of its faces, oriented by
    /// that face's own centroid rather than by a winding convention - the walk has to start inside
    /// the triangle it starts in, and reading the winding wrong would send it straight back out.
    pub fn try_across_direction(&self, face: i32, edge: &FeatureEdge) -> Option<Vector3> {
        let normal = self.try_get_face_normal(face)?;

        let candidate = normal.cross(edge.direction);

        if candidate.length() < 1e-9 {
            return None;
        }

        let candidate = candidate.get_normal();

        let midpoint = (edge.start + edge.end) * 0.5;

        Some(if candidate.dot(self.centroid(face) - midpoint) < 0.0 {
            -candidate
        } else {
            candidate
        })
    }

    /// Walks the surface from `point` on the side `entry_vertex_a`-`entry_vertex_b` of `face`,
    /// heading `direction` in that triangle's plane, across the faces in front of it, and reports
    /// how far it got before the surface turned away or the mesh ran out. It stops once `limit`
    /// has been travelled; nothing further along matters to the caller. Every triangle the walk
    /// passes through is appended to `crossed`, in order, where the caller wants them.
    #[allow(clippy::too_many_arguments)]
    pub fn walk(
        &self,
        mut face: i32,
        mut point: Vector3,
        mut direction: Vector3,
        mut entry_vertex_a: i32,
        mut entry_vertex_b: i32,
        limit: f64,
        mut crossed: Option<&mut Vec<i32>>,
    ) -> SurfaceWalkStop {
        let mut stop = SurfaceWalkStop {
            travelled: 0.0,
            blocking_edge: -1,
            stopped_in_face: -1,
        };
        if let Some(crossed) = crossed.as_deref_mut() {
            crossed.push(face);
        }

        // A walk that keeps going enters a new triangle every step, so it cannot outlast the mesh;
        // the count is the guard against a degenerate step that makes no progress and would
        // otherwise spin.
        let mut steps_left = self.mesh.faces.len() as i64 + 1;

        while steps_left > 0 {
            steps_left -= 1;

            let Some(normal) = self.try_get_face_normal(face) else {
                return stop;
            };

            // Re-projected into every triangle's own plane, which is what lets the walk follow a
            // wall that curves away from where it started rather than leaving the surface.
            let along = direction - (normal * direction.dot(normal));

            if along.length() < 1e-9 {
                return stop;
            }

            let along = along.get_normal();

            let sideways = normal.cross(along);

            let mesh_face = self.mesh.faces[face as usize];

            let mut best_distance = f64::MAX;
            let mut best_exit = Vector3::ZERO;
            let mut best_vertex_a = -1;
            let mut best_vertex_b = -1;

            for (vertex_a, vertex_b) in [
                (mesh_face.v0, mesh_face.v1),
                (mesh_face.v1, mesh_face.v2),
                (mesh_face.v2, mesh_face.v0),
            ] {
                if (vertex_a == entry_vertex_a && vertex_b == entry_vertex_b)
                    || (vertex_a == entry_vertex_b && vertex_b == entry_vertex_a)
                {
                    continue;
                }

                let position_a = self.position(vertex_a);
                let position_b = self.position(vertex_b);

                // Where this side crosses the line of the walk, in the frame where the walk runs
                // along +x from the current point
                let offset_a = position_a - point;
                let offset_b = position_b - point;
                let across_a = offset_a.dot(sideways);
                let across_b = offset_b.dot(sideways);

                if (across_a > 0.0 && across_b > 0.0)
                    || (across_a < 0.0 && across_b < 0.0)
                    || across_a == across_b
                {
                    continue;
                }

                let fraction = across_a / (across_a - across_b);
                let exit = position_a + ((position_b - position_a) * fraction);
                let forward = (exit - point).dot(along);

                if forward <= 1e-9 || forward >= best_distance {
                    continue;
                }

                best_distance = forward;
                best_exit = exit;
                best_vertex_a = vertex_a;
                best_vertex_b = vertex_b;
            }

            if best_vertex_a < 0 {
                // Nothing in front of the walk in this triangle - a degenerate face, or a start
                // point rounding onto a corner of it. Whatever was measured up to here is what
                // there is.
                return stop;
            }

            stop.travelled += (best_exit - point).length();

            let Some(&exit_edge) = self
                .edges_by_vertex_pair
                .get(&vertex_pair(best_vertex_a, best_vertex_b))
            else {
                return stop;
            };

            if self.surface_edges.contains(&exit_edge) {
                stop.blocking_edge = exit_edge;
                stop.stopped_in_face = face;

                return stop;
            }

            // The walk can leave a triangle exactly through a CORNER of it rather than through the
            // middle of a side, and then the side it nominally crossed says nothing about what is
            // in its way. Left alone it walks on into the fan around that vertex, finds every
            // further exit is zero distance ahead, and gives up reporting no blocking edge at all -
            // so an obstruction whose crease happens to END on the sample line is invisible while
            // the same obstruction sampled a hair to either side stops the walk properly. That
            // made admission depend on how the mesh was triangulated, which is what this asks
            // instead.
            if let Some(at_corner) = self.try_blocking_edge_at(
                best_exit,
                best_vertex_a,
                best_vertex_b,
                along,
                normal,
                exit_edge,
            ) {
                stop.blocking_edge = at_corner;
                stop.stopped_in_face = face;

                return stop;
            }

            if self.graph.get_face_count(exit_edge) != 2 {
                // An open rim or a non-manifold junction: the face really does end here, and there
                // is no second face to carry the walk on into.
                return stop;
            }

            if stop.travelled >= limit {
                return stop;
            }

            let first_face = self.graph.get_face(exit_edge, 0);

            face = if first_face == face {
                self.graph.get_face(exit_edge, 1)
            } else {
                first_face
            };
            if let Some(crossed) = crossed.as_deref_mut() {
                crossed.push(face);
            }
            point = best_exit;
            direction = along;
            entry_vertex_a = best_vertex_a;
            entry_vertex_b = best_vertex_b;
        }

        stop
    }

    /// The face's normal from its vertex positions, for the reason [`FeatureEdge`] takes its own
    /// the same way: the cached one is float and can be stale. `None` for a degenerate face.
    pub fn try_get_face_normal(&self, face: i32) -> Option<Vector3> {
        let mesh_face = self.mesh.faces[face as usize];
        let position0 = self.position(mesh_face.v0);
        let position1 = self.position(mesh_face.v1);
        let position2 = self.position(mesh_face.v2);

        let cross = (position1 - position0).cross(position2 - position0);

        // C# `!(LengthSquared > 0)`: zero, negative or NaN.
        let length_squared = cross.length_squared();
        if length_squared <= 0.0 || length_squared.is_nan() || length_squared.is_infinite() {
            return None;
        }

        Some(cross.get_normal())
    }

    fn centroid(&self, face: i32) -> Vector3 {
        let mesh_face = self.mesh.faces[face as usize];

        (self.position(mesh_face.v0) + self.position(mesh_face.v1) + self.position(mesh_face.v2))
            / 3.0
    }

    /// The feature edge that stops a walk which has arrived exactly on a VERTEX: the one meeting
    /// there that lies most squarely across the way the walk is heading.
    ///
    /// Only edges that BOUND the face being walked are candidates, and that filter is what makes
    /// the answer right: a vertex where a crease dies is usually shared with edges of unrelated
    /// surfaces ("Encroaching Side.mcx": the nose's ridge meets the same corner as the crease,
    /// runs more squarely across the walk, and is not part of the slope at all). So a candidate
    /// has to carry a face lying in the plane of the one being walked. Among those, "most squarely
    /// across": an edge running ALONG the walk does not block it. The side the walk nominally
    /// exited by (`exit_edge`) is excluded; the caller has already established it is not a
    /// feature edge. Where nothing qualifies this answers `None` and the walk behaves as before,
    /// so it can only ever ADD an obstruction that genuinely bounds the face.
    fn try_blocking_edge_at(
        &self,
        exit: Vector3,
        vertex_a: i32,
        vertex_b: i32,
        along: Vector3,
        surface_normal: Vector3,
        exit_edge: i32,
    ) -> Option<i32> {
        // Only when the exit really is a corner. Anywhere along the side, the side itself is the
        // honest answer and the caller has already given it.
        let corner = [vertex_a, vertex_b].into_iter().find(|&candidate| {
            (self.position(candidate) - exit).length_squared()
                < VERTEX_COINCIDENCE * VERTEX_COINCIDENCE
        })?;

        let here = self.surface_edges_at_vertex.get(&corner)?;

        let mut most_across = 0.0;
        let mut blocking_edge = -1;

        for &candidate in here {
            if candidate == exit_edge {
                continue;
            }

            let from = self.position(self.graph.get_vertex0(candidate));
            let to = self.position(self.graph.get_vertex1(candidate));
            let direction = to - from;

            if direction.length() < 1e-9 {
                continue;
            }

            if !self.bounds_the_face(candidate, surface_normal) {
                continue;
            }

            let across = 1.0 - direction.get_normal().dot(along).abs();

            if across > most_across {
                most_across = across;
                blocking_edge = candidate;
            }
        }

        (blocking_edge >= 0).then_some(blocking_edge)
    }

    /// Whether an edge bounds the surface being walked - whether either of its faces lies in the
    /// same plane as the triangle the walk is in.
    fn bounds_the_face(&self, edge_index: i32, surface_normal: Vector3) -> bool {
        (0..self.graph.get_face_count(edge_index)).any(|i| {
            self.try_get_face_normal(self.graph.get_face(edge_index, i))
                .is_some_and(|normal| normal.dot(surface_normal) > SAME_SURFACE)
        })
    }
}

fn vertex_pair(vertex_a: i32, vertex_b: i32) -> (i32, i32) {
    if vertex_a < vertex_b {
        (vertex_a, vertex_b)
    } else {
        (vertex_b, vertex_a)
    }
}
