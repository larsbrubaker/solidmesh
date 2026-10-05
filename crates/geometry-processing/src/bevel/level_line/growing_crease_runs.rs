//! The room past a GROWING neighbour, in closed form ([`GrowingCreaseRuns`]). Ports the type of
//! `Bevel/LevelLine/GrowingCreaseRuns.cs`; the run walks, the bounds and the fillet pierce room
//! land with the room-reading method package.

use std::cell::RefCell;
use std::collections::{HashMap, HashSet};
use std::rc::Rc;

use vector_math::Vector3;

use super::triangle_edges;
use super::{LevelLineMesh, LevelLineTriangle};

/// The distance from a point to the chain (C# `Func<Vector3, double>`).
pub type DistanceToChain = Rc<dyn Fn(Vector3) -> f64>;

/// Whether a triangle tag is one of a chain's own sides (C# `Func<int, bool>`).
pub type IsOwnSide = Rc<dyn Fn(i32) -> bool>;

/// Per walked crease piece, the edges that bound the room past it, as (low, high) edges.
pub type CreaseBounds = HashMap<(i32, i32), Vec<(i32, i32)>>;

/// C# `GrowingCreaseRuns`: the room past a GROWING neighbour C (flow rung 2), in closed form. An
/// opposite-sense crease does not bound the room where the band first reaches it: the rows run on
/// virtually past it and C's plane clips them (`VirtualRail`, `StripClip`). What bounds it is C's
/// cap reaching a SECOND crease of C.
///
/// The refinement splits creases, so the run is walked over collinear opposite-sense pieces, once
/// per crease, on a vertex-to-edge adjacency built once per room read and only when a growing
/// crease is met; every piece on a walked run is cached with its answer. Each step of a walk is
/// charged to the work budget. The C# fills those caches from its queries through whatever
/// reference the caller holds (`ForeignEdgeRegions.GrowingFor` hands the same runs to every
/// reader), so the caches are interior-mutable here and the runs are shared through `Rc`.
pub struct GrowingCreaseRuns {
    /// The working mesh the runs are walked on.
    pub mesh: Rc<RefCell<LevelLineMesh>>,
    /// The chain edge the one-edge readings stand on, as (from, to).
    pub chain_edge: (i32, i32),
    /// Every edge of the chain, in order. A ridge's round is read against the edge its start stands
    /// nearest: the round is a cylinder only along ONE straight edge, and a chain that TURNS has one
    /// per leg.
    pub chain_edges: Vec<(i32, i32)>,
    /// The distance from a point to the chain.
    pub distance_to_chain: DistanceToChain,
    /// Whether a triangle tag is one of THIS chain's own sides. The one-chain tagging names them 1
    /// and 2, but a pick of several open chains tags every chain's sides apart, and read as 1 and 2
    /// there the runs judged the wrong chain's sides: on "Bevel to Center Feature.mcx" the right
    /// tip's side is tag 3, so the nose creases on its own slope never read as growing and bounded
    /// it at 2.93 where the left tip, the mirror image, grew over its own and kept 3.49. Only which
    /// creases GROW is read this way; the pierce room's two sides still read tags 1 and 2.
    pub is_own_side: IsOwnSide,
    /// Per walked crease piece, the far end of its run (the walk's cache).
    pub far_end: RefCell<HashMap<(i32, i32), i32>>,
    /// Per walked crease piece, the edges that bound the room past it (the bounds' cache).
    pub bounds: RefCell<CreaseBounds>,
    /// The vertex-to-vertex adjacency the walks run on, built on first need; `None` before.
    pub neighbours: RefCell<Option<HashMap<i32, Vec<i32>>>>,
    /// The growing neighbours' ridges met so far, each as (its start on the crease, its far end).
    pub ridges: RefCell<Vec<(i32, i32)>>,
    /// The edges the bounds have handed out - creases AWAY from the picked face that bound the
    /// room in place of a growing crease - so the room they set is told apart from one set by an
    /// edge on the face itself.
    pub bound_edges: RefCell<HashSet<(i32, i32)>>,
}

impl std::fmt::Debug for GrowingCreaseRuns {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        // The two callbacks have no Debug; give the chain and what has been met.
        f.debug_struct("GrowingCreaseRuns")
            .field("chain_edge", &self.chain_edge)
            .field("chain_edges", &self.chain_edges)
            .field("ridges", &self.ridges)
            .finish_non_exhaustive()
    }
}

impl GrowingCreaseRuns {
    /// The unit normal of a working-mesh triangle.
    pub fn normal(&self, t: LevelLineTriangle) -> Vector3 {
        let mesh = self.mesh.borrow();
        (mesh.position(t.b) - mesh.position(t.a))
            .cross(mesh.position(t.c) - mesh.position(t.a))
            .get_normal()
    }

    /// The chain edge nearest `point` - the first of a tie.
    pub fn nearest_chain_edge(&self, point: Vector3) -> (i32, i32) {
        let mesh = self.mesh.borrow();
        let mut best = self.chain_edges[0];
        let mut best_distance = f64::INFINITY;
        for &edge in &self.chain_edges {
            let a = mesh.position(edge.0);
            let ab = mesh.position(edge.1) - a;
            let t = if ab.length_squared() > 0.0 {
                ((point - a).dot(ab) / ab.length_squared()).clamp(0.0, 1.0)
            } else {
                0.0
            };
            let distance = (point - (a + (ab * t))).length();
            if distance < best_distance {
                best_distance = distance;
                best = edge;
            }
        }

        best
    }

    /// True when `edge` is one of the far bounds the growing creases' room was set by.
    pub fn is_a_far_bound(&self, edge: (i32, i32)) -> bool {
        self.bound_edges
            .borrow()
            .contains(&triangle_edges::key(edge.0, edge.1))
    }
}
