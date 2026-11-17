//! The reducer's distance guard (`reducer.rs`, whose child module this is): a bound on how far
//! the reduced surface may stray from the original, enforced by refusing collapses rather than
//! by pricing them. The quadric error orders the collapses; it is a sum over planes, so a thin
//! tip or a small boss whose few faces add little to it can be cut away cheaply. The guard
//! keeps that from happening.
//!
//! - **When.** While the mesh is dense its collapses move the surface far less than the last
//!   ones will, so the guard starts once the triangle count is down to [`LATE_START`] times the
//!   target; every original point is then placed on its nearest current triangle and the bound
//!   starts at the farthest of them.
//! - **Original points.** Every vertex of the original mesh is held by exactly one current
//!   triangle (intrusive lists: `head` per triangle id, `next` per point, `u32`), with the
//!   centroid and long-edge midpoints of any original triangle much longer than the typical
//!   edge, whose inside its corners do not speak for. After a collapse a point of a touched
//!   triangle stays on it while within half the bound, and otherwise goes to its nearest
//!   triangle around the kept vertex, ties to the lowest id.
//! - **The check.** A collapse to a position passes when every point held by a triangle
//!   touching the edge lies within `tau` of one of the triangles the collapse leaves around the
//!   kept vertex (each point tries the triangle that replaces its own first, and a point whose
//!   recorded distance and barycentric weight show it cannot have moved out of reach is not
//!   measured; the point that failed this edge's last check is tried first of all), and the
//!   new position and the
//!   midpoints of its new edges each lie within `tau` of the
//!   original surface (a search through the original's AABB tree that stops at the first
//!   triangle within `tau`).
//! - **Positions.** On a refusal the reducer tries the other positions in order of their
//!   quadric value (`Reducer::collapse_candidates`): the ends and the midpoint, each projected
//!   onto the original with Maintain Surface; it takes the first that passes. A
//!   collapse that would leave the kept vertex no triangle is refused as invalid.
//! - **Hints.** A refused edge remembers the point that failed; the next check of that edge
//!   tries it first, but only while a triangle the collapse touches still holds it.
//! - **The bound grows.** A refused edge records the bound it would need; when the queue runs
//!   dry before the target, `tau` grows to the larger of 1.25 times itself and the smallest
//!   recorded need, and the edges that now fit go back in the queue in order of need. After
//!   [`MAX_ROUNDS`] rounds the guard turns off, so a reduce always reaches its target.
//!
//! Everything runs in index order with explicit tie-breaks, so the guard keeps Reduce's output
//! the same to the bit on every run.

use super::super::dmesh3::{tri_distance_sqr, DMesh3, INVALID_ID};
use super::super::dmesh_aabb_tree3::DMeshAABBTree3;
use super::super::tri_distance::distance_sqr_closest_params;
use super::super::vector3d::{add, distance, scale};
use super::{ProcessResult, Reducer};

/// No point, no triangle.
const NONE: u32 = u32::MAX;
/// The first bound, as a fraction of the original's largest bounding-box side.
const FIRST_TAU: f64 = 1e-4;
/// How much the bound grows at least, each time the queue runs dry.
const GROWTH: f64 = 1.25;
/// An original edge longer than this many times the median edge has its midpoint held, and a
/// triangle with such an edge its centroid.
const LONG_EDGE: f64 = 2.0;
/// The most original triangles a check tries before searching the tree for a probe.
const LOCAL_ORIGINS: usize = 32;
/// The guard starts once the triangle count is down to this many times the target.
pub(crate) const LATE_START: f64 = 4.0;
/// Rounds of growth after which the guard turns off.
pub(crate) const MAX_ROUNDS: u32 = 64;

/// How Reduce runs: which checks are on and which positions a collapse may take. The default
/// is the guard on and positions off the surface allowed; Reduce's "Maintain Surface" keeps
/// them on it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct ReduceOptions {
    /// Refuse collapses that would move the surface farther than the guard's bound.
    pub(crate) guard: bool,
    /// Keep every collapse on the original surface (Reduce's "Maintain Surface").
    pub(crate) on_surface: bool,
}

impl Default for ReduceOptions {
    fn default() -> Self {
        ReduceOptions {
            guard: true,
            on_surface: false,
        }
    }
}

/// The guard's state over one reduce.
pub(crate) struct Guard<'t> {
    original: &'t DMeshAABBTree3<'t>,
    points: Vec<[f64; 3]>,
    /// Per point, an original triangle it lies on (or at a corner of): where a search for the
    /// original surface near it starts.
    origin: Vec<i32>,
    /// Per point, its distance to the triangle holding it when it was last placed there; the
    /// holder changes only when a collapse touches it, and then the point is placed again.
    last: Vec<f64>,
    /// Per point, the barycentric weights (in its holder's corner order) of its nearest point
    /// on the holder when it was placed: a move of one corner by `d` moves that nearest point
    /// by its weight times `d`, which bounds how much farther the point can be.
    weight: Vec<[f64; 3]>,
    /// Per point, the triangle holding it (`INVALID_ID` until the guard starts).
    holder: Vec<i32>,
    head: Vec<u32>,
    next: Vec<u32>,
    pub(crate) tau: f64,
    pub(crate) enabled: bool,
    pub(crate) rounds: u32,
    pub(crate) refusals: u64,
    /// Whether the guard has placed its points (it starts once and never again).
    started: bool,
    /// Per edge, the point that failed its last check.
    hint: Vec<u32>,
    /// Per edge, the bound it needs while it waits (NaN when it is not waiting).
    need: Vec<f64>,
    waiting: Vec<(f64, i32)>,
    touch_buf: Vec<i32>,
    ring_buf: Vec<(i32, [[f64; 3]; 3])>,
    local_buf: Vec<i32>,
}

impl<'t> Guard<'t> {
    /// A guard over `mesh` (the mesh about to be reduced, identical to the tree's mesh), its
    /// first bound a fraction of the part's largest side `size`. It is off until
    /// [`Guard::rehome`] places its points.
    pub(crate) fn new(mesh: &DMesh3, original: &'t DMeshAABBTree3<'t>, size: f64) -> Self {
        let mut points: Vec<[f64; 3]> = Vec::new();
        let mut holder: Vec<i32> = Vec::new();
        let mut lengths: Vec<f64> = mesh
            .edge_indices()
            .map(|e| {
                let [a, b] = mesh.get_edge_v(e);
                distance(mesh.vertex(a), mesh.vertex(b))
            })
            .collect();
        lengths.sort_by(f64::total_cmp);
        let median = lengths.get(lengths.len() / 2).copied().unwrap_or(0.0);
        let long = LONG_EDGE * median;
        for vid in mesh.vertex_indices() {
            if let Some(t) = mesh.vtx_triangles_iter(vid).min() {
                points.push(mesh.vertex(vid));
                holder.push(t);
            }
        }
        for e in mesh.edge_indices() {
            let [a, b, t0, t1] = mesh.get_edge(e);
            if distance(mesh.vertex(a), mesh.vertex(b)) > long {
                points.push(scale(add(mesh.vertex(a), mesh.vertex(b)), 0.5));
                holder.push(if t1 == INVALID_ID { t0 } else { t0.min(t1) });
            }
        }
        for t in mesh.triangle_indices() {
            let c = mesh.get_tri_vertices(t as usize);
            let longest = distance(c[0], c[1])
                .max(distance(c[1], c[2]))
                .max(distance(c[2], c[0]));
            if longest > long {
                points.push(scale(add(add(c[0], c[1]), c[2]), 1.0 / 3.0));
                holder.push(t);
            }
        }
        // The points are placed on the mesh when the guard starts (`Guard::rehome`).
        let count = points.len();
        let origin = holder;
        let n_edges = mesh.max_edge_id() as usize;
        Guard {
            original,
            points,
            origin,
            last: vec![0.0; count],
            weight: vec![[0.0; 3]; count],
            holder: vec![INVALID_ID; count],
            head: Vec::new(),
            next: vec![NONE; count],
            tau: FIRST_TAU * size,
            enabled: false,
            rounds: 0,
            refusals: 0,
            started: false,
            hint: vec![NONE; n_edges],
            need: vec![f64::NAN; n_edges],
            waiting: Vec::new(),
            touch_buf: Vec::new(),
            ring_buf: Vec::new(),
            local_buf: Vec::new(),
        }
    }

    /// Places every point on its nearest triangle of `current` (the mesh as the unguarded first
    /// stretch of the reduce left it), raises the bound to the farthest of them and turns the
    /// guard on.
    pub(crate) fn rehome(&mut self, current: &DMesh3) {
        let tree = DMeshAABBTree3::new(current);
        self.head = vec![NONE; current.max_triangle_id() as usize];
        // Nearly every point lies within an edge or two of the current surface: a search bounded
        // there prunes most of the tree, and only a point beyond it needs the whole search.
        let mut lengths: Vec<f64> = current
            .edge_indices()
            .map(|e| {
                let [a, b] = current.get_edge_v(e);
                distance(current.vertex(a), current.vertex(b))
            })
            .collect();
        lengths.sort_by(f64::total_cmp);
        let near = 2.0 * lengths.get(lengths.len() / 2).copied().unwrap_or(0.0);
        let mut farthest = 0.0f64;
        for o in 0..self.points.len() {
            let q = self.points[o];
            let mut t = tree.find_nearest_triangle(q, near);
            if t < 0 {
                t = tree.find_nearest_triangle(q, f64::MAX);
            }
            if t < 0 {
                continue;
            }
            let c = current.get_tri_vertices(t as usize);
            let d = tri_distance_sqr(&c, q).sqrt();
            farthest = farthest.max(d);
            self.last[o] = d;
            self.weight[o] = barycentric(&c, q);
            self.holder[o] = t;
            self.next[o] = self.head[t as usize];
            self.head[t as usize] = o as u32;
        }
        self.start_at(farthest);
        self.hint.iter_mut().for_each(|h| *h = NONE);
        self.enabled = true;
    }

    /// Starts the bound at `tau` when that is larger than the floor it started at.
    fn start_at(&mut self, tau: f64) {
        if tau.is_finite() && tau > self.tau {
            self.tau = tau;
        }
    }

    /// The triangles around `a` and `b` other than the edge's own, as they would be with both
    /// ends at `p`, each with its id.
    fn new_ring(mesh: &DMesh3, a: i32, b: i32, p: [f64; 3]) -> Vec<(i32, [[f64; 3]; 3])> {
        let mut ring = Vec::new();
        for t in touching(mesh, a, b) {
            let tri = mesh.get_triangle(t);
            let has_a = tri.contains(&a);
            let has_b = tri.contains(&b);
            if has_a && has_b {
                continue;
            }
            let mut corners = mesh.get_tri_vertices(t as usize);
            for k in 0..3 {
                if tri[k] == a || tri[k] == b {
                    corners[k] = p;
                }
            }
            ring.push((t, corners));
        }
        ring
    }

    /// The points around the new position whose distance to the original surface is checked
    /// once a position has passed everything else: the midpoint of each new edge from it (each
    /// once: the corner after `p` in its triangle's winding).
    fn probes(ring: &[(i32, [[f64; 3]; 3])], p: [f64; 3]) -> Vec<[f64; 3]> {
        let mut probes = Vec::new();
        for (_, c) in ring {
            if let Some(k) = (0..3).find(|&k| c[k] == p) {
                probes.push(scale(add(c[(k + 1) % 3], p), 0.5));
            }
        }
        probes
    }

    /// Whether collapsing edge `eid` (`a`, `b`) to `p` keeps the surface within the bound.
    pub(crate) fn passes(&mut self, mesh: &DMesh3, eid: i32, a: i32, b: i32, p: [f64; 3]) -> bool {
        let mut touch = std::mem::take(&mut self.touch_buf);
        let mut ring = std::mem::take(&mut self.ring_buf);
        let mut local = std::mem::take(&mut self.local_buf);
        touching_into(mesh, a, b, &mut touch);
        new_ring_into(mesh, a, b, p, &touch, &mut ring);
        let result = !ring.is_empty() && self.check(mesh, eid, a, b, p, &touch, &ring, &mut local);
        self.touch_buf = touch;
        self.ring_buf = ring;
        self.local_buf = local;
        result
    }

    /// [`Guard::passes`] over the gathered triangles: the failing point of the edge's last check
    /// first, then every held point (its own replaced triangle first, the rest of the ring only
    /// when that is too far), then the new position and its new edges' midpoints against the
    /// original.
    #[allow(clippy::too_many_arguments)]
    fn check(
        &mut self,
        mesh: &DMesh3,
        eid: i32,
        a: i32,
        b: i32,
        p: [f64; 3],
        touch: &[i32],
        ring: &[(i32, [[f64; 3]; 3])],
        local: &mut Vec<i32>,
    ) -> bool {
        let tau_sqr = self.tau * self.tau;
        let near = |q: [f64; 3], own: i32| -> bool {
            let own_corners = ring.iter().find(|(t, _)| *t == own);
            if let Some((_, c)) = own_corners {
                if tri_distance_sqr(c, q) <= tau_sqr {
                    return true;
                }
            }
            ring.iter()
                .any(|(t, c)| *t != own && tri_distance_sqr(c, q) <= tau_sqr)
        };
        // The point that failed this edge's last check, if a triangle the collapse touches
        // still holds it: one that has moved on to another triangle says nothing about this
        // collapse, and testing it would refuse a collapse that leaves it alone.
        let hint = self.hint[eid as usize];
        if hint != NONE
            && touch.binary_search(&self.holder[hint as usize]).is_ok()
            && !near(self.points[hint as usize], INVALID_ID)
        {
            return false;
        }
        // The original triangles under the held points: a probe near the original surface is
        // nearly always within the bound of one of them, which spares the tree search.
        local.clear();
        for &t in touch {
            let mut o = self.head[t as usize];
            let moved_corner = moved_corner(mesh, ring, t, a, b, p);
            while o != NONE {
                let settled = moved_corner.is_some_and(|(k, d)| {
                    self.last[o as usize] + self.weight[o as usize][k] * d <= self.tau
                });
                if !settled && !near(self.points[o as usize], t) {
                    self.hint[eid as usize] = o;
                    return false;
                }
                if local.len() < LOCAL_ORIGINS {
                    local.push(self.origin[o as usize]);
                }
                o = self.next[o as usize];
            }
        }
        let original = self.original;
        let on_original = |q: [f64; 3]| {
            local.iter().any(|&t| {
                tri_distance_sqr(&original.mesh.get_tri_vertices(t as usize), q) <= tau_sqr
            }) || original.any_triangle_within(q, self.tau)
        };
        // The position first: most refusals of the reverse kind end there.
        if !on_original(p) {
            return false;
        }
        ring.iter().all(|(_, c)| match (0..3).find(|&k| c[k] == p) {
            Some(k) => on_original(scale(add(c[(k + 1) % 3], p), 0.5)),
            None => true,
        })
    }

    /// The bound collapsing edge `a`-`b` to `p` would need: the farthest held point from the
    /// new triangles, and the farthest probe from the original surface.
    pub(crate) fn needed(&self, mesh: &DMesh3, a: i32, b: i32, p: [f64; 3]) -> f64 {
        let ring = Self::new_ring(mesh, a, b, p);
        if ring.is_empty() {
            return f64::INFINITY;
        }
        // A point can raise the farthest distance only if it is farther than it from every new
        // triangle, its own first: most points are settled by their own triangle alone.
        let mut worst = 0.0f64;
        for t in touching(mesh, a, b) {
            let own = ring.iter().find(|(r, _)| *r == t).map(|(_, c)| c);
            let moved = moved_corner(mesh, &ring, t, a, b, p);
            let mut o = self.head[t as usize];
            while o != NONE {
                let q = self.points[o as usize];
                let skip = match own {
                    Some(c) => {
                        let shift =
                            moved.map_or(f64::INFINITY, |(k, d)| self.weight[o as usize][k] * d);
                        let bound = self.last[o as usize] + shift;
                        bound * bound <= worst || tri_distance_sqr(c, q) <= worst
                    }
                    None => false,
                };
                if !skip {
                    let d = ring
                        .iter()
                        .map(|(_, c)| tri_distance_sqr(c, q))
                        .fold(f64::INFINITY, f64::min);
                    worst = worst.max(d);
                }
                o = self.next[o as usize];
            }
        }
        // A probe within the bound found so far cannot raise it, and the search bounded by it
        // is cheap; only a probe beyond it needs the unbounded search.
        let original_mesh = self.original.mesh;
        for q in std::iter::once(p).chain(Self::probes(&ring, p)) {
            let bound = worst.sqrt().max(self.tau);
            if self.original.any_triangle_within(q, bound) {
                continue;
            }
            let t = self.original.find_nearest_triangle(q, f64::MAX);
            if t >= 0 {
                worst = worst.max(tri_distance_sqr(
                    &original_mesh.get_tri_vertices(t as usize),
                    q,
                ));
            }
        }
        worst.sqrt()
    }

    /// Whether collapsing edge `a`-`b` leaves the kept vertex any triangle (one touching `a` or
    /// `b` that is not on the edge).
    pub(crate) fn leaves_a_ring(mesh: &DMesh3, a: i32, b: i32) -> bool {
        mesh.vtx_triangles_iter(a)
            .chain(mesh.vtx_triangles_iter(b))
            .any(|t| {
                let tri = mesh.get_triangle(t);
                !(tri.contains(&a) && tri.contains(&b))
            })
    }

    /// The triangle ids touching `a` or `b`, ascending, before a collapse.
    pub(crate) fn touching_before(mesh: &DMesh3, a: i32, b: i32) -> Vec<i32> {
        touching(mesh, a, b)
    }

    /// After a collapse onto `kept`: the points of `before` (the triangles that touched the
    /// edge) are placed again. A point whose triangle survives and lies within half the bound
    /// of it stays there; the rest go to their nearest triangle around `kept`, ties to the
    /// lowest id. Each point's distance to its holder is recorded.
    pub(crate) fn after_collapse(&mut self, mesh: &DMesh3, kept: i32, before: &[i32]) {
        let half_sqr = 0.25 * self.tau * self.tau;
        let mut moved = Vec::new();
        for &t in before {
            let mut o = self.head[t as usize];
            self.head[t as usize] = NONE;
            let own = mesh
                .is_triangle(t)
                .then(|| mesh.get_tri_vertices(t as usize));
            while o != NONE {
                let following = self.next[o as usize];
                let stays = own.map(|c| (tri_distance_sqr(&c, self.points[o as usize]), c));
                match stays {
                    Some((d, c)) if d <= half_sqr => {
                        self.last[o as usize] = d.sqrt();
                        self.weight[o as usize] = barycentric(&c, self.points[o as usize]);
                        self.holder[o as usize] = t;
                        self.next[o as usize] = self.head[t as usize];
                        self.head[t as usize] = o;
                    }
                    _ => moved.push(o),
                }
                o = following;
            }
        }
        let mut ring = mesh.vtx_triangles(kept);
        if ring.is_empty() {
            // A collapse that leaves the kept vertex no triangle (a closed mesh never does)
            // must not lose its points: they go to the surviving triangles it touched.
            ring = before
                .iter()
                .copied()
                .filter(|&t| mesh.is_triangle(t))
                .collect();
        }
        if ring.is_empty() {
            ring = mesh.triangle_indices().take(1).collect();
        }
        ring.sort_unstable();
        let corners: Vec<[[f64; 3]; 3]> = ring
            .iter()
            .map(|&t| mesh.get_tri_vertices(t as usize))
            .collect();
        for o in moved {
            let q = self.points[o as usize];
            // Ties go to the lowest id: `ring` is ascending and only a strictly nearer one wins.
            let mut best = (f64::INFINITY, 0usize);
            for (k, c) in corners.iter().enumerate() {
                let d = tri_distance_sqr(c, q);
                if d < best.0 {
                    best = (d, k);
                }
            }
            if let Some(&t) = ring.get(best.1) {
                let d = tri_distance_sqr(&corners[best.1], q);
                self.last[o as usize] = d.sqrt();
                self.weight[o as usize] = barycentric(&corners[best.1], q);
                self.holder[o as usize] = t;
                self.next[o as usize] = self.head[t as usize];
                self.head[t as usize] = o;
            }
        }
    }

    /// Edge `eid` was refused and needs bound `need` to collapse.
    pub(crate) fn wait(&mut self, eid: i32, need: f64) {
        self.refusals += 1;
        if self.need[eid as usize].is_nan() {
            self.need[eid as usize] = need;
            self.waiting.push((need, eid));
        }
    }

    /// Edge `eid` was re-scored: it no longer waits, and its old failing point is stale.
    pub(crate) fn rescored(&mut self, eid: i32) {
        if let Some(n) = self.need.get_mut(eid as usize) {
            *n = f64::NAN;
        }
        if let Some(h) = self.hint.get_mut(eid as usize) {
            *h = NONE;
        }
    }

    /// The queue ran dry before the target: grows the bound and returns the waiting edges that
    /// now fit, in order of need (all of them once the guard turns off).
    pub(crate) fn grow(&mut self) -> Vec<i32> {
        self.waiting
            .retain(|&(n, e)| self.need[e as usize].to_bits() == n.to_bits());
        self.waiting
            .sort_by(|x, y| x.0.total_cmp(&y.0).then(x.1.cmp(&y.1)));
        self.rounds += 1;
        if self.rounds > MAX_ROUNDS {
            self.enabled = false;
        } else if let Some(&(smallest, _)) = self.waiting.first() {
            self.tau = (GROWTH * self.tau).max(smallest);
        }
        let mut ready = Vec::new();
        let mut keep = Vec::new();
        for &(n, e) in &self.waiting {
            if !self.enabled || n <= self.tau {
                ready.push(e);
                self.need[e as usize] = f64::NAN;
            } else {
                keep.push((n, e));
            }
        }
        self.waiting = keep;
        ready
    }

    /// Whether any edge still waits.
    pub(crate) fn has_waiting(&self) -> bool {
        !self.waiting.is_empty()
    }
}

/// The triangle ids touching `a` or `b`, ascending and without repeats.
fn touching_into(mesh: &DMesh3, a: i32, b: i32, out: &mut Vec<i32>) {
    out.clear();
    out.extend(mesh.vtx_triangles_iter(a));
    out.extend(mesh.vtx_triangles_iter(b));
    out.sort_unstable();
    out.dedup();
}

/// [`Guard::new_ring`] into a reused buffer, over the already gathered `touch`.
fn new_ring_into(
    mesh: &DMesh3,
    a: i32,
    b: i32,
    p: [f64; 3],
    touch: &[i32],
    out: &mut Vec<(i32, [[f64; 3]; 3])>,
) {
    out.clear();
    for &t in touch {
        let tri = mesh.get_triangle(t);
        if tri.contains(&a) && tri.contains(&b) {
            continue;
        }
        let mut corners = mesh.get_tri_vertices(t as usize);
        for k in 0..3 {
            if tri[k] == a || tri[k] == b {
                corners[k] = p;
            }
        }
        out.push((t, corners));
    }
}

fn touching(mesh: &DMesh3, a: i32, b: i32) -> Vec<i32> {
    let mut tris = mesh.vtx_triangles(a);
    tris.extend(mesh.vtx_triangles(b));
    tris.sort_unstable();
    tris.dedup();
    tris
}

impl<'t> Reducer<'_, 't> {
    /// Runs the reduce with the guard over `original` (the tree of a copy of the mesh before
    /// any collapse), its first bound set by the part's largest side `size`; the guard starts
    /// at [`LATE_START`] times the target (`Reducer::maybe_start_guard`).
    pub(crate) fn attach_guard(&mut self, original: &'t DMeshAABBTree3<'t>, size: f64) {
        self.guard = Some(Guard::new(&*self.base.mesh, original, size));
    }

    /// Turns the guard on once the triangle count is down to [`LATE_START`] times the target.
    pub(super) fn maybe_start_guard(&mut self) {
        let Some(guard) = &mut self.guard else {
            return;
        };
        if guard.enabled || guard.rounds > MAX_ROUNDS || guard.started {
            return;
        }
        if (self.base.mesh.triangle_count() as f64) <= LATE_START * self.target_count as f64 {
            guard.started = true;
            guard.rehome(&*self.base.mesh);
        }
    }

    /// The positions edge `eid` may collapse to, cheapest by its quadric first (ties in this
    /// order): the scored point `p`, the two ends and the midpoint, the last three projected
    /// onto the target with Maintain Surface. A pinned end or a textured mesh keeps the one
    /// position its rules chose.
    fn collapse_candidates(
        &self,
        eid: i32,
        [a, b]: [i32; 2],
        p: [f64; 3],
        fixed: bool,
    ) -> Vec<[f64; 3]> {
        if fixed || self.face_data.is_some() || self.guard.as_ref().is_none_or(|g| !g.enabled) {
            return vec![p];
        }
        let va = self.base.mesh.vertex(a);
        let vb = self.base.mesh.vertex(b);
        let mid = scale(add(va, vb), 0.5);
        let all = match (self.on_surface, self.target) {
            // Maintain Surface: `p` was projected as it was scored; the fallbacks are too.
            (true, Some(target)) => vec![
                p,
                target.project(va, -1),
                target.project(vb, -1),
                target.project(mid, -1),
            ],
            _ => vec![p, va, vb, mid],
        };
        let q = &self.edge_quadrics[eid as usize].q;
        let mut scored: Vec<(f64, usize, [f64; 3])> = all
            .into_iter()
            .enumerate()
            .map(|(i, x)| (q.evaluate(x), i, x))
            .collect();
        scored.sort_by(|x, y| x.0.total_cmp(&y.0).then(x.1.cmp(&y.1)));
        scored.into_iter().map(|(_, _, x)| x).collect()
    }

    /// The first candidate position that turns no triangle over and passes the guard, or why
    /// none does. A refusal by the guard records the bound the cheapest flip-free position
    /// would need.
    pub(super) fn choose_position(
        &mut self,
        eid: i32,
        [a, b]: [i32; 2],
        [t0, t1]: [i32; 2],
        p: [f64; 3],
        fixed: bool,
    ) -> Result<[f64; 3], ProcessResult> {
        let candidates = self.collapse_candidates(eid, [a, b], p, fixed);
        let mut first_unflipped = None;
        for x in candidates {
            if self.base.collapse_creates_flip_or_invalid(a, b, x, t0, t1)
                || self.base.collapse_creates_flip_or_invalid(b, a, x, t0, t1)
            {
                continue;
            }
            match &mut self.guard {
                Some(guard) if guard.enabled => {
                    // A collapse that leaves no triangle around the kept vertex is not a
                    // shape the guard can measure: it is refused as invalid, never priced.
                    if !Guard::leaves_a_ring(&*self.base.mesh, a, b) {
                        continue;
                    }
                    first_unflipped.get_or_insert(x);
                    if guard.passes(&*self.base.mesh, eid, a, b, x) {
                        return Ok(x);
                    }
                }
                _ => return Ok(x),
            }
        }
        let Some(x) = first_unflipped else {
            return Err(ProcessResult::IgnoredCreatesFlip);
        };
        if let Some(guard) = &mut self.guard {
            let need = guard.needed(&*self.base.mesh, a, b, x);
            if !need.is_finite() {
                return Err(ProcessResult::IgnoredCannotCollapse);
            }
            guard.wait(eid, need);
        }
        Err(ProcessResult::IgnoredConstrained)
    }

    /// The queue ran dry before the target: grows the guard's bound and puts the edges that
    /// now fit back in the queue. False when there is nothing to put back.
    pub(super) fn requeue_waiting(&mut self) -> bool {
        let Some(guard) = &mut self.guard else {
            return false;
        };
        if !guard.has_waiting() {
            return false;
        }
        let ready = guard.grow();
        for eid in ready {
            if self.base.mesh.is_edge(eid) && !self.edge_queue.contains(eid) {
                let err = self.edge_quadrics[eid as usize].err;
                self.edge_queue.insert(eid, err);
            }
        }
        true
    }
}

/// The barycentric weights of the point of triangle `c` nearest `q`.
fn barycentric(c: &[[f64; 3]; 3], q: [f64; 3]) -> [f64; 3] {
    let (_, _, [s, t]) = distance_sqr_closest_params(c, q);
    [1.0 - s - t, s, t]
}

/// For triangle `t` of the new ring, the corner that moves to `p` (the one that is `a` or
/// `b`) and how far it moves; `None` when `t` leaves the mesh.
fn moved_corner(
    mesh: &DMesh3,
    ring: &[(i32, [[f64; 3]; 3])],
    t: i32,
    a: i32,
    b: i32,
    p: [f64; 3],
) -> Option<(usize, f64)> {
    ring.iter().find(|(r, _)| *r == t)?;
    let tri = mesh.get_triangle(t);
    let k = (0..3).find(|&k| tri[k] == a || tri[k] == b)?;
    Some((k, distance(p, mesh.vertex(tri[k]))))
}
