//! Tracing a bundle of rays together: [`RayBundle`] holds the rays, their per-ray results and
//! the deterministic work counters [`RayBundleStats`], and is what
//! [`Traceable::trace_ray_bundle`] traces. Every shape works with no code of its own (the
//! trait's default body runs each active ray alone through `get_closest_intersection`); the
//! packet traversal lives in `bvh_bundle_traversal` beside the node it walks.
//!
//! The contract is that every ray's result equals `get_closest_intersection` on that ray alone,
//! bit for bit, including how far the ray's `max_distance_to_consider` was narrowed. The rules for
//! which hit wins live with each kind of node (`trace_scene` here holds a scene's, matching
//! `RayTracer::trace_primary_ray`); this file otherwise carries results up the tree. Each call pushes at most one `(ray, hit)` per
//! active ray onto a stack shared by the whole trace, in any order; a parent pairs the entries
//! of its two children by ray index through a table reset as it goes, so a merge costs the
//! number of hits, not the number of rays.
//!
//! [`PacketBounds`] is the miss test: interval arithmetic over the active rays' origins and
//! reciprocal directions, for any bundle (shared origin or not, jittered, shadow rays). It only
//! ever proves that every active ray misses a box under `Ray::intersection`, the test a lone ray
//! uses. Those bounds are computed with the same two floating-point operations in the same
//! order as that test, so rounding (monotone in each operand) keeps every real entry and exit
//! distance inside them. An axis with a zero direction component gives no bound at all: a lone
//! ray there sees `0 * inf = NaN` for an origin exactly on a box plane, which its comparisons
//! read as no constraint, or a miss on x.
//!
//! The ray index is a `u32`, and [`RayBundle::trace`] refuses a bundle of more than `u32::MAX`
//! rays rather than truncate. Everything that takes an index from outside this module
//! ([`RayBundle::trace_one`], [`Traceable::trace_ray_bundle`]) returns a
//! [`RayTracerError::InvalidArgument`] for one past the end, so the indices the traversal
//! reads are always in range; a refusal is a `Result` rather than a skipped ray so a caller
//! cannot mistake it for a miss.

use solidmesh_math::{AxisAlignedBoundingBox, IntersectionType, Ray};

use super::{IntersectInfo, RayTracerError, Traceable, TraceableRef};

/// How much work a bundle trace did. Every field is a plain count, so two traces of the same
/// rays and scene report the same numbers on every machine; a bundle of one ray counts what a
/// lone ray does.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Hash)]
pub struct RayBundleStats {
    /// Ray-box tests, including the ones [`RayBundleStats::gate_tests`] counts.
    pub box_tests: u64,
    /// Interval-arithmetic miss tests against a node's box.
    pub interval_tests: u64,
    /// Rays handed to a leaf's own intersection test through [`RayBundle::trace_one`]
    /// (ray-triangle tests, on a mesh). Rays traced whole by the lone path are in
    /// [`RayBundleStats::lone_traces`] and [`RayBundleStats::retraced_rays`], not here.
    pub leaf_tests: u64,
    /// Node steps settled by the first active ray hitting the box.
    pub early_hits: u64,
    /// Node steps settled by the interval test proving every active ray misses.
    pub interval_misses: u64,
    /// Node steps where the first active ray missed and the interval test could not prove the
    /// rest do, so the remaining rays were tested one at a time.
    pub last_resorts: u64,
    /// The rays those steps tested, the first active ray not counted.
    pub last_resort_rays: u64,
    /// Ray-box tests pruning the active rays at a node: at a node with a leaf child, deciding
    /// which rays really reach it, and periodically on the way down.
    pub gate_tests: u64,
    /// Rays a node traced alone from its own top because they, or the node's box, cannot use
    /// the packet traversal (non-finite values, an unsound box). What they test is not counted.
    pub lone_traces: u64,
    /// Rays traced alone again after a shape widened their window (see
    /// [`RayBundle::note_widened`]). What they test is not counted.
    pub retraced_rays: u64,
}

impl RayBundleStats {
    /// Adds `other`'s counts to these.
    pub fn add(&mut self, other: &RayBundleStats) {
        self.box_tests += other.box_tests;
        self.interval_tests += other.interval_tests;
        self.leaf_tests += other.leaf_tests;
        self.early_hits += other.early_hits;
        self.interval_misses += other.interval_misses;
        self.last_resorts += other.last_resorts;
        self.last_resort_rays += other.last_resort_rays;
        self.gate_tests += other.gate_tests;
        self.lone_traces += other.lone_traces;
        self.retraced_rays += other.retraced_rays;
    }
}

/// Which child a node visits first for the rays under test.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum AxisOrder {
    LeftFirst,
    RightFirst,
    /// The rays disagree on the direction's sign along this axis.
    Mixed,
}

/// True when a ray visits a node's right child first along `axis`: the one place the packet
/// traversal reads a direction's sign, so it cannot differ from what the node's lone-ray
/// child order reads (`direction < 0`, so `-0.0` goes left first).
pub(super) fn visits_right_first(ray: &Ray, axis: usize) -> bool {
    ray.direction_normal[axis] < 0.0
}

/// What [`RayBundle::combine_hits`] keeps for a ray.
pub(super) enum Pick {
    First,
    Second,
    Neither,
}

/// A conservative box around a set of rays: the bounds of their origins and reciprocal
/// directions, and what any of them could still accept as a hit distance. Public only so the
/// `ray_bundle_bench` example can time [`PacketBounds::misses`]; nothing else should use it.
#[derive(Debug, Clone, Copy)]
pub struct PacketBounds {
    origin_min: [f64; 3],
    origin_max: [f64; 3],
    inverse_min: [f64; 3],
    inverse_max: [f64; 3],
    /// True on an axis where some ray's reciprocal direction is infinite: no bound there.
    unbounded: [bool; 3],
    window_min: f64,
    pub(super) window_max: f64,
    order: [AxisOrder; 3],
}

impl PacketBounds {
    /// The bounds of `rays[indices]`; `indices` is not empty.
    pub fn new(rays: &[Ray], indices: &[u32]) -> Self {
        let mut bounds = PacketBounds {
            origin_min: [f64::INFINITY; 3],
            origin_max: [f64::NEG_INFINITY; 3],
            inverse_min: [f64::INFINITY; 3],
            inverse_max: [f64::NEG_INFINITY; 3],
            unbounded: [false; 3],
            window_min: f64::INFINITY,
            window_max: f64::NEG_INFINITY,
            order: [AxisOrder::Mixed; 3],
        };
        let mut seen_right = [false; 3];
        let mut seen_left = [false; 3];
        for &index in indices {
            let ray = &rays[index as usize];
            if ray.min_distance_to_consider < bounds.window_min {
                bounds.window_min = ray.min_distance_to_consider;
            }
            if ray.max_distance_to_consider > bounds.window_max {
                bounds.window_max = ray.max_distance_to_consider;
            }
            for axis in 0..3 {
                let origin = ray.origin[axis];
                let inverse = ray.one_over_direction[axis];
                if origin < bounds.origin_min[axis] {
                    bounds.origin_min[axis] = origin;
                }
                if origin > bounds.origin_max[axis] {
                    bounds.origin_max[axis] = origin;
                }
                if !inverse.is_finite() {
                    bounds.unbounded[axis] = true;
                }
                if inverse < bounds.inverse_min[axis] {
                    bounds.inverse_min[axis] = inverse;
                }
                if inverse > bounds.inverse_max[axis] {
                    bounds.inverse_max[axis] = inverse;
                }
                if visits_right_first(ray, axis) {
                    seen_right[axis] = true;
                } else {
                    seen_left[axis] = true;
                }
            }
        }
        for axis in 0..3 {
            bounds.order[axis] = match (seen_right[axis], seen_left[axis]) {
                (true, false) => AxisOrder::RightFirst,
                (false, true) => AxisOrder::LeftFirst,
                _ => AxisOrder::Mixed,
            };
        }
        bounds
    }

    pub(super) fn axis_order(&self, axis: usize) -> AxisOrder {
        self.order[axis]
    }

    /// True when every ray these bounds cover is certain to miss `bounds` under
    /// `Ray::intersection`, whatever each ray's own window.
    pub fn misses(&self, bounds: &AxisAlignedBoundingBox) -> bool {
        let mut entry = [f64::NEG_INFINITY; 3];
        let mut exit = [f64::INFINITY; 3];
        for axis in 0..3 {
            if self.unbounded[axis] {
                continue;
            }
            let to_min = (
                bounds.min_xyz[axis] - self.origin_max[axis],
                bounds.min_xyz[axis] - self.origin_min[axis],
            );
            let to_max = (
                bounds.max_xyz[axis] - self.origin_max[axis],
                bounds.max_xyz[axis] - self.origin_min[axis],
            );
            let inverse = (self.inverse_min[axis], self.inverse_max[axis]);
            let (low, high) = if inverse.0 > 0.0 {
                (span(to_min, inverse).0, span(to_max, inverse).1)
            } else if inverse.1 < 0.0 {
                (span(to_max, inverse).0, span(to_min, inverse).1)
            } else {
                let min_span = span(to_min, inverse);
                let max_span = span(to_max, inverse);
                (
                    min_of(min_span.0, max_span.0),
                    max_of(min_span.1, max_span.1),
                )
            };
            entry[axis] = low;
            exit[axis] = high;
        }

        let latest_entry = max_of(max_of(entry[0], entry[1]), entry[2]);
        let earliest_exit = min_of(min_of(exit[0], exit[1]), exit[2]);
        latest_entry >= self.window_max
            || earliest_exit <= self.window_min
            || entry[0] > exit[1]
            || entry[0] > exit[2]
            || entry[1] > exit[0]
            || entry[1] > exit[2]
            || entry[2] > exit[0]
            || entry[2] > exit[1]
    }
}

fn min_of(a: f64, b: f64) -> f64 {
    if b < a {
        b
    } else {
        a
    }
}

fn max_of(a: f64, b: f64) -> f64 {
    if b > a {
        b
    } else {
        a
    }
}

/// The product of two intervals: the extremes of its four corner products. Each product is
/// rounded like the real one, and rounding is monotone, so the real product stays inside.
fn span(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    let (p0, p1, p2, p3) = (a.0 * b.0, a.0 * b.1, a.1 * b.0, a.1 * b.1);
    (
        min_of(min_of(min_of(p0, p1), p2), p3),
        max_of(max_of(max_of(p0, p1), p2), p3),
    )
}

/// A bundle of rays traced together and what became of each.
pub struct RayBundle<'a> {
    rays: Vec<Ray>,
    results: Vec<Option<IntersectInfo<'a>>>,
    stats: RayBundleStats,
    pub(super) hits: Vec<(u32, IntersectInfo<'a>)>,
    /// Ray index to stack position + 1 while hits are paired, or a stop mark; 0 otherwise.
    slot: Vec<u32>,
    pub(super) scratch: Vec<u32>,
    entry_windows: Vec<f64>,
    widened: Vec<u32>,
    bvh_depth: u32,
}

impl<'a> RayBundle<'a> {
    /// A bundle of `rays`, untraced.
    pub fn new(rays: Vec<Ray>) -> Self {
        let count = rays.len();
        RayBundle {
            rays,
            results: vec![None; count],
            stats: RayBundleStats::default(),
            hits: Vec::new(),
            slot: vec![0; count],
            scratch: Vec::new(),
            entry_windows: vec![0.0; count],
            widened: Vec::new(),
            bvh_depth: 0,
        }
    }

    /// Replaces the rays and clears the results and counters, keeping the allocations, so one
    /// bundle can trace a whole image a tile at a time.
    pub fn reset(&mut self, rays: impl IntoIterator<Item = Ray>) {
        self.rays.clear();
        self.rays.extend(rays);
        let count = self.rays.len();
        self.results.clear();
        self.results.resize(count, None);
        self.slot.clear();
        self.slot.resize(count, 0);
        self.entry_windows.clear();
        self.entry_windows.resize(count, 0.0);
        self.hits.clear();
        self.stats = RayBundleStats::default();
        self.widened.clear();
        self.bvh_depth = 0;
    }

    /// The rays, narrowed as tracing narrowed them, exactly as lone tracing would. A later
    /// trace starts from these windows, as one [`Ray`] traced twice does.
    pub fn rays(&self) -> &[Ray] {
        &self.rays
    }

    /// Each ray's result from the last [`RayBundle::trace`], in ray order: what
    /// `get_closest_intersection` returned for it alone.
    pub fn results(&self) -> &[Option<IntersectInfo<'a>>] {
        &self.results
    }

    /// The work counters since the last [`RayBundle::reset`].
    pub fn stats(&self) -> &RayBundleStats {
        &self.stats
    }

    /// Traces every ray into `shape`, replacing any earlier results; afterwards
    /// [`RayBundle::results`] holds what `shape.get_closest_intersection` returns for each ray
    /// alone. An error leaves every result `None`.
    pub fn trace(&mut self, shape: &'a dyn Traceable) -> Result<(), RayTracerError> {
        let all = self.begin_trace()?;
        shape.trace_ray_bundle(self, &all)?;
        for (index, hit) in self.hits.drain(..) {
            self.results[index as usize] = Some(hit);
        }
        Ok(())
    }

    /// Clears what an earlier trace or a direct [`Traceable::trace_ray_bundle`] call left, and
    /// returns every ray's index.
    fn begin_trace(&mut self) -> Result<Vec<u32>, RayTracerError> {
        let count = ray_index_count(self.rays.len())?;
        self.results.iter_mut().for_each(|result| *result = None);
        self.hits.clear();
        self.widened.clear();
        self.bvh_depth = 0;
        Ok((0..count).collect())
    }

    /// Traces every ray into `shapes` the way a lone ray's primary trace does: per ray, the
    /// nearest hit at a distance of at least 0, a miss being an [`IntersectInfo::new`].
    pub(super) fn trace_scene(&mut self, shapes: &'a [TraceableRef]) -> Result<(), RayTracerError> {
        let all = self.begin_trace()?;
        let mut best = vec![IntersectInfo::new(); self.rays.len()];
        for shape in shapes {
            shape.trace_ray_bundle(self, &all)?;
            for (index, info) in self.hits.drain(..) {
                let best_hit = &mut best[index as usize];
                if info.hit_type != IntersectionType::None
                    && info.distance_to_hit < best_hit.distance_to_hit
                    && info.distance_to_hit >= 0.0
                {
                    *best_hit = info;
                }
            }
        }
        for (result, info) in self.results.iter_mut().zip(best) {
            *result = Some(info);
        }
        Ok(())
    }

    /// Runs ray `index` alone through `shape` and records what it returned: the body of
    /// [`Traceable::trace_ray_bundle`]'s default, and how an override hands a ray over. The
    /// entry stays on the bundle's internal stack until the next [`RayBundle::trace`], which
    /// discards it; results are read after a `trace`, not after a direct call.
    pub fn trace_one<S: Traceable + ?Sized>(
        &mut self,
        index: u32,
        shape: &'a S,
    ) -> Result<(), RayTracerError> {
        self.check_indices(std::slice::from_ref(&index))?;
        self.stats.leaf_tests += 1;
        self.trace_alone(index, shape);
        Ok(())
    }

    /// Refuses a slice holding a ray index past the end of this bundle.
    pub(super) fn check_indices(&self, active: &[u32]) -> Result<(), RayTracerError> {
        if active
            .iter()
            .any(|&index| index as usize >= self.rays.len())
        {
            return Err(RayTracerError::InvalidArgument(
                "a ray index is past the end of the bundle's rays",
            ));
        }
        Ok(())
    }

    /// Runs ray `index` alone through `shape` as a node's fallback.
    pub(super) fn trace_lone(&mut self, index: u32, shape: &'a (impl Traceable + ?Sized)) {
        self.stats.lone_traces += 1;
        self.trace_alone(index, shape);
    }

    fn trace_alone<S: Traceable + ?Sized>(&mut self, index: u32, shape: &'a S) {
        let ray = &mut self.rays[index as usize];
        let window_before = ray.max_distance_to_consider;
        let hit = shape.get_closest_intersection(ray);
        if is_wider(ray.max_distance_to_consider, window_before) {
            self.note_widened(index);
        }
        if let Some(hit) = hit {
            self.hits.push((index, hit));
        }
    }

    // ---- For the packet traversal and the containers. ----

    pub(super) fn all_rays(&self) -> &[Ray] {
        &self.rays
    }

    pub(super) fn stats_mut(&mut self) -> &mut RayBundleStats {
        &mut self.stats
    }

    /// A ray the packet traversal can skip and bound: a finite origin and direction and no NaN
    /// in its window (an infinite window is the default and is regular), so a slab test's only NaN
    /// is the `0 * inf` of an origin exactly on a box plane along a zero direction component.
    pub(super) fn ray_is_regular(&self, index: u32) -> bool {
        let ray = &self.rays[index as usize];
        (0..3).all(|axis| ray.origin[axis].is_finite() && ray.direction_normal[axis].is_finite())
            && !ray.min_distance_to_consider.is_nan()
            && !ray.max_distance_to_consider.is_nan()
    }

    /// The real test a lone ray makes at a node.
    pub(super) fn ray_hits_box(&mut self, index: u32, bounds: &AxisAlignedBoundingBox) -> bool {
        self.stats.box_tests += 1;
        self.rays[index as usize].intersection(bounds)
    }

    /// The position in `active` of the first ray hitting `bounds`, `None` when none does.
    pub(super) fn find_first(
        &mut self,
        bounds: &AxisAlignedBoundingBox,
        active: &[u32],
        packet: &PacketBounds,
    ) -> Option<usize> {
        if self.ray_hits_box(active[0], bounds) {
            self.stats.early_hits += 1;
            return Some(0);
        }
        if active.len() > 1 {
            self.stats.interval_tests += 1;
            if packet.misses(bounds) {
                self.stats.interval_misses += 1;
                return None;
            }
        }
        self.stats.last_resorts += 1;
        for (position, &index) in active.iter().enumerate().skip(1) {
            self.stats.last_resort_rays += 1;
            if self.ray_hits_box(index, bounds) {
                return Some(position);
            }
        }
        None
    }

    /// Narrows the rays in `hits[base..mid]` to their hits the way a node does after its near
    /// child. True when a shadow ray stopped there; those rays stay marked for
    /// [`RayBundle::stopped`] until [`RayBundle::clear_stopped`].
    pub(super) fn narrow_after_near(&mut self, base: usize, mid: usize) -> bool {
        let mut any_stopped = false;
        for position in base..mid {
            let (index, hit) = self.hits[position];
            if hit.hit_type == IntersectionType::None {
                continue;
            }
            if self.rays[index as usize].is_shadow_ray {
                self.slot[index as usize] = 1;
                any_stopped = true;
            } else {
                narrow_ray(
                    &mut self.rays,
                    &mut self.widened,
                    self.bvh_depth,
                    index,
                    hit.distance_to_hit,
                );
            }
        }
        any_stopped
    }

    pub(super) fn stopped(&self, index: u32) -> bool {
        self.slot[index as usize] != 0
    }

    pub(super) fn clear_stopped(&mut self, base: usize, mid: usize) {
        for position in base..mid {
            let index = self.hits[position].0 as usize;
            self.slot[index] = 0;
        }
    }

    /// Merges two ranges of the stack: the first child's entries in `hits[base..mid]`, the
    /// second's in `hits[mid..]`, keeping per ray what `pick` chooses.
    pub(super) fn combine_hits<F>(&mut self, base: usize, mid: usize, mut pick: F)
    where
        F: FnMut(
            &mut NarrowContext<'_>,
            u32,
            Option<&IntersectInfo<'a>>,
            Option<&IntersectInfo<'a>>,
        ) -> Pick,
    {
        let end = self.hits.len();
        let mut narrow = NarrowContext {
            rays: &mut self.rays,
            widened: &mut self.widened,
            bvh_depth: self.bvh_depth,
        };
        for position in mid..end {
            // Relative to `mid`, so it fits a u32: a merge's second range holds at most one
            // entry per ray of the bundle. An absolute position could exceed u32 on a big bundle.
            self.slot[self.hits[position].0 as usize] = (position - mid + 1) as u32;
        }
        let mut write = base;
        for position in base..mid {
            let (index, first) = self.hits[position];
            let paired = self.slot[index as usize];
            let second = if paired == 0 {
                None
            } else {
                self.slot[index as usize] = 0;
                Some(self.hits[mid + paired as usize - 1].1)
            };
            match pick(&mut narrow, index, Some(&first), second.as_ref()) {
                Pick::First => {
                    self.hits[write] = (index, first);
                    write += 1;
                }
                Pick::Second => {
                    if let Some(second) = second {
                        self.hits[write] = (index, second);
                        write += 1;
                    }
                }
                Pick::Neither => {}
            }
        }
        for position in mid..end {
            let (index, second) = self.hits[position];
            if self.slot[index as usize] == 0 {
                continue;
            }
            self.slot[index as usize] = 0;
            if let Pick::Second = pick(&mut narrow, index, None, Some(&second)) {
                self.hits[write] = (index, second);
                write += 1;
            }
        }
        self.hits.truncate(write);
    }

    /// Records that a BVH subtree began; the outermost keeps each active ray's window so a ray
    /// whose window widens can be retraced alone (see [`RayBundle::note_widened`]).
    pub(super) fn enter_bvh(&mut self, active: &[u32]) {
        if self.bvh_depth == 0 {
            self.widened.clear();
            for &index in active {
                self.entry_windows[index as usize] =
                    self.rays[index as usize].max_distance_to_consider;
            }
        }
        self.bvh_depth += 1;
    }

    /// The end of a BVH subtree begun with [`RayBundle::enter_bvh`]; true when it was the
    /// outermost.
    pub(super) fn leave_bvh(&mut self) -> bool {
        self.bvh_depth -= 1;
        self.bvh_depth == 0
    }

    /// A shape widened this ray's window ([`super::PlaneShape`] has no upper limit, and the
    /// node above it copies its distance over the ray's). A skipped box test relies on a
    /// window that only narrows, so such a ray is retraced alone once its BVH is done.
    pub(super) fn note_widened(&mut self, index: u32) {
        if self.bvh_depth > 0 {
            self.widened.push(index);
        }
    }

    /// Replaces the outcome of every ray whose window widened with a lone trace from its window
    /// at this BVH's entry. Hits above `base` belong to this BVH.
    pub(super) fn retrace_widened<S: Traceable + ?Sized>(&mut self, base: usize, shape: &'a S) {
        if self.widened.is_empty() {
            return;
        }
        let mut widened = std::mem::take(&mut self.widened);
        widened.sort_unstable();
        widened.dedup();
        for &index in &widened {
            self.slot[index as usize] = 1;
        }
        let end = self.hits.len();
        let mut write = base;
        for position in base..end {
            let entry = self.hits[position];
            if self.slot[entry.0 as usize] == 0 {
                self.hits[write] = entry;
                write += 1;
            }
        }
        self.hits.truncate(write);
        for &index in &widened {
            self.slot[index as usize] = 0;
            let ray = &mut self.rays[index as usize];
            ray.max_distance_to_consider = self.entry_windows[index as usize];
            let hit = shape.get_closest_intersection(ray);
            self.stats.retraced_rays += 1;
            if let Some(hit) = hit {
                self.hits.push((index, hit));
            }
        }
        widened.clear();
        self.widened = widened;
    }

    /// A bundle of `rays` sharing this one's stack and counters: a [`super::Transform`]'s rays
    /// in its child's space. The caller returns it through [`RayBundle::finish_local`].
    pub(super) fn local(&mut self, rays: Vec<Ray>) -> RayBundle<'a> {
        let mut local = RayBundle::new(rays);
        local.hits = std::mem::take(&mut self.hits);
        local.stats = self.stats;
        local
    }

    /// Ends a [`RayBundle::local`] bundle: every entry at or above `base` becomes this bundle's
    /// entry for `active[local index]` with its hit mapped by `to_global`.
    pub(super) fn finish_local(
        &mut self,
        mut local: RayBundle<'a>,
        base: usize,
        active: &[u32],
        mut to_global: impl FnMut(IntersectInfo<'a>) -> IntersectInfo<'a>,
    ) {
        for entry in &mut local.hits[base..] {
            entry.0 = active[entry.0 as usize];
            entry.1 = to_global(entry.1);
        }
        self.hits = std::mem::take(&mut local.hits);
        self.stats = local.stats;
    }
}

/// How many rays or active indices fit a `u32` index, refused past that: the `as u32` casts
/// that follow are then exact.
pub(super) fn ray_index_count(count: usize) -> Result<u32, RayTracerError> {
    u32::try_from(count)
        .map_err(|_| RayTracerError::InvalidArgument("a ray bundle holds at most 4294967295 rays"))
}

/// What a merge may change on a ray: its window.
pub(super) struct NarrowContext<'b> {
    rays: &'b mut [Ray],
    widened: &'b mut Vec<u32>,
    bvh_depth: u32,
}

impl NarrowContext<'_> {
    pub fn is_shadow_ray(&self, index: u32) -> bool {
        self.rays[index as usize].is_shadow_ray
    }

    /// Sets the ray's window to `distance`, as a node does on a hit.
    pub fn narrow(&mut self, index: u32, distance: f64) {
        narrow_ray(self.rays, self.widened, self.bvh_depth, index, distance);
    }
}

/// True when a window's new upper end is not a narrowing: larger, or NaN (a comparison with NaN
/// is false both ways, so this is written as the negative of the case that is fine).
fn is_wider(new_max: f64, old_max: f64) -> bool {
    !matches!(
        new_max.partial_cmp(&old_max),
        Some(std::cmp::Ordering::Less | std::cmp::Ordering::Equal)
    )
}

/// The one way a packet traversal narrows a ray. A value that is not smaller (a shape that
/// ignores the window, or a NaN) would let a ray pass a box its earlier window failed.
fn narrow_ray(rays: &mut [Ray], widened: &mut Vec<u32>, bvh_depth: u32, index: u32, distance: f64) {
    let ray = &mut rays[index as usize];
    if is_wider(distance, ray.max_distance_to_consider) && bvh_depth > 0 {
        widened.push(index);
    }
    ray.max_distance_to_consider = distance;
}
