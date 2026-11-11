//! The packet traversal of a [`BoundingVolumeHierarchy`]: one pass of a whole
//! [`RayBundle`](super::RayBundle) down the tree, testing only as many rays at each node as it
//! takes to know what to do there, while every ray still gets exactly the result it would get
//! alone.
//!
//! At each node the first ray not yet known to have missed is tested (hit: descend); failing
//! that, an interval-arithmetic test over all the rays can prove none of them hits; failing
//! that, the rest are tested one by one until one hits. A node with a leaf child, and every few
//! levels a node on the way down (`PRUNE_EVERY_LEVELS`), then tests the rays the first ray's
//! early hit carried past their own test, which also drops the ones that have fallen behind.
//!
//! Why a ray's result cannot change:
//! * **Each ray sees the children in its own order.** The rays are split wherever they disagree
//!   on a node's axis sign, since which of two equally distant triangles wins depends on it.
//! * **A ray whose box test was skipped never reaches a leaf it would not have.** A node with a
//!   leaf child tests each ray against its own box on the way in, before anything below
//!   narrows the ray: the test a lone ray makes there. If that passes, the lone ray also passed
//!   every ancestor, because a child's box is inside its parent's and a ray's window only
//!   shrinks while it descends. Entry and exit distances are computed by operations monotone
//!   under rounding. Along a zero direction component a slab either constrains nothing (the
//!   origin inside it, or exactly on one of its planes, which the slab test counts as touching)
//!   or is a miss (the origin outside it), on every axis alike; a ray inside or on the child's
//!   slab is inside or on the parent's. So a ray that fails a box can be skipped below it, and
//!   one that a parent's early hit carried past its test is caught here.
//! * **Rays are narrowed in the order a lone ray narrows.** Per ray, leaf visits happen in the
//!   same depth-first order, and a node's near child narrows it before its far child starts.
//!
//! Three things fall back to tracing each ray alone: a node whose box is not sound (NaN, inverted,
//! or a child's box not inside it: the interval test and the skipped-test proof assume a sound
//! box), a ray with a non-finite origin or direction or a NaN window (an infinite window is the
//! default and is fine), and a ray whose window widened (a [`super::PlaneShape`] ignores its
//! upper limit).

use solidmesh_math::IntersectionType;

use super::bounding_volume_hierarchy::BoundingVolumeHierarchy;
use super::ray_bundle::{visits_right_first, AxisOrder, NarrowContext, PacketBounds, Pick};
use super::{IntersectInfo, RayBundle, RayTracerError, TraceableRef};

/// How many levels a ray may ride along untested before the next node tests it. An early hit
/// carries every active ray down untested; a ray that really missed several levels up keeps
/// being carried until a leaf parent tests it, which on small triangles costs more than the
/// ray-box tests saved. Pruning every few levels throws those rays out at a small cost; the
/// values were measured (the `ray_bundle_bench` example), and pruning is only ever a real box test, so it
/// cannot change a result.
const PRUNE_EVERY_LEVELS: u32 = 3;

/// Too few rays to bother pruning: the leaf-parent test handles them directly.
const PRUNE_MIN_RAYS: usize = 8;

/// True when `info` is a real hit.
fn is_real_hit(info: &IntersectInfo<'_>) -> bool {
    info.hit_type != IntersectionType::None
}

/// Merges a node's two children for one ray: the same decisions the lone-ray path makes, in
/// the same order.
fn merge_children<'a>(
    narrow: &mut NarrowContext<'_>,
    index: u32,
    first: Option<&IntersectInfo<'a>>,
    second: Option<&IntersectInfo<'a>>,
) -> Pick {
    if first.is_some_and(is_real_hit) && narrow.is_shadow_ray(index) {
        return Pick::First;
    }
    if let Some(second) = second {
        if is_real_hit(second) {
            if narrow.is_shadow_ray(index) {
                return Pick::Second;
            }
            narrow.narrow(index, second.distance_to_hit);
        }
    }
    if let Some(first) = first.filter(|first| is_real_hit(first) && first.distance_to_hit >= 0.0) {
        if let Some(second) = second {
            if is_real_hit(second)
                && second.distance_to_hit < first.distance_to_hit
                && second.distance_to_hit >= 0.0
            {
                return Pick::Second;
            }
        }
        return Pick::First;
    }
    if second.is_some() {
        Pick::Second
    } else {
        Pick::Neither
    }
}

impl BoundingVolumeHierarchy {
    /// Traces `active` through this node: regular rays together, the rest alone.
    pub(super) fn trace_bundle_entry<'a>(
        &'a self,
        bundle: &mut RayBundle<'a>,
        active: &[u32],
    ) -> Result<(), RayTracerError> {
        bundle.check_indices(active)?;
        if !self.monotone {
            for &index in active {
                bundle.trace_lone(index, self);
            }
            return Ok(());
        }

        let base = bundle.hits.len();
        bundle.enter_bvh(active);
        let mut work = std::mem::take(&mut bundle.scratch);
        work.clear();
        for &index in active {
            if bundle.ray_is_regular(index) {
                work.push(index);
            } else {
                bundle.trace_lone(index, self);
            }
        }
        let mut outcome = Ok(());
        if !work.is_empty() {
            let bounds = PacketBounds::new(bundle.all_rays(), &work);
            let end = work.len();
            outcome = self.bundle_node(bundle, &mut work, 0, end, &bounds, 0);
        }
        work.clear();
        bundle.scratch = work;
        let outermost = bundle.leave_bvh();
        outcome?;
        if outermost {
            bundle.retrace_widened(base, self);
        }
        Ok(())
    }

    fn left_node(&self) -> Option<&BoundingVolumeHierarchy> {
        if self.left_descends {
            self.left.as_any().downcast_ref()
        } else {
            None
        }
    }

    fn right_node(&self) -> Option<&BoundingVolumeHierarchy> {
        if self.right_descends {
            self.right.as_any().downcast_ref()
        } else {
            None
        }
    }

    /// One node of the traversal. `work[start..end]` are the rays not known to have missed.
    fn bundle_node<'a>(
        &'a self,
        bundle: &mut RayBundle<'a>,
        work: &mut Vec<u32>,
        start: usize,
        end: usize,
        bounds: &PacketBounds,
        since_pruned: u32,
    ) -> Result<(), RayTracerError> {
        let Some(first) = bundle.find_first(&self.aabb, &work[start..end], bounds) else {
            return Ok(());
        };
        let frame = work.len();
        let mut list = (start + first, end);
        let (left_node, right_node) = (self.left_node(), self.right_node());
        let mut since_pruned = since_pruned + 1;
        if left_node.is_none()
            || right_node.is_none()
            || (since_pruned >= PRUNE_EVERY_LEVELS && list.1 - list.0 > PRUNE_MIN_RAYS)
        {
            list = self.gate_at_leaf_parent(bundle, work, list.0, list.1);
            since_pruned = 0;
        }

        let axis = self.splitting_plane;
        let outcome = match bounds.axis_order(axis) {
            AxisOrder::LeftFirst => self.visit_children(
                bundle,
                work,
                list,
                bounds,
                false,
                left_node,
                right_node,
                since_pruned,
            ),
            AxisOrder::RightFirst => self.visit_children(
                bundle,
                work,
                list,
                bounds,
                true,
                left_node,
                right_node,
                since_pruned,
            ),
            AxisOrder::Mixed => {
                let split_base = work.len();
                for position in list.0..list.1 {
                    let index = work[position];
                    if visits_right_first(&bundle.all_rays()[index as usize], axis) {
                        work.push(index);
                    }
                }
                let split_mid = work.len();
                for position in list.0..list.1 {
                    let index = work[position];
                    if !visits_right_first(&bundle.all_rays()[index as usize], axis) {
                        work.push(index);
                    }
                }
                let split_end = work.len();
                if split_mid > split_base {
                    let sub = PacketBounds::new(bundle.all_rays(), &work[split_base..split_mid]);
                    let group = (split_base, split_mid);
                    self.visit_children(
                        bundle,
                        work,
                        group,
                        &sub,
                        true,
                        left_node,
                        right_node,
                        since_pruned,
                    )?;
                }
                if split_end > split_mid {
                    let sub = PacketBounds::new(bundle.all_rays(), &work[split_mid..split_end]);
                    let group = (split_mid, split_end);
                    self.visit_children(
                        bundle,
                        work,
                        group,
                        &sub,
                        false,
                        left_node,
                        right_node,
                        since_pruned,
                    )?;
                }
                Ok(())
            }
        };
        work.truncate(frame);
        outcome
    }

    /// The rays that really reach this node's children, tested against the node's own box
    /// now, before either child narrows them: a lone ray makes this test on its way in, and a
    /// near child's hit on the far side of a tie must not change its answer. `start`'s ray
    /// already passed.
    fn gate_at_leaf_parent<'a>(
        &self,
        bundle: &mut RayBundle<'a>,
        work: &mut Vec<u32>,
        start: usize,
        end: usize,
    ) -> (usize, usize) {
        let gated = work.len();
        work.push(work[start]);
        for position in start + 1..end {
            let index = work[position];
            bundle.stats_mut().gate_tests += 1;
            if bundle.ray_hits_box(index, &self.aabb) {
                work.push(index);
            }
        }
        (gated, work.len())
    }

    /// Visits both children for `list`, near first, narrowing rays by the near hits and
    /// merging the two sets of results.
    #[allow(clippy::too_many_arguments)]
    fn visit_children<'a>(
        &'a self,
        bundle: &mut RayBundle<'a>,
        work: &mut Vec<u32>,
        list: (usize, usize),
        bounds: &PacketBounds,
        right_first: bool,
        left_node: Option<&'a BoundingVolumeHierarchy>,
        right_node: Option<&'a BoundingVolumeHierarchy>,
        since_pruned: u32,
    ) -> Result<(), RayTracerError> {
        let (near, far, near_node, far_node) = if right_first {
            (&self.right, &self.left, right_node, left_node)
        } else {
            (&self.left, &self.right, left_node, right_node)
        };

        let base = bundle.hits.len();
        Self::visit(near, near_node, bundle, work, list, bounds, since_pruned)?;
        let mid = bundle.hits.len();

        let frame = work.len();
        let mut far_list = list;
        let mut far_bounds = *bounds;
        if mid > base {
            if bundle.narrow_after_near(base, mid) {
                for position in list.0..list.1 {
                    let index = work[position];
                    if !bundle.stopped(index) {
                        work.push(index);
                    }
                }
                bundle.clear_stopped(base, mid);
                far_list = (frame, work.len());
            }
            if far_list.1 > far_list.0 {
                // The rays the hits narrowed can only shrink the group's window: a tighter bound
                // lets the interval test cull the part of the far child behind those hits.
                let mut window_max = f64::NEG_INFINITY;
                for &index in &work[far_list.0..far_list.1] {
                    let max = bundle.all_rays()[index as usize].max_distance_to_consider;
                    if max > window_max {
                        window_max = max;
                    }
                }
                far_bounds.window_max = window_max;
            }
        }

        if far_list.1 > far_list.0 {
            Self::visit(
                far,
                far_node,
                bundle,
                work,
                far_list,
                &far_bounds,
                since_pruned,
            )?;
        }
        work.truncate(frame);
        bundle.combine_hits(base, mid, merge_children);
        Ok(())
    }

    fn visit<'a>(
        child: &'a TraceableRef,
        node: Option<&'a BoundingVolumeHierarchy>,
        bundle: &mut RayBundle<'a>,
        work: &mut Vec<u32>,
        (start, end): (usize, usize),
        bounds: &PacketBounds,
        since_pruned: u32,
    ) -> Result<(), RayTracerError> {
        match node {
            Some(node) => node.bundle_node(bundle, work, start, end, bounds, since_pruned),
            None => child.trace_ray_bundle(bundle, &work[start..end]),
        }
    }
}
