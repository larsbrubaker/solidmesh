//! The surface-area-heuristic builder:
//! sorts the items on each axis by `center`, sweeps every split point, and keeps the cheapest
//! split; groups larger than `do_simple_sort_size` split at the median on a rotating axis.
//!
//! A plain item costs one box test; [`TradeOffBvhConstructor::create_new_hierarchy`] is the
//! path for plain items and [`TradeOffBvhConstructor::create_new_hierarchy_intersectable`] the
//! one for [`Intersectable`] items, which report their own cost. The items are taken by value,
//! so the caller's list is never reordered. Sorting uses a fixed introsort
//! ([`crate::fixed_tie_sort`]) so tied centers always end up in the same place and the tree's shape is
//! the same on every platform. `do_simple_sort_size` applies at every level of the tree.

use super::{BvhTree, BvhTreeItemData, Intersectable};
use crate::fixed_tie_sort::introspective_sort;
use crate::{AxisAlignedBoundingBox, Vector3};
use std::cmp::Ordering;

/// Rotates the axis used for groups too large for the full sweep.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SortingAccelerator {
    pub next_axis_for_big_groups: usize,
}

impl Default for SortingAccelerator {
    fn default() -> Self {
        SortingAccelerator {
            next_axis_for_big_groups: 2,
        }
    }
}

impl SortingAccelerator {
    pub fn new() -> Self {
        Self::default()
    }

    /// Advances to and returns the next axis.
    pub fn next_axis(&mut self) -> usize {
        self.next_axis_for_big_groups = (self.next_axis_for_big_groups + 1) % 3;
        self.next_axis_for_big_groups
    }
}

/// The builder.
pub struct TradeOffBvhConstructor;

impl TradeOffBvhConstructor {
    /// The default `max_recursion`: no depth limit.
    pub const DEFAULT_MAX_RECURSION: i32 = i32::MAX;
    /// The default `do_simple_sort_size`: larger groups split at the median instead of sweeping.
    pub const DEFAULT_DO_SIMPLE_SORT_SIZE: usize = 5000;

    /// Builds a tree over items that each cost one bounds test; `None` for no items. Usual
    /// arguments: `max_recursion` [`Self::DEFAULT_MAX_RECURSION`], `recursion_depth` 0,
    /// `accelerator` a new one, `do_simple_sort_size` [`Self::DEFAULT_DO_SIMPLE_SORT_SIZE`].
    pub fn create_new_hierarchy<T>(
        items_to_add: Vec<BvhTreeItemData<T>>,
        max_recursion: i32,
        recursion_depth: i32,
        accelerator: Option<&mut SortingAccelerator>,
        do_simple_sort_size: usize,
    ) -> Option<BvhTree<T>> {
        let mut new_accelerator = SortingAccelerator::new();
        let accelerator = accelerator.unwrap_or(&mut new_accelerator);
        build(
            items_to_add,
            max_recursion,
            recursion_depth,
            accelerator,
            do_simple_sort_size,
            &|_| AxisAlignedBoundingBox::get_intersect_cost(),
        )
    }

    /// [`Self::create_new_hierarchy`] for items that know their own intersect cost.
    pub fn create_new_hierarchy_intersectable<T: Intersectable>(
        items_to_add: Vec<BvhTreeItemData<T>>,
        max_recursion: i32,
        recursion_depth: i32,
        accelerator: Option<&mut SortingAccelerator>,
        do_simple_sort_size: usize,
    ) -> Option<BvhTree<T>> {
        let mut new_accelerator = SortingAccelerator::new();
        let accelerator = accelerator.unwrap_or(&mut new_accelerator);
        build(
            items_to_add,
            max_recursion,
            recursion_depth,
            accelerator,
            do_simple_sort_size,
            &|item: &T| item.get_intersect_cost(),
        )
    }
}

/// Sorts by `center[axis]`, equal when neither is greater (so NaN ties with everything).
fn sort_on_axis<T>(items: &mut [BvhTreeItemData<T>], which_axis: usize) {
    let which_axis = which_axis % 3;
    introspective_sort(items, &mut |a, b| {
        let axis_center_a = a.center[which_axis];
        let axis_center_b = b.center[which_axis];
        if axis_center_a > axis_center_b {
            Ordering::Greater
        } else if axis_center_a < axis_center_b {
            Ordering::Less
        } else {
            Ordering::Equal
        }
    });
}

fn build<T>(
    mut items_to_add: Vec<BvhTreeItemData<T>>,
    max_recursion: i32,
    recursion_depth: i32,
    accelerator: &mut SortingAccelerator,
    do_simple_sort_size: usize,
    item_cost: &dyn Fn(&T) -> f64,
) -> Option<BvhTree<T>> {
    let num_items = items_to_add.len();

    if num_items == 0 {
        return None;
    }

    if num_items == 1 {
        return Some(BvhTree::from_items(items_to_add));
    }

    let mut best_axis: Option<usize> = None;
    let mut best_index_to_split_on = 0;

    if recursion_depth < max_recursion {
        if num_items > do_simple_sort_size {
            best_axis = Some(accelerator.next_axis());
            // The split keeps items 0..=index on the left, so index n/2 - 1 halves the group
            // and leaves both sides non-empty (n/2 would put both of two items on the left
            // and recurse on the same group forever).
            best_index_to_split_on = num_items / 2 - 1;
        } else {
            let mut total_intersect_cost = 0.0;
            for item in &items_to_add {
                total_intersect_cost += item_cost(&item.item);
            }

            // get the bounding box of all the items we are going to consider.
            let mut overall_box = items_to_add[0].aabb;
            for item in &items_to_add[1..] {
                overall_box = overall_box + item.aabb;
            }

            let area_of_total_bounds = overall_box.get_surface_area();

            let mut best_cost = total_intersect_cost;

            let mut total_deviation_on_axis = Vector3::ZERO;
            let mut surface_arrea_of_item = vec![0.0; num_items - 1];
            let mut right_bounds_at_item = vec![0.0; num_items - 1];

            for axis in 0..3 {
                let mut intersect_cost_on_left = 0.0;

                sort_on_axis(&mut items_to_add, axis);

                // Get all left bounds
                let mut current_left_bounds = items_to_add[0].aabb;
                surface_arrea_of_item[0] = current_left_bounds.get_surface_area();
                for item_index in 1..num_items - 1 {
                    current_left_bounds = current_left_bounds + items_to_add[item_index].aabb;
                    surface_arrea_of_item[item_index] = current_left_bounds.get_surface_area();

                    total_deviation_on_axis[axis] += (items_to_add[item_index].center[axis]
                        - items_to_add[item_index - 1].center[axis])
                        .abs();
                }

                // Get all right bounds
                if num_items > 1 {
                    let mut current_right_bounds = items_to_add[num_items - 1].aabb;
                    right_bounds_at_item[num_items - 2] = current_right_bounds.get_surface_area();
                    let mut item_index = num_items - 1;
                    while item_index > 1 {
                        current_right_bounds =
                            current_right_bounds + items_to_add[item_index - 1].aabb;
                        right_bounds_at_item[item_index - 2] =
                            current_right_bounds.get_surface_area();
                        item_index -= 1;
                    }
                }

                // Sweep from left
                for item_index in 0..num_items - 1 {
                    // Evaluate Surface Cost Equation
                    // the cost of the two children AABB tests
                    let cost_of_two_aabb = 2.0 * AxisAlignedBoundingBox::get_intersect_cost();

                    // do the left cost
                    intersect_cost_on_left += item_cost(&items_to_add[item_index].item);
                    let left_cost = (surface_arrea_of_item[item_index] / area_of_total_bounds)
                        * intersect_cost_on_left;

                    // do the right cost
                    let intersect_cost_on_right = total_intersect_cost - intersect_cost_on_left;
                    let right_cost = (right_bounds_at_item[item_index] / area_of_total_bounds)
                        * intersect_cost_on_right;

                    let this_cost = cost_of_two_aabb + left_cost + right_cost;

                    // if it is less within some tiny error
                    if this_cost < best_cost + 0.000000001 {
                        if this_cost > best_cost - 0.000000001 {
                            // they are the same within the error
                            // we have changed axis since last best and we need to decide if this
                            // is better than the last axis best
                            if axis > 0
                                && best_axis != Some(axis)
                                && total_deviation_on_axis[axis] > total_deviation_on_axis[axis - 1]
                            {
                                // this new axis is better and we'll switch to it. Otherwise
                                // don't switch.
                                best_cost = this_cost;
                                best_index_to_split_on = item_index;
                                best_axis = Some(axis);
                            }
                        } else {
                            // this is just better
                            best_cost = this_cost;
                            best_index_to_split_on = item_index;
                            best_axis = Some(axis);
                        }
                    }
                }
            }
        }
    }

    match best_axis {
        // No better partition found
        None => Some(BvhTree::from_items(items_to_add)),
        Some(best_axis) => {
            sort_on_axis(&mut items_to_add, best_axis);
            let right_items = items_to_add.split_off(best_index_to_split_on + 1);
            let left_items = items_to_add;
            let left_group = build(
                left_items,
                max_recursion,
                recursion_depth + 1,
                accelerator,
                do_simple_sort_size,
                item_cost,
            );
            let right_group = build(
                right_items,
                max_recursion,
                recursion_depth + 1,
                accelerator,
                do_simple_sort_size,
                item_cost,
            );
            match (left_group, right_group) {
                (Some(left_group), Some(right_group)) => {
                    Some(BvhTree::from_nodes(left_group, right_group, best_axis))
                }
                // The split index is at most n - 2, so neither side is empty; should one ever
                // be, the other side holds every item and is the tree.
                (Some(group), None) | (None, Some(group)) => Some(group),
                (None, None) => None,
            }
        }
    }
}
