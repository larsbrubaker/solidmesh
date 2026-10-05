//! .NET's unstable introspective sort (`ArraySortHelper<T>.IntrospectiveSort`, the algorithm
//! behind C# `List<T>.Sort(IComparer)` and `Array.Sort(T[], Comparison)`), ported from the
//! .NET runtime (MIT). The BVH builders sort items whose keys often tie (agg-sharp never sets
//! `BvhTreeItemData.Center`, so every key is 0), and the order .NET leaves tied items in decides
//! the tree's shape and the order searches report items. Rust's sorts order ties differently,
//! so this keeps the trees identical to MatterCAD's.
//!
//! The .NET code moves elements through a held temporary ("hole" moves) in insertion sort and
//! the heap's sift-down; here the same moves are made with adjacent swaps, which leave every
//! element in the same final place.

use std::cmp::Ordering;

const INTROSORT_SIZE_THRESHOLD: usize = 16;

/// Sorts `keys` exactly as .NET's `Array.Sort(keys, comparer)` does.
pub fn introspective_sort<T>(keys: &mut [T], comparer: &mut impl FnMut(&T, &T) -> Ordering) {
    if keys.len() > 1 {
        let depth_limit = 2 * ((keys.len() as u32).ilog2() as i32 + 1);
        intro_sort(keys, depth_limit, comparer);
    }
}

fn intro_sort<T>(
    keys: &mut [T],
    mut depth_limit: i32,
    comparer: &mut impl FnMut(&T, &T) -> Ordering,
) {
    let mut partition_size = keys.len();
    while partition_size > 1 {
        if partition_size <= INTROSORT_SIZE_THRESHOLD {
            if partition_size == 2 {
                swap_if_greater(keys, comparer, 0, 1);
                return;
            }

            if partition_size == 3 {
                swap_if_greater(keys, comparer, 0, 1);
                swap_if_greater(keys, comparer, 0, 2);
                swap_if_greater(keys, comparer, 1, 2);
                return;
            }

            insertion_sort(&mut keys[..partition_size], comparer);
            return;
        }

        if depth_limit == 0 {
            heap_sort(&mut keys[..partition_size], comparer);
            return;
        }

        depth_limit -= 1;

        let p = pick_pivot_and_partition(&mut keys[..partition_size], comparer);

        // Note we've already partitioned around the pivot and do not have to move the pivot again.
        intro_sort(&mut keys[p + 1..partition_size], depth_limit, comparer);
        partition_size = p;
    }
}

fn swap_if_greater<T>(
    keys: &mut [T],
    comparer: &mut impl FnMut(&T, &T) -> Ordering,
    i: usize,
    j: usize,
) {
    if comparer(&keys[i], &keys[j]) == Ordering::Greater {
        keys.swap(i, j);
    }
}

fn pick_pivot_and_partition<T>(
    keys: &mut [T],
    comparer: &mut impl FnMut(&T, &T) -> Ordering,
) -> usize {
    let hi = keys.len() - 1;

    // Compute median-of-three. But also partition them, since we've done the comparison.
    let middle = hi >> 1;

    // Sort lo, mid and hi appropriately, then pick mid as the pivot.
    swap_if_greater(keys, comparer, 0, middle); // swap the low with the mid point
    swap_if_greater(keys, comparer, 0, hi); // swap the low with the high
    swap_if_greater(keys, comparer, middle, hi); // swap the middle with the high

    // The pivot is parked at hi - 1 and stays there until the end, so compare against it there.
    keys.swap(middle, hi - 1);
    let pivot = hi - 1;
    // We already partitioned lo and hi and put the pivot in hi - 1. And we pre-increment &
    // decrement below.
    let mut left = 0;
    let mut right = hi - 1;

    while left < right {
        loop {
            left += 1;
            if comparer(&keys[left], &keys[pivot]) != Ordering::Less {
                break;
            }
        }
        loop {
            right -= 1;
            if comparer(&keys[pivot], &keys[right]) != Ordering::Less {
                break;
            }
        }

        if left >= right {
            break;
        }

        keys.swap(left, right);
    }

    // Put pivot in the right location.
    if left != hi - 1 {
        keys.swap(left, hi - 1);
    }

    left
}

fn heap_sort<T>(keys: &mut [T], comparer: &mut impl FnMut(&T, &T) -> Ordering) {
    let n = keys.len();
    let mut i = n >> 1;
    while i >= 1 {
        down_heap(keys, i, n, comparer);
        i -= 1;
    }

    let mut i = n;
    while i > 1 {
        keys.swap(0, i - 1);
        down_heap(keys, 1, i - 1, comparer);
        i -= 1;
    }
}

/// Sift-down with 1-based `i`; the .NET version carries the element `d` in a temporary and
/// compares it against each child, which the swaps below reproduce (the element being sifted is
/// always at `i - 1`).
fn down_heap<T>(
    keys: &mut [T],
    mut i: usize,
    n: usize,
    comparer: &mut impl FnMut(&T, &T) -> Ordering,
) {
    while i <= n >> 1 {
        let mut child = 2 * i;
        if child < n && comparer(&keys[child - 1], &keys[child]) == Ordering::Less {
            child += 1;
        }

        if comparer(&keys[i - 1], &keys[child - 1]) != Ordering::Less {
            break;
        }

        keys.swap(i - 1, child - 1);
        i = child;
    }
}

fn insertion_sort<T>(keys: &mut [T], comparer: &mut impl FnMut(&T, &T) -> Ordering) {
    for i in 0..keys.len().saturating_sub(1) {
        // The element being inserted moves left one place per swap, so it is always at j + 1.
        let mut j = i as isize;
        while j >= 0 && comparer(&keys[j as usize + 1], &keys[j as usize]) == Ordering::Less {
            keys.swap(j as usize, j as usize + 1);
            j -= 1;
        }
    }
}
