//! The default BVH builder: the items' centers are Morton-coded over their joint bounds and
//! radix-sorted into a tree; trees of up to 10,000 items are then rebuilt from their leaves,
//! re-sorted by Morton code, cut into up to 32 bins, each bin built by the surface-area
//! heuristic and the bins joined the same way.
//!
//! A range whose items all share a code cannot be split by it, so a large one is cut in half
//! instead: splitting one item off at a time would recurse once per item, and the resulting
//! chain would overflow the stack of anything that builds, walks or drops it.
//!
//! The build is sequential: the Morton coding, subtree builds and bin builds are independent
//! tasks that each write their own slot, so running them in parallel would give the same tree.
//! The leaves are sorted with [`introspective_sort`], an unstable sort with a fixed tie order,
//! so tied Morton codes always land in the same order and the tree shape is deterministic. The
//! joint bounds are a fresh value, so building never changes an item's own box.

use solidmesh_math::bvh::morton_codes;
use solidmesh_math::fixed_tie_sort::introspective_sort;
use solidmesh_math::{AxisAlignedBoundingBox, Matrix4X4, Vector3};

use super::{BoundingVolumeHierarchy, TraceableRef};
use std::sync::Arc;

const MIN_NODES_FOR_PARALLEL: usize = 1000;
/// Maximum number of nodes to consider for SAH optimization.
const MAX_NODES_TO_OPTIMIZE: usize = 10000;
const ENABLE_TREE_OPTIMIZATION: bool = true;
/// Maximum stack depth for non-recursive processing.
const MAX_STACK_DEPTH: usize = 64;

type MortonNode = (TraceableRef, i64);

/// The builder.
pub struct BvhBuilderLocallyOrderedClustering;

fn node(left: TraceableRef, right: TraceableRef) -> TraceableRef {
    Arc::new(BoundingVolumeHierarchy::new(left, right))
}

impl BvhBuilderLocallyOrderedClustering {
    /// The tree over `source_nodes`: `None` when empty, the item itself when alone.
    pub fn create(source_nodes: &[TraceableRef]) -> Option<TraceableRef> {
        if source_nodes.is_empty() {
            return None;
        }
        if source_nodes.len() == 1 {
            return Some(source_nodes[0].clone());
        }

        let mut nodes = morton_coded(source_nodes);
        radix_sort(&mut nodes);

        let mut root_node = build_tree_parallel(&nodes, 0, nodes.len() - 1);

        // Apply optimization pass if enabled and the tree is not too large
        if ENABLE_TREE_OPTIMIZATION && source_nodes.len() <= MAX_NODES_TO_OPTIMIZE {
            root_node = optimize_tree(root_node);
        }

        Some(root_node)
    }
}

/// Each node with the Morton code of its center over the nodes' joint bounds.
fn morton_coded(source_nodes: &[TraceableRef]) -> Vec<MortonNode> {
    let bounds = calculate_bounds(source_nodes);
    let scale = morton_codes::center_scale(bounds.size());
    let centers_to_morton_space = calculate_morton_transform(&bounds, scale);
    source_nodes
        .iter()
        .map(|source| {
            let code = calculate_morton_code(source.get_center(), centers_to_morton_space);
            (source.clone(), code)
        })
        .collect()
}

/// A stable LSD radix sort on the 64-bit codes, 8 bits a pass.
fn radix_sort(a: &mut Vec<MortonNode>) {
    const R: i32 = 8;
    const MASK: i64 = (1 << R) - 1;
    let mut count = [0usize; 1 << R];
    for shift in (0..64).step_by(R as usize) {
        count.fill(0);
        for item in a.iter() {
            count[((item.1 >> shift) & MASK) as usize] += 1;
        }

        let mut pref = [0usize; 1 << R];
        for i in 1..pref.len() {
            pref[i] = pref[i - 1] + count[i - 1];
        }

        let mut t: Vec<Option<MortonNode>> = vec![None; a.len()];
        for item in a.iter() {
            let index = ((item.1 >> shift) & MASK) as usize;
            t[pref[index]] = Some(item.clone());
            pref[index] += 1;
        }

        *a = t.into_iter().flatten().collect();
    }
}

fn build_tree_parallel(nodes: &[MortonNode], start: usize, end: usize) -> TraceableRef {
    if start == end {
        return nodes[start].0.clone();
    }

    if end - start < MIN_NODES_FOR_PARALLEL {
        return build_tree_recursive(nodes, start, end);
    }

    // The range is sorted, so equal ends mean every code in it is the same.
    let split = if nodes[start].1 == nodes[end].1 {
        start + (end - start) / 2
    } else {
        find_best_split(nodes, start, end)
    };
    // The two halves are independent builds.
    let left = build_tree_parallel(nodes, start, split);
    let right = build_tree_parallel(nodes, split + 1, end);
    node(left, right)
}

fn build_tree_recursive(nodes: &[MortonNode], start: usize, end: usize) -> TraceableRef {
    // Non-recursive implementation to avoid stack overflow on deep trees
    if end - start > MAX_STACK_DEPTH {
        return build_balanced_tree_from_range(nodes, start, end);
    }

    if start == end {
        return nodes[start].0.clone();
    }

    let split = find_best_split(nodes, start, end);
    let left = build_tree_recursive(nodes, start, split);
    let right = build_tree_recursive(nodes, split + 1, end);
    node(left, right)
}

/// A balanced tree straight from the sorted range.
fn build_balanced_tree_from_range(nodes: &[MortonNode], start: usize, end: usize) -> TraceableRef {
    if start == end {
        return nodes[start].0.clone();
    }
    if start + 1 == end {
        return node(nodes[start].0.clone(), nodes[end].0.clone());
    }

    let mid = (start + end) / 2;
    let left = build_balanced_tree_from_range(nodes, start, mid);
    let right = build_balanced_tree_from_range(nodes, mid + 1, end);
    node(left, right)
}

/// The leading zeros of `a ^ b`, less 64. Prefixes are only compared with each other, so the
/// offset does not change any split.
fn common_prefix(a: i64, b: i64) -> i32 {
    ((a ^ b) as u64).leading_zeros() as i32 - 64
}

fn find_best_split(nodes: &[MortonNode], start: usize, end: usize) -> usize {
    if end - start <= 1 {
        return start;
    }

    let start_code = nodes[start].1;
    let common = common_prefix(start_code, nodes[end].1);

    // Binary search for split point
    let mut split = start;
    let mut step = end - start;
    loop {
        step = (step + 1) >> 1;
        let new_split = split + step;
        if new_split < end && common_prefix(start_code, nodes[new_split].1) > common {
            split = new_split;
        }

        if step <= 1 {
            break;
        }
    }

    split
}

fn optimize_tree(root_node: TraceableRef) -> TraceableRef {
    // Only optimize BVH nodes
    if root_node
        .as_any()
        .downcast_ref::<BoundingVolumeHierarchy>()
        .is_none()
    {
        return root_node;
    }

    let mut leaf_nodes = Vec::with_capacity(1024);
    collect_leaf_nodes(&root_node, &mut leaf_nodes);
    if leaf_nodes.len() <= 1 {
        return root_node;
    }

    build_optimized_tree(&leaf_nodes)
}

/// Every non-BVH node under `node`, left to right (a hierarchy given as an item is flattened
/// too).
fn collect_leaf_nodes(node: &TraceableRef, leaf_nodes: &mut Vec<TraceableRef>) {
    let mut stack = vec![node.clone()];
    while let Some(current) = stack.pop() {
        if let Some(bvh) = current.as_any().downcast_ref::<BoundingVolumeHierarchy>() {
            stack.push(bvh.right().clone());
            stack.push(bvh.left().clone());
        } else {
            leaf_nodes.push(current);
        }
    }
}

fn build_optimized_tree(leaf_nodes: &[TraceableRef]) -> TraceableRef {
    // For small numbers of nodes, just use bottom-up SAH clustering
    if leaf_nodes.len() <= 4 {
        return build_sah_tree(leaf_nodes);
    }

    // For larger trees, partition into spatial bins with Morton codes, then apply SAH
    // optimization to each bin.
    let leaf_count = leaf_nodes.len();
    let mut nodes = morton_coded(leaf_nodes);

    // Sort by Morton code (unstable, with a fixed tie order)
    introspective_sort(&mut nodes, &mut |a, b| a.1.cmp(&b.1));

    // Determine optimal bin count based on node count (more bins for more nodes)
    let bin_count = 32.min(4.max(leaf_count / 256));
    let nodes_per_bin = 1.max(leaf_count.div_ceil(bin_count));

    let mut optimized_bins = Vec::with_capacity(bin_count);
    for i in 0..bin_count {
        let start = i * nodes_per_bin;
        let end = (start + nodes_per_bin - 1).min(leaf_count - 1);
        if start <= end {
            let bin_nodes: Vec<TraceableRef> =
                nodes[start..=end].iter().map(|n| n.0.clone()).collect();
            optimized_bins.push(build_sah_tree(&bin_nodes));
        }
    }

    // Apply SAH to combine the bins
    build_sah_tree(&optimized_bins)
}

/// The surface-area-heuristic tree over `nodes` in their order: each level splits where
/// `left_count * left_area + right_count * right_area` is least (the first such split).
fn build_sah_tree(nodes: &[TraceableRef]) -> TraceableRef {
    let node_count = nodes.len();
    if node_count == 1 {
        return nodes[0].clone();
    }
    if node_count == 2 {
        return node(nodes[0].clone(), nodes[1].clone());
    }

    let all_bounds: Vec<AxisAlignedBoundingBox> = nodes
        .iter()
        .map(|n| n.get_axis_aligned_bounding_box())
        .collect();

    // Precompute prefix and suffix bounds
    let mut left_bounds = vec![AxisAlignedBoundingBox::default(); node_count - 1];
    let mut right_bounds = vec![AxisAlignedBoundingBox::default(); node_count - 1];
    left_bounds[0] = all_bounds[0];
    for i in 1..node_count - 1 {
        left_bounds[i] = left_bounds[i - 1] + all_bounds[i];
    }

    right_bounds[node_count - 2] = all_bounds[node_count - 1];
    for i in (0..node_count.saturating_sub(2)).rev() {
        right_bounds[i] = right_bounds[i + 1] + all_bounds[i + 1];
    }

    let mut best_cost = f64::MAX;
    let mut best_split = 0;
    for split_index in 0..node_count - 1 {
        let left_count = split_index + 1;
        let right_count = node_count - left_count;
        let left_sa = left_bounds[split_index].get_surface_area();
        let right_sa = right_bounds[split_index].get_surface_area();
        let cost = left_count as f64 * left_sa + right_count as f64 * right_sa;
        if cost < best_cost {
            best_cost = cost;
            best_split = split_index;
        }
    }

    let left = build_sah_tree(&nodes[..=best_split]);
    let right = build_sah_tree(&nodes[best_split + 1..]);
    node(left, right)
}

/// The joint bounds of the nodes.
fn calculate_bounds(nodes: &[TraceableRef]) -> AxisAlignedBoundingBox {
    let mut bounds = nodes[0].get_axis_aligned_bounding_box();
    for item in &nodes[1..] {
        bounds.expand_to_include(&item.get_axis_aligned_bounding_box());
    }

    bounds
}

fn calculate_morton_transform(bounds: &AxisAlignedBoundingBox, scale: Vector3) -> Matrix4X4 {
    let translation = Matrix4X4::create_translation_vector(-bounds.min_xyz);
    let scale_matrix = Matrix4X4::create_scale_vector(morton_codes::SIZE as f64 / scale);
    Matrix4X4::mult(translation, scale_matrix)
}

fn calculate_morton_code(center: Vector3, transform: Matrix4X4) -> i64 {
    let morton_space = center.transform(transform);
    morton_codes::encode3_position(morton_space)
}
