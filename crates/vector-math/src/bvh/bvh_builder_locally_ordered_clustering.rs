//! A flat BVH from Morton-ordered box centers: the boxes' centers are mapped into a 2^21 grid
//! over their joint bounds, sorted by Morton code, and split recursively where the codes'
//! common prefix changes.
//!
//! Each subtree writes its own disjoint index range, so building large subtrees in parallel
//! would give the same result as the recursive build that runs here (see [`crate::parallel`]).
//! Box 0's Morton code is taken from the joint bounds' center rather than its own, which keeps
//! the node array identical to the established output; the caller's boxes are read only, never
//! changed, since a tree build that edits its input is a latent bug.

use super::morton_codes;
use crate::fixed_tie_sort::introspective_sort;
use crate::{AxisAlignedBoundingBox, Matrix4X4, Vector3};

/// A node of the flat tree: a leaf when `left == right` (both the box index), else the indices
/// of its two child nodes.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Hash)]
pub struct BvhNode {
    pub left: i32,
    pub right: i32,
}

impl BvhNode {
    pub fn is_leaf(&self) -> bool {
        self.left == self.right
    }
}

#[derive(Debug, Clone, Copy)]
struct NodeInfo {
    morton_code: i64,
    original_index: i32,
}

/// The builder.
pub struct BvhBuilderLocallyOrderedClustering;

impl BvhBuilderLocallyOrderedClustering {
    /// The flat tree over `aabbs`: node 0 is the root, `2 * n - 1` nodes in all.
    pub fn build_bvh(aabbs: &[AxisAlignedBoundingBox]) -> Vec<BvhNode> {
        if aabbs.is_empty() {
            return Vec::new();
        }

        if aabbs.len() == 1 {
            return vec![BvhNode { left: 0, right: 0 }];
        }

        let bounds = calculate_bounds(aabbs);
        let scale = Vector3::max(bounds.size(), Vector3::ONE) * 1.001;
        let centers_to_morton_space = calculate_morton_transform(&bounds, scale);

        let mut node_infos: Vec<NodeInfo> = (0..aabbs.len())
            .map(|i| {
                // Box 0's code comes from the joint bounds' center (see the header).
                let center = if i == 0 {
                    bounds.center()
                } else {
                    aabbs[i].center()
                };
                NodeInfo {
                    morton_code: calculate_morton_code(center, centers_to_morton_space),
                    original_index: i as i32,
                }
            })
            .collect();

        introspective_sort(&mut node_infos, &mut |a, b| {
            a.morton_code.cmp(&b.morton_code)
        });

        let mut nodes = vec![BvhNode::default(); aabbs.len() * 2 - 1];
        let end = node_infos.len() - 1;
        build_tree_recursive(&node_infos, 0, end, &mut nodes, 0);

        nodes
    }
}

fn build_tree_recursive(
    node_infos: &[NodeInfo],
    start: usize,
    end: usize,
    nodes: &mut [BvhNode],
    node_index: usize,
) -> usize {
    if start == end {
        nodes[node_index] = BvhNode {
            left: node_infos[start].original_index,
            right: node_infos[start].original_index,
        };
        return node_index;
    }

    let split = find_best_split(node_infos, start, end);

    let left_child = build_tree_recursive(node_infos, start, split, nodes, node_index + 1);
    let right_child = build_tree_recursive(
        node_infos,
        split + 1,
        end,
        nodes,
        node_index + 2 * (split - start + 1),
    );

    nodes[node_index] = BvhNode {
        left: left_child as i32,
        right: right_child as i32,
    };

    node_index
}

/// The leading zero count of `a ^ b` as 64 bits, minus 64.
fn common_prefix(a: i64, b: i64) -> i32 {
    ((a ^ b) as u64).leading_zeros() as i32 - 64
}

fn find_best_split(node_infos: &[NodeInfo], start: usize, end: usize) -> usize {
    if end - start <= 1 {
        return start;
    }

    let common = common_prefix(node_infos[start].morton_code, node_infos[end].morton_code);

    let mut split = start;
    let mut step = end - start;
    loop {
        step = (step + 1) >> 1;
        let new_split = split + step;
        if new_split < end {
            let split_prefix = common_prefix(
                node_infos[start].morton_code,
                node_infos[new_split].morton_code,
            );
            if split_prefix > common {
                split = new_split;
            }
        }

        if step <= 1 {
            break;
        }
    }

    split
}

fn calculate_bounds(aabbs: &[AxisAlignedBoundingBox]) -> AxisAlignedBoundingBox {
    let mut bounds = aabbs[0];
    for aabb in &aabbs[1..] {
        bounds.expand_to_include(aabb);
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
