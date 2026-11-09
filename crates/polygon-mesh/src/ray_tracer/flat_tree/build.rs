//! Collapses the crate's binary BVH into a flat tree's buffers. The binary tree is first copied
//! into an arena (a walk with an explicit stack, parents before children) so that its bounds can
//! be recomputed bottom-up in `f32` from the source triangles' `f64` corners and so collapsing
//! never recurses: a flat or tied-centroid mesh can make long chains of splits, and nothing here
//! may spend call stack per level of one.
//!
//! Collapsing grows a node's four slots from the binary node's two children by repeatedly
//! opening the widest expandable child (largest surface area; among equal areas the earliest
//! slot), so the slot a ray is most likely to enter is the one split further. A subtree of at
//! most [`MAX_LEAF_TRIANGLES`] triangles becomes a leaf run. A subtree the builder left as a
//! chain, taller than [`DEGENERATE_HEIGHT`], is first replaced by a tree split by count; the
//! balanced parts of the tree around it are left as built (`rebalance_degenerate_subtrees`).
//! Everything is written in a fixed order, so the same triangles give the same bytes.

use std::sync::Arc;

use solidmesh_math::Vector3;

use super::layout::{child_code, FlatNode, FlatTriangle, TRIANGLE_STRIDE};
use super::{
    bounds_max_f32, bounds_min_f32, FlatTree, FlatTreeBuffer, FlatTreeError, COORDINATE_LIMIT,
    MAX_LEAF_TRIANGLES, MAX_TREE_DEPTH, MAX_TRIANGLES, NODE_STRIDE,
};
use crate::ray_tracer::{
    BoundingVolumeHierarchy, BvhCreationOptions, Traceable, TraceableRef, TriangleShape,
};

/// Marks a missing child in the arena.
const NONE: u32 = u32::MAX;

/// A binary subtree taller than this is a chain the builder could not separate: items with tied
/// bounds or centers leave its surface-area and Morton splits nothing to choose between, and an
/// unbroken chain collapses to about one flat level per three triangles, deeper than
/// [`MAX_TREE_DEPTH`] allows. A Morton tree of real geometry stays well under this, and halving
/// a run of items that cannot be told apart spatially costs nothing, so such a subtree is split
/// by count instead, keeping the builder's left-to-right order.
const DEGENERATE_HEIGHT: u32 = 48;

/// A short sibling holding at most one part in this of its parent's triangles is a side branch of
/// a chain rather than a subtree worth keeping apart from it.
const SIDE_BRANCH_DENOMINATOR: u64 = 8;

/// A binary node, copied out of the BVH.
struct ArenaNode {
    left: u32,
    right: u32,
    /// The source triangle of a leaf, `NONE` otherwise.
    source: u32,
    min: [f32; 3],
    max: [f32; 3],
    triangle_count: u32,
}

impl ArenaNode {
    fn is_leaf(&self) -> bool {
        self.left == NONE
    }

    /// The bounds' surface area, the expansion priority.
    fn area(&self) -> f64 {
        let dx = f64::from(self.max[0]) - f64::from(self.min[0]);
        let dy = f64::from(self.max[1]) - f64::from(self.min[1]);
        let dz = f64::from(self.max[2]) - f64::from(self.min[2]);
        2.0 * (dx * dy + dy * dz + dz * dx)
    }

    /// True when this child should become a node of its own, which also makes it openable.
    fn is_interior(&self) -> bool {
        !self.is_leaf() && self.triangle_count > MAX_LEAF_TRIANGLES
    }
}

pub(super) fn build(
    triangles: &[[Vector3; 3]],
    max_storage_binding_size: u64,
) -> Result<FlatTree, FlatTreeError> {
    if triangles.is_empty() {
        return Err(FlatTreeError::NoTriangles);
    }
    let triangle_count = triangles.len() as u64;
    if triangle_count > MAX_TRIANGLES {
        return Err(FlatTreeError::TooManyTriangles {
            count: triangle_count,
            limit: MAX_TRIANGLES,
        });
    }
    for (triangle_index, corners) in triangles.iter().enumerate() {
        let valid = corners.iter().all(|corner| {
            (0..3).all(|axis| corner[axis].is_finite() && corner[axis].abs() <= COORDINATE_LIMIT)
        });
        if !valid {
            return Err(FlatTreeError::InvalidCoordinate {
                triangle_index: triangle_index as u64,
            });
        }
    }

    // A triangle-buffer refusal needs no tree: that buffer's size is known up front.
    let triangle_bytes = triangle_count * TRIANGLE_STRIDE as u64;
    if triangle_bytes > max_storage_binding_size {
        return Err(FlatTreeError::BufferTooLarge {
            buffer: FlatTreeBuffer::Triangles,
            needed_bytes: triangle_bytes,
            limit_bytes: max_storage_binding_size,
            triangle_count,
            triangles_that_fit: max_storage_binding_size / TRIANGLE_STRIDE as u64,
        });
    }

    let arena = copy_into_arena(triangles)?;
    let (nodes, flat_triangles, depth) = collapse(&arena, triangles);

    let node_bytes = nodes.len() as u64 * NODE_STRIDE as u64;
    if node_bytes > max_storage_binding_size {
        return Err(FlatTreeError::BufferTooLarge {
            buffer: FlatTreeBuffer::Nodes,
            needed_bytes: node_bytes,
            limit_bytes: max_storage_binding_size,
            triangle_count,
            triangles_that_fit: triangles_fitting_in_both(
                triangle_count,
                node_bytes,
                max_storage_binding_size,
            ),
        });
    }
    if depth > MAX_TREE_DEPTH {
        return Err(FlatTreeError::TreeTooDeep {
            depth,
            limit: MAX_TREE_DEPTH,
        });
    }

    Ok(FlatTree {
        nodes,
        triangles: flat_triangles,
        depth,
    })
}

/// How many triangles fit when this tree's node bytes per triangle are taken as typical: scaled
/// down from this scene, and capped by the triangle buffer.
fn triangles_fitting_in_both(triangle_count: u64, node_bytes: u64, limit: u64) -> u64 {
    // u128: triangle_count * limit does not fit in u64 for the largest limits.
    let by_nodes = (u128::from(triangle_count) * u128::from(limit) / u128::from(node_bytes)) as u64;
    by_nodes.min(limit / TRIANGLE_STRIDE as u64)
}

/// The BVH over `triangles`, copied into an arena with parents before children and bounds
/// computed bottom-up from the source corners.
fn copy_into_arena(triangles: &[[Vector3; 3]]) -> Result<Vec<ArenaNode>, FlatTreeError> {
    let items: Vec<TraceableRef> = triangles
        .iter()
        .enumerate()
        .map(|(index, corners)| {
            // `index` fits i32: MAX_TRIANGLES is below i32::MAX.
            Arc::new(TriangleShape::new(
                corners[0],
                corners[1],
                corners[2],
                index as i32,
            )) as TraceableRef
        })
        .collect();
    let root = BoundingVolumeHierarchy::create_new_hierarchy(items, BvhCreationOptions::default())
        .ok_or(FlatTreeError::NoTriangles)?;

    let mut arena: Vec<ArenaNode> = Vec::with_capacity(triangles.len() * 2);
    // (node, parent's arena index, whether it is the parent's right child)
    let mut stack: Vec<(&dyn Traceable, u32, bool)> = vec![(root.as_ref(), NONE, false)];
    while let Some((node, parent, is_right)) = stack.pop() {
        let index = arena.len() as u32;
        if let Some(interior) = node.as_any().downcast_ref::<BoundingVolumeHierarchy>() {
            arena.push(ArenaNode {
                left: NONE,
                right: NONE,
                source: NONE,
                min: [0.0; 3],
                max: [0.0; 3],
                triangle_count: 0,
            });
            stack.push((interior.right().as_ref(), index, true));
            stack.push((interior.left().as_ref(), index, false));
        } else {
            let leaf = node
                .as_any()
                .downcast_ref::<TriangleShape>()
                .ok_or(FlatTreeError::UnexpectedBvhLeaf)?;
            let source = leaf.index as u32;
            let corners = triangles
                .get(source as usize)
                .ok_or(FlatTreeError::UnexpectedBvhLeaf)?;
            let (min, max) = leaf_bounds(corners);
            arena.push(ArenaNode {
                left: NONE,
                right: NONE,
                source,
                min,
                max,
                triangle_count: 1,
            });
        }
        if parent != NONE {
            if is_right {
                arena[parent as usize].right = index;
            } else {
                arena[parent as usize].left = index;
            }
        }
    }

    // Parents come before children, so a reverse pass sees children first.
    for index in (0..arena.len()).rev() {
        if arena[index].is_leaf() {
            continue;
        }
        let (left, right) = (
            &arena[arena[index].left as usize],
            &arena[arena[index].right as usize],
        );
        let mut min = left.min;
        let mut max = left.max;
        for axis in 0..3 {
            min[axis] = min[axis].min(right.min[axis]);
            max[axis] = max[axis].max(right.max[axis]);
        }
        let count = left.triangle_count + right.triangle_count;
        let node = &mut arena[index];
        node.min = min;
        node.max = max;
        node.triangle_count = count;
    }
    rebalance_degenerate_subtrees(&mut arena);
    Ok(arena)
}

/// Splits only what is actually degenerate. Walking down from the root through subtrees taller
/// than [`DEGENERATE_HEIGHT`], it passes through a node with a substantial short child (the
/// child stays as the builder made it) and splits a node by count when nothing shorter below it
/// could be split instead: its only tall child is nearly all of it (an unbroken chain, whose side
/// branches are at most one part in [`SIDE_BRANCH_DENOMINATOR`]), or no child is tall. A tree too
/// tall even after that, a spine with substantial side branches, is split by count whole, since
/// the depth bound comes first.
fn rebalance_degenerate_subtrees(arena: &mut Vec<ArenaNode>) {
    let heights = subtree_heights(arena);
    if heights[0] <= DEGENERATE_HEIGHT {
        return;
    }
    let mut stack = vec![0u32];
    while let Some(index) = stack.pop() {
        let node = &arena[index as usize];
        if node.is_leaf() || heights[index as usize] <= DEGENERATE_HEIGHT {
            continue;
        }
        let (left, right) = (node.left, node.right);
        let left_tall = heights[left as usize] > DEGENERATE_HEIGHT;
        let right_tall = heights[right as usize] > DEGENERATE_HEIGHT;
        if left_tall && right_tall {
            stack.push(right);
            stack.push(left);
            continue;
        }
        if left_tall || right_tall {
            let (tall, short) = if left_tall {
                (left, right)
            } else {
                (right, left)
            };
            let side_triangles = u64::from(arena[short as usize].triangle_count);
            if side_triangles * SIDE_BRANCH_DENOMINATOR > u64::from(node.triangle_count) {
                stack.push(tall);
                continue;
            }
        }
        rebalance_by_count(arena, index);
    }
    if subtree_heights(arena)[0] > DEGENERATE_HEIGHT {
        rebalance_by_count(arena, 0);
    }
}

/// Each reachable node's height, leaves being 1. A post-order walk with an explicit stack, since
/// a split subtree's new nodes point back at older leaf indices and the arena stops being
/// parents-before-children.
fn subtree_heights(arena: &[ArenaNode]) -> Vec<u32> {
    let mut heights = vec![1u32; arena.len()];
    let mut stack: Vec<(u32, bool)> = vec![(0, false)];
    while let Some((index, children_done)) = stack.pop() {
        let node = &arena[index as usize];
        if node.is_leaf() {
            continue;
        }
        let (left, right) = (node.left, node.right);
        if children_done {
            heights[index as usize] = 1 + heights[left as usize].max(heights[right as usize]);
        } else {
            stack.push((index, true));
            stack.push((right, false));
            stack.push((left, false));
        }
    }
    heights
}

/// Splits the leaves under `root`, kept in their left-to-right order (which is the builder's
/// spatial order), into a tree of equal-count halves, and recomputes the new nodes' bounds.
fn rebalance_by_count(arena: &mut Vec<ArenaNode>, root: u32) {
    let mut leaves = Vec::new();
    let mut walk = vec![root];
    while let Some(index) = walk.pop() {
        let node = &arena[index as usize];
        if node.is_leaf() {
            leaves.push(index);
        } else {
            walk.push(node.right);
            walk.push(node.left);
        }
    }

    let first_new = arena.len();
    let mut stack: Vec<(u32, usize, usize)> = vec![(root, 0, leaves.len())];
    while let Some((node, start, length)) = stack.pop() {
        let left_length = length / 2;
        let left = subtree_for_range(arena, &leaves, start, left_length, &mut stack);
        let right = subtree_for_range(
            arena,
            &leaves,
            start + left_length,
            length - left_length,
            &mut stack,
        );
        arena[node as usize].left = left;
        arena[node as usize].right = right;
    }

    // New nodes come after their parents, so a reverse pass sees children first.
    for index in (first_new..arena.len()).rev() {
        let (left, right) = (
            &arena[arena[index].left as usize],
            &arena[arena[index].right as usize],
        );
        let mut min = left.min;
        let mut max = left.max;
        for axis in 0..3 {
            min[axis] = min[axis].min(right.min[axis]);
            max[axis] = max[axis].max(right.max[axis]);
        }
        let count = left.triangle_count + right.triangle_count;
        let node = &mut arena[index];
        node.min = min;
        node.max = max;
        node.triangle_count = count;
    }
}

/// The arena index for a range of leaves: the leaf itself for one, a new node queued for more.
fn subtree_for_range(
    arena: &mut Vec<ArenaNode>,
    leaves: &[u32],
    start: usize,
    length: usize,
    stack: &mut Vec<(u32, usize, usize)>,
) -> u32 {
    if length == 1 {
        return leaves[start];
    }
    let index = arena.len() as u32;
    arena.push(ArenaNode {
        left: NONE,
        right: NONE,
        source: NONE,
        min: [0.0; 3],
        max: [0.0; 3],
        triangle_count: 0,
    });
    stack.push((index, start, length));
    index
}

/// A triangle's stored bounds from its exact corners.
fn leaf_bounds(corners: &[Vector3; 3]) -> ([f32; 3], [f32; 3]) {
    let mut min = [f32::MAX; 3];
    let mut max = [f32::MIN; 3];
    for corner in corners {
        for axis in 0..3 {
            min[axis] = min[axis].min(bounds_min_f32(corner[axis]));
            max[axis] = max[axis].max(bounds_max_f32(corner[axis]));
        }
    }
    (min, max)
}

/// The arena collapsed into node and triangle records, and the tree's depth.
fn collapse(
    arena: &[ArenaNode],
    triangles: &[[Vector3; 3]],
) -> (Vec<FlatNode>, Vec<FlatTriangle>, u32) {
    let mut nodes = vec![FlatNode::EMPTY];
    let mut flat_triangles = Vec::with_capacity(triangles.len());
    let mut deepest = 1;
    // (arena node, flat node index, depth)
    let mut pending: Vec<(u32, u32, u32)> = vec![(0, 0, 1)];
    let mut run_stack: Vec<u32> = Vec::new();

    while let Some((arena_index, flat_index, depth)) = pending.pop() {
        deepest = deepest.max(depth);
        let slots = opened_slots(arena, arena_index);
        let mut node = FlatNode::EMPTY;
        for (slot, &slot_arena) in slots.iter().enumerate() {
            if slot_arena == NONE {
                continue;
            }
            let child = &arena[slot_arena as usize];
            node.min_x[slot] = child.min[0];
            node.max_x[slot] = child.max[0];
            node.min_y[slot] = child.min[1];
            node.max_y[slot] = child.max[1];
            node.min_z[slot] = child.min[2];
            node.max_z[slot] = child.max[2];
            node.child[slot] = if child.is_interior() {
                let child_index = nodes.len() as u32;
                nodes.push(FlatNode::EMPTY);
                pending.push((slot_arena, child_index, depth + 1));
                child_index
            } else {
                let first_triangle = flat_triangles.len() as u32;
                append_triangles(
                    arena,
                    slot_arena,
                    triangles,
                    &mut flat_triangles,
                    &mut run_stack,
                );
                child_code::leaf(first_triangle, child.triangle_count)
            };
        }
        nodes[flat_index as usize] = node;
    }

    (nodes, flat_triangles, deepest)
}

/// The arena nodes a flat node holds: the binary node's children, split further while a slot is
/// free and a child can be opened; `NONE` for unused slots.
fn opened_slots(arena: &[ArenaNode], arena_index: u32) -> [u32; 4] {
    let mut slots = [NONE; 4];
    let root = &arena[arena_index as usize];
    if !root.is_interior() {
        // A whole tree of at most one leaf run: only the first flat node can get here.
        slots[0] = arena_index;
        return slots;
    }
    slots[0] = root.left;
    slots[1] = root.right;
    let mut used = 2;
    while used < slots.len() {
        let mut widest = None;
        let mut widest_area = f64::NEG_INFINITY;
        for (position, &slot) in slots[..used].iter().enumerate() {
            let child = &arena[slot as usize];
            // `>` so that of equal areas the earliest slot opens.
            if child.is_interior() && child.area() > widest_area {
                widest_area = child.area();
                widest = Some(position);
            }
        }
        let Some(position) = widest else {
            break;
        };
        let opened = &arena[slots[position] as usize];
        let (left, right) = (opened.left, opened.right);
        slots[position] = left;
        slots[used] = right;
        used += 1;
    }
    slots
}

/// Appends the triangles under `start`, left to right, walking the arena with `run_stack`.
fn append_triangles(
    arena: &[ArenaNode],
    start: u32,
    triangles: &[[Vector3; 3]],
    flat_triangles: &mut Vec<FlatTriangle>,
    run_stack: &mut Vec<u32>,
) {
    run_stack.clear();
    run_stack.push(start);
    while let Some(index) = run_stack.pop() {
        let node = &arena[index as usize];
        if node.is_leaf() {
            flat_triangles.push(flat_triangle(&triangles[node.source as usize], node.source));
        } else {
            run_stack.push(node.right);
            run_stack.push(node.left);
        }
    }
}

/// The stored record of a triangle: corners at the nearest `f32`, as the BVH's leaves hold them.
fn flat_triangle(corners: &[Vector3; 3], source_index: u32) -> FlatTriangle {
    let narrow = |corner: Vector3| [corner.x as f32, corner.y as f32, corner.z as f32];
    FlatTriangle {
        v0: narrow(corners[0]),
        v1: narrow(corners[1]),
        v2: narrow(corners[2]),
        source_index,
    }
}
