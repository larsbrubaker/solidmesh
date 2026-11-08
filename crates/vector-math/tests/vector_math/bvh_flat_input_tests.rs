// The flat tree over boxes with an axis of no extent, or all at one center. The builder recurses
// once per level, so a chain over many boxes would overflow the stack of whoever builds it; these
// run on the ordinary test-thread stack and bound the depth.

use solidmesh_math::bvh::{BvhBuilderLocallyOrderedClustering, BvhNode};
use solidmesh_math::{AxisAlignedBoundingBox, Vector3};

fn box_at(center: Vector3) -> AxisAlignedBoundingBox {
    AxisAlignedBoundingBox::centered_box(Vector3::new(0.5, 0.5, 0.0), center)
}

/// The depth of the flat tree from its root, walked without recursing.
fn depth(nodes: &[BvhNode]) -> usize {
    let mut deepest = 0;
    let mut stack = vec![(0usize, 1usize)];
    while let Some((index, level)) = stack.pop() {
        deepest = deepest.max(level);
        let node = nodes[index];
        if !node.is_leaf() {
            stack.push((node.left as usize, level + 1));
            stack.push((node.right as usize, level + 1));
        }
    }
    deepest
}

fn assert_shallow(boxes: &[AxisAlignedBoundingBox], what: &str) {
    let nodes = BvhBuilderLocallyOrderedClustering::build_bvh(boxes);
    let level = depth(&nodes);
    let bound = 64 + 2 * (usize::BITS - boxes.len().leading_zeros()) as usize;
    assert!(
        level <= bound,
        "{what}: depth {level} over {} boxes",
        boxes.len()
    );
}

fn flat_grid(n: i32, step: f64) -> Vec<AxisAlignedBoundingBox> {
    let mut boxes = Vec::new();
    for j in 0..n {
        for i in 0..n {
            boxes.push(box_at(Vector3::new(
                f64::from(i) * step,
                f64::from(j) * step,
                0.0,
            )));
        }
    }
    boxes
}

#[test]
fn a_flat_grid_of_boxes_gets_a_shallow_tree() {
    for n in [60, 200] {
        assert_shallow(&flat_grid(n, 1.0), "integer grid");
    }
    assert_shallow(&flat_grid(150, 0.37), "fine plate");
}

#[test]
fn boxes_at_one_center_get_a_shallow_tree() {
    let boxes = vec![box_at(Vector3::new(1.0, 2.0, 0.0)); 50_000];
    assert_shallow(&boxes, "one center");
}

#[test]
fn boxes_with_extent_on_every_axis_keep_their_exact_tree() {
    // Pinned from the builder as it was before flat input was fixed: nothing but a degenerate
    // input may change a tree.
    let boxes: Vec<AxisAlignedBoundingBox> = (0..3000)
        .map(|i| {
            let t = f64::from(i) * 0.137;
            AxisAlignedBoundingBox::centered_box(
                Vector3::new(1.0, 1.0, 1.0),
                Vector3::new(t.sin() * 40.0, t.cos() * 55.0, (t * 0.3).sin() * 9.0),
            )
        })
        .collect();
    let nodes = BvhBuilderLocallyOrderedClustering::build_bvh(&boxes);
    let hash = nodes.iter().fold(0xcbf2_9ce4_8422_2325_u64, |hash, node| {
        let word = (u64::from(node.left as u32) << 32) | u64::from(node.right as u32);
        (hash ^ word).wrapping_mul(0x100_0000_01b3)
    });
    assert_eq!(hash, PINNED_TREE, "the tree over ordinary boxes changed");
}

const PINNED_TREE: u64 = 4968738463416079177;
