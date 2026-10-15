// How the BVH builders group items: the surface-area builder sorts by each item's center and
// honours the caller's median-split size at every level; the Morton builder places every box,
// box 0 included, by its own center.

use vector_math::bvh::{
    BvhBuilderLocallyOrderedClustering, BvhNode, BvhTree, BvhTreeItemData, TradeOffBvhConstructor,
};
use vector_math::{AxisAlignedBoundingBox, Vector3};

fn unit_box_at(x: f64, y: f64) -> AxisAlignedBoundingBox {
    AxisAlignedBoundingBox::new(Vector3::new(x, y, 0.0), Vector3::new(x + 1.0, y + 1.0, 1.0))
}

#[test]
fn tree_item_center_is_the_bounds_center() {
    let bounds = unit_box_at(4.0, -2.0);
    assert_eq!(BvhTreeItemData::new(0, bounds).center, bounds.center());
}

#[test]
fn trade_off_tree_separates_two_distant_clusters_at_the_root() {
    // Interleave the two clusters so only sorting by center can separate them.
    let items: Vec<BvhTreeItemData<usize>> = (0..16)
        .map(|i| {
            let x = if i % 2 == 0 { 0.0 } else { 100.0 };
            BvhTreeItemData::new(i, unit_box_at(x, (i / 2) as f64 * 1.5))
        })
        .collect();
    let tree = TradeOffBvhConstructor::create_new_hierarchy(
        items,
        TradeOffBvhConstructor::DEFAULT_MAX_RECURSION,
        0,
        None,
        TradeOffBvhConstructor::DEFAULT_DO_SIMPLE_SORT_SIZE,
    )
    .expect("16 items make a tree");

    let (Some(a), Some(b)) = tree.nodes() else {
        panic!("the root splits the two clusters");
    };
    for side in [a, b] {
        let bounds = side.aabb();
        assert!(
            bounds.max_xyz.x < 50.0 || bounds.min_xyz.x > 50.0,
            "a child spans both clusters: {bounds:?}"
        );
    }
}

#[test]
fn simple_sort_size_applies_below_the_root() {
    // Identical boxes never pay off under the surface-area cost, so only the median split
    // (groups larger than the simple-sort size) divides them.
    let items: Vec<BvhTreeItemData<usize>> = (0..8)
        .map(|i| BvhTreeItemData::new(i, unit_box_at(0.0, 0.0)))
        .collect();
    let tree: BvhTree<usize> = TradeOffBvhConstructor::create_new_hierarchy(
        items,
        TradeOffBvhConstructor::DEFAULT_MAX_RECURSION,
        0,
        None,
        1,
    )
    .expect("8 items make a tree");
    // Median splits all the way down: 8 leaves and 7 branches.
    assert_eq!(tree.count_branches(), 15);
}

fn leaves_under(nodes: &[BvhNode], index: usize, out: &mut Vec<i32>) {
    let node = nodes[index];
    if node.is_leaf() {
        out.push(node.left);
    } else {
        leaves_under(nodes, node.left as usize, out);
        leaves_under(nodes, node.right as usize, out);
    }
}

#[test]
fn morton_tree_places_box_zero_by_its_own_center() {
    // Box 0 sits with box 3 on the far right; the joint bounds' center is on the left half.
    let boxes = [
        unit_box_at(100.0, 0.0),
        unit_box_at(0.0, 0.0),
        unit_box_at(0.5, 0.0),
        unit_box_at(100.5, 0.0),
    ];
    let nodes = BvhBuilderLocallyOrderedClustering::build_bvh(&boxes);
    let root = nodes[0];
    let mut left = Vec::new();
    leaves_under(&nodes, root.left as usize, &mut left);
    let mut right = Vec::new();
    leaves_under(&nodes, root.right as usize, &mut right);
    left.sort();
    right.sort();
    assert_eq!(left, vec![1, 2]);
    assert_eq!(right, vec![0, 3]);
}
