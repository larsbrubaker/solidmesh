//! Port of `Agg.PolygonMesh/MinkowskiProgressModelTests.cs`: the sweep's and the union tree's
//! unit-to-time mappings in `minkowski_progress_model`.
//!
//! Skipped until its code lands:
//! - `APatchedDilationsPhaseTotalMatchesTheModelsLayout`: needs the kernel's
//!   `Manifold::try_dilate_by_convex` and `ProgressReporter::phase_total`, which manifold-rust
//!   does not have yet (manifold-sharp's `ConvexDilation.cs`; an upstream sync step).

use polygon_mesh::csg::minkowski_progress_model::{
    kernel_units, time_fraction, tree_hull_units, tree_time_fraction, tree_units,
    TREE_LEAF_TIME_SHARE, TREE_LEVEL_TIME_SHARE,
};

fn assert_within(actual: f64, expected: f64, tolerance: f64) {
    assert!(
        (actual - expected).abs() <= tolerance,
        "expected {expected} within {tolerance}, got {actual}"
    );
}

#[test]
fn the_time_fraction_rises_through_every_kernel_unit_from_zero_to_one() {
    const TRIANGLES: i32 = 2500;
    let total = kernel_units(TRIANGLES);
    assert_eq!(total, 2500 + 3 + 1);

    let mut previous = -1.0;
    for unit in 0..=total {
        let fraction = time_fraction(TRIANGLES, unit as f64 / total as f64);
        assert!(fraction >= previous);
        previous = fraction;
    }

    assert_eq!(time_fraction(TRIANGLES, 0.0), 0.0);
    assert_eq!(time_fraction(TRIANGLES, 1.0), 1.0);
}

#[test]
fn hulls_are_cheap_and_batch_unions_carry_the_time() {
    const TRIANGLES: i32 = 3000;
    let total = kernel_units(TRIANGLES);

    // Every hull of the first batch done, its union not: the bar has barely moved, because the
    // union that follows is where the batch's time goes.
    let hulls_done = time_fraction(TRIANGLES, 1000.0 / total as f64);
    assert!(hulls_done < 0.05);

    // One of three batches unioned: close to a third of the time, less its share of the merge.
    let one_batch = time_fraction(TRIANGLES, 1001.0 / total as f64);
    assert_within(one_batch, 0.3, 0.001);

    // Every batch unioned, the closing merge still to run: its share is left.
    let all_batches = time_fraction(TRIANGLES, (total - 1) as f64 / total as f64);
    assert_within(all_batches, 0.9, 0.001);
}

#[test]
fn the_dilation_tree_fraction_rises_linearly_through_the_leaves_then_the_tree() {
    const TRIANGLES: i64 = 1600;
    let total = tree_units(TRIANGLES, false);

    // 1600 hulls, 101 leaves (the solid is one), 100 tree nodes and the closing pass.
    assert_eq!(total, 1600 + 101 + 100 + 1);

    let mut previous = -1.0;
    for unit in 0..=total {
        let fraction = tree_time_fraction(TRIANGLES, unit as f64 / total as f64, 10, false);
        assert!(fraction >= previous);
        previous = fraction;
    }

    assert_eq!(tree_time_fraction(TRIANGLES, 0.0, 10, false), 0.0);
    assert_eq!(tree_time_fraction(TRIANGLES, 1.0, 10, false), 1.0);

    // Half the leaf work done is half the leaves' share of the clock.
    let half_leaves = tree_time_fraction(TRIANGLES, (1701.0 / 2.0) / total as f64, 10, false);
    assert_within(half_leaves, TREE_LEAF_TIME_SHARE / 2.0, 1e-9);

    // The bottom level is half the tree's unions but many small ones on every core, so it takes
    // well under half the levels' share; the few big unions at the top get the rest, which is
    // what stops the bar holding at the very end.
    let bottom_level_done =
        tree_time_fraction(TRIANGLES, (1701 + 50) as f64 / total as f64, 10, false);
    let levels_share_after_bottom =
        (bottom_level_done - TREE_LEAF_TIME_SHARE) / TREE_LEVEL_TIME_SHARE;
    assert!(levels_share_after_bottom < 0.3);

    // Everything but the closing pass done: the leaves and levels, and not the last stretch.
    let all_nodes = tree_time_fraction(TRIANGLES, (total - 1) as f64 / total as f64, 10, false);
    assert_within(
        all_nodes,
        TREE_LEAF_TIME_SHARE + TREE_LEVEL_TIME_SHARE,
        1e-9,
    );
}

#[test]
fn the_erosion_tree_counts_its_closing_subtraction_as_a_top_level() {
    const TRIANGLES: i64 = 1600;
    let total = tree_units(TRIANGLES, true);

    // 1600 hulls, 100 leaves (the solid is not one), 99 tree nodes, the subtraction and the
    // closing pass.
    assert_eq!(total, 1600 + 100 + 99 + 1 + 1);

    let mut previous = -1.0;
    for unit in 0..=total {
        let fraction = tree_time_fraction(TRIANGLES, unit as f64 / total as f64, 10, true);
        assert!(fraction >= previous);
        previous = fraction;
    }

    assert_eq!(tree_time_fraction(TRIANGLES, 1.0, 10, true), 1.0);

    // Every leaf done is the leaves' whole share.
    let leaves_done = tree_time_fraction(TRIANGLES, 1700.0 / total as f64, 10, true);
    assert_within(leaves_done, TREE_LEAF_TIME_SHARE, 1e-9);

    // Every tree node done but not the subtraction: the subtraction still holds a real part of
    // the levels' share - it is the biggest boolean of the run.
    let nodes_done = tree_time_fraction(TRIANGLES, (1700 + 99) as f64 / total as f64, 10, true);
    assert!(nodes_done < TREE_LEAF_TIME_SHARE + (0.9 * TREE_LEVEL_TIME_SHARE));

    // The subtraction done, the closing pass not: the leaves and all the levels.
    let subtracted = tree_time_fraction(TRIANGLES, (total - 1) as f64 / total as f64, 10, true);
    assert_within(
        subtracted,
        TREE_LEAF_TIME_SHARE + TREE_LEVEL_TIME_SHARE,
        1e-9,
    );
}

#[test]
fn the_tree_hull_count_is_recovered_from_its_unit_total() {
    for hulls in 1..=5000i64 {
        for erosion in [false, true] {
            let total = tree_units(hulls, erosion);
            assert_eq!(tree_hull_units(total, erosion), hulls);
        }
    }
}
