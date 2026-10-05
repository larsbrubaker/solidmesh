//! Port of agg-sharp `Csg/MinkowskiProgressModel.cs`: turns the kernel's Minkowski progress -
//! which counts work items - into a fraction of the expected running time, so a bar driven by
//! it moves at a steady rate. Used by the Minkowski runs in `minkowski_processing`.
//!
//! The kernel's nonconvex (+) convex sweep reports one unit per per-triangle hull, one per
//! 1000-hull batch union and one for the closing merge, all weighted the same. They are not the
//! same cost. Measured in Release on a sphere-minus-cube dilated by a 12-segment ball (730 to
//! 7722 triangles): the hulls are about 5% of the run, the batch unions about 85% and the
//! closing merge the rest. Counted as units the bar raced to the end of each batch's hulls and
//! then sat through its union, and sat again at the very end through the merge. Weighted by
//! those measured costs the bar tracks the clock to within the batch granularity: it still
//! holds during a union (the kernel reports nothing inside one) but no longer lies about how
//! much is left.
//!
//! The unit layout mirrors manifold-rust's `minkowski.rs` (`BATCH_SIZE`, one `advance` per hull
//! and per batch union, `complete_phase` for the merge); if the kernel's batch size or
//! reporting changes, this has to follow it. The union-tree half mirrors the kernel's
//! `ConvexDilation` leaf size and reduction (manifold-sharp; see the tree functions below).

/// The kernel's batch size (`Minkowski.BatchSize`).
pub const BATCH_SIZE: i32 = 1000;

// Relative cost per triangle of each piece of the sweep, from the Release measurements above.
const HULL_WEIGHT: f64 = 1.0;
const BATCH_UNION_WEIGHT: f64 = 17.0;
const MERGE_WEIGHT: f64 = 2.0;

/// The number of progress units the kernel's sweep reports for a solid of `triangles`
/// triangles: a hull each, a union per batch and the closing merge.
pub fn kernel_units(triangles: i32) -> i64 {
    let batches = (triangles as i64 + BATCH_SIZE as i64 - 1) / BATCH_SIZE as i64;
    triangles as i64 + batches + 1
}

/// Maps the kernel's unit fraction for a sweep over `triangles` triangles to the fraction of
/// the expected running time spent so far. Monotonic, 0 at 0 and 1 at 1.
pub fn time_fraction(triangles: i32, kernel_fraction: f64) -> f64 {
    if triangles <= 0 || kernel_fraction >= 1.0 {
        return kernel_fraction.clamp(0.0, 1.0);
    }

    if kernel_fraction <= 0.0 {
        return 0.0;
    }

    // The kernel emits fraction = done / total; rounding recovers the whole unit count it had.
    // C# Math.Round is banker's rounding.
    let mut units = (kernel_fraction * kernel_units(triangles) as f64).round_ties_even() as i64;
    let mut work = 0.0;
    let mut start = 0;
    while start < triangles && units > 0 {
        let size = BATCH_SIZE.min(triangles - start);
        if units > size as i64 {
            // Every hull of this batch and its union are done.
            work += size as f64 * (HULL_WEIGHT + BATCH_UNION_WEIGHT);
            units -= size as i64 + 1;
        } else {
            work += units as f64 * HULL_WEIGHT;
            units = 0;
        }
        start += BATCH_SIZE;
    }

    if units > 0 {
        work += triangles as f64 * MERGE_WEIGHT;
    }

    (work / (triangles as f64 * (HULL_WEIGHT + BATCH_UNION_WEIGHT + MERGE_WEIGHT))).clamp(0.0, 1.0)
}

/// The kernel's dilation-tree leaf size (`ConvexDilation.LeafSize`).
pub const TREE_LEAF_SIZE: i32 = 16;

/// The share of a dilation tree's running time spent building and unioning the leaves.
///
/// From two Release runs of the G203 part (46854 triangles, a 12-segment ball): the leaves took
/// 55-57 s of runs lasting 81 s and 120 s (the second on a loaded machine), so a bit over half.
/// The pairwise levels and the closing pass share the rest, and in the slower run the top few
/// levels alone held the bar for 40 s when the levels were counted by node - which is why the
/// levels are weighted below rather than counted.
pub const TREE_LEAF_TIME_SHARE: f64 = 0.55;

/// The share of a dilation tree's running time spent in the pairwise levels.
pub const TREE_LEVEL_TIME_SHARE: f64 = 0.37;

/// How much more a union at one tree level costs than one at the level below. Each level's
/// operands are about twice the size, but siblings only overlap along a thin seam, so the cost
/// grows more slowly than the size. 1.5 puts the first two levels of the G203 tree at about a
/// third of the levels' time, as measured (about 20 s of about 60 s).
pub const TREE_LEVEL_COST_GROWTH: f64 = 1.5;

/// The number of progress units the kernel's union tree reports for `hull_units` hulls: a unit
/// per hull, one per leaf, one per tree node - a binary reduction of L leaves is L - 1 unions -
/// and one for the closing pass. A dilation (`Manifold.TryDilateByConvex`) has the solid as
/// leaf 0; an erosion (`Manifold.TryErodeByConvex`, `erosion`) leaves it out of the tree and
/// spends one more unit subtracting the union from it.
///
/// An erosion builds one hull per triangle, so its hull units are the triangle count. A
/// dilation builds one hull per convex patch of up to 16 triangles, a count only the kernel
/// knows; [`tree_hull_units`] recovers it from the phase total the kernel reports. Either way a
/// solid with nested shells is first rebuilt by the kernel, which changes its triangle count,
/// so the total is the one to trust.
pub fn tree_units(hull_units: i64, erosion: bool) -> i64 {
    let leaves = tree_leaves(hull_units, erosion);
    hull_units + leaves + (leaves - 1) + 1 + if erosion { 1 } else { 0 }
}

/// The hull count behind a union tree whose phase total is `total_units` (the kernel
/// reporter's phase total): the inverse of [`tree_units`], which grows strictly with the hull
/// count. 0 when no hull count produces that total.
pub fn tree_hull_units(total_units: i64, erosion: bool) -> i64 {
    // tree_units is about hulls * 18 / 16, so start just under the answer and walk up.
    let mut hulls = 0i64.max(((total_units - 4) * 8 / 9) - 2);
    while tree_units(hulls + 1, erosion) <= total_units {
        hulls += 1;
    }

    if tree_units(hulls, erosion) == total_units {
        hulls
    } else {
        0
    }
}

/// Maps the kernel's unit fraction for a union tree over `hull_units` hulls to the fraction of
/// the expected running time. Monotonic, 0 at 0 and 1 at 1.
///
/// Unlike the sweep, the tree's units are not reported in a fixed order: leaves run in
/// parallel, each reporting its hulls and then itself, so the count only says how much is done,
/// not which piece. That is enough, because the stages are sequential: every hull and leaf unit
/// is spent before the first tree node, and each tree level finishes before the next starts, so
/// a count of finished nodes names the level being worked on and how far through it the run is.
///
/// The leaf stage maps linearly onto its share - a leaf's sixteen hull units and its union unit
/// arrive together, so the count is proportional to leaf work. The levels do not: the bottom
/// one is half the nodes but many small unions spread over every core, while the top ones are a
/// few large unions with cores idle. Each level is given a wall-time weight of its rounds of
/// work (unions over `parallelism`, rounded up) times a per-union cost growing by
/// [`TREE_LEVEL_COST_GROWTH`] per level, and the bar moves through a level linearly by its own
/// nodes. The closing pass - one kernel unit, spent only when the phase completes - is the
/// remainder. The caller's high-water mark keeps the bar from stepping back when two workers
/// report out of order.
///
/// An erosion's closing subtraction - the solid minus the tree's union, one boolean on the two
/// largest operands - is counted as one more level of one union above the tree's top, which is
/// what it is: the dilation tree unions that same solid in as its extra leaf.
///
/// `parallelism` is how many unions run at once: the core count when the kernel runs in
/// parallel, 1 when it does not.
pub fn tree_time_fraction(
    hull_units: i64,
    kernel_fraction: f64,
    parallelism: i32,
    erosion: bool,
) -> f64 {
    if hull_units <= 0 || kernel_fraction >= 1.0 {
        return kernel_fraction.clamp(0.0, 1.0);
    }

    if kernel_fraction <= 0.0 {
        return 0.0;
    }

    let total = tree_units(hull_units, erosion);
    let leaves = tree_leaves(hull_units, erosion);
    let leaf_stage_units = hull_units + leaves;
    let units = kernel_fraction * total as f64;
    if units <= leaf_stage_units as f64 {
        return TREE_LEAF_TIME_SHARE * units / leaf_stage_units as f64;
    }

    // Walk the levels exactly as ConvexDilation reduces them: a level of n nodes performs n / 2
    // unions and carries an odd one up unchanged.
    let cores = parallelism.max(1) as i64;
    let mut unions_per_level: Vec<i64> = Vec::new();
    let mut weights: Vec<f64> = Vec::new();
    let mut total_weight = 0.0;
    let mut cost = 1.0;
    let mut nodes = leaves;
    while nodes > 1 {
        let unions = nodes / 2;
        // Integer rounds of work, as C#'s long division.
        let weight = cost * ((unions + cores - 1) / cores) as f64;
        unions_per_level.push(unions);
        weights.push(weight);
        total_weight += weight;
        cost *= TREE_LEVEL_COST_GROWTH;
        nodes = (nodes / 2) + (nodes & 1);
    }

    if erosion {
        // The closing subtraction: one union's worth at the next level up.
        unions_per_level.push(1);
        weights.push(cost);
        total_weight += cost;
    }

    let mut done_nodes = units - leaf_stage_units as f64;
    let mut done_weight = 0.0;
    let mut level = 0;
    while level < unions_per_level.len() && done_nodes > 0.0 {
        let through = done_nodes.min(unions_per_level[level] as f64);
        done_weight += weights[level] * through / unions_per_level[level] as f64;
        done_nodes -= through;
        level += 1;
    }

    let levels_done = if total_weight <= 0.0 {
        1.0
    } else {
        done_weight / total_weight
    };
    (TREE_LEAF_TIME_SHARE + (TREE_LEVEL_TIME_SHARE * levels_done)).clamp(0.0, 1.0)
}

fn tree_leaves(hull_units: i64, erosion: bool) -> i64 {
    // A dilation adds the solid as one more leaf; an erosion subtracts it at the end instead.
    ((hull_units + TREE_LEAF_SIZE as i64 - 1) / TREE_LEAF_SIZE as i64) + if erosion { 0 } else { 1 }
}
