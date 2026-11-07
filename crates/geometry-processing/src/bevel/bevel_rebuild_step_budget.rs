//! How many booleans one bevel rebuild is going to ask the kernel for, which is what the progress
//! bar is shared out over. Read off the bead batch ([`super::BevelBeadBatch`]) and the groups'
//! resolutions before anything is cut.
//!
//! The bar counts what the rebuild's combine is ASKED rather than what succeeds, so the WORST case
//! is what has to be budgeted: a term left OUT here does not make the bar finish early, it makes
//! the ratio walk past 1.0 and stay there, while one left IN only finishes it early. That asymmetry
//! is why several of the terms below count a fallback that usually does not happen.

use std::collections::HashSet;

use polygon_mesh::Mesh;

use super::BevelBeadBatch;
use crate::topology::{BevelEdgeGroupResolution, EdgeConvexity};

/// The whole sum, in the order the rebuild spends it: joining each batch, subtracting, clipping
/// each cutter to the columns some other feature stands in, clipping each bead to the void its own
/// chain may fill, un-burying them, unioning them, and taking the mixed-corner blends off what that
/// leaves.
///
/// `beads` is the bead batch as it stands, whose clips are the largest and most conditional term.
/// `shadow_clips_with_a_body` is how many of the shadow clips carry an obstruction BODY, each of
/// which may have to be cut a second time with the footprint prisms alone - counted where it CAN
/// happen, since nothing knows which way the kernel will go until the first cut.
#[allow(clippy::too_many_arguments)]
pub fn count(
    beads: &BevelBeadBatch,
    cutters: i32,
    blends: i32,
    pair_corner_trims: i32,
    capped_cutters: i32,
    shadow_clips: i32,
    shadow_clips_with_a_body: i32,
    round_profile: bool,
    resolutions: &[BevelEdgeGroupResolution],
    eligible: &[bool],
) -> i32 {
    // Each clippable bead arrives at the batch on its own now, rather than as the one solid the
    // shared clip used to hand back, so the union below has that many more operands to join.
    let bead_count = (beads.whole.len() + beads.clippable.len()) as i32;

    // Every chain's clip is two booleans: the keep-set unioned, then the bead intersected with it.
    // What BevelBeadClip can spend on top of that is a cascade, and the bar counts what the combine
    // is ASKED rather than what succeeds, so the WORST case is what has to be budgeted - anything
    // less walks the ratio past 1.0 on the parts that take a later rung. Each 5.3 CORNER costs
    // three of its own (the beads meeting there unioned, the keep-set unioned, the intersect),
    // counted once however many beads share it, plus one join for the batch they go into.
    let mut clip_steps = 0;
    // Corners are counted by identity, the shared `Rc`'s address: every bead at one corner holds
    // the same void, and two corners never merge for having equal geometry.
    let mut counted_corners: HashSet<*const Mesh> = HashSet::new();

    for clippable in &beads.clippable {
        let Some(corner_voids) = &clippable.corner_voids else {
            continue;
        };

        for corner in corner_voids {
            if counted_corners.insert(std::rc::Rc::as_ptr(corner)) {
                clip_steps += 3;
            }
        }
    }

    // The join that turns the batch of pieces back into one bead solid, once, wherever there was a
    // corner to make pieces of.
    if !counted_corners.is_empty() {
        clip_steps += 1;
    }

    for clippable in &beads.clippable {
        let corners = clippable
            .corner_voids
            .as_ref()
            .map_or(0, |voids| voids.len() as i32);

        // A bead that reaches a corner can spend three on copies of its own corner pieces and
        // their join, two on each corner it copies, two more on the wedge that overshoots when it
        // loses them, and two again if the net fires. A bead with NO corner still runs its two a
        // second time when the net fires, because that re-clips every bead in the batch and not
        // only the ones that had a corner.
        clip_steps += if corners > 0 {
            7 + (2 * corners)
        } else if !counted_corners.is_empty() {
            4
        } else {
            2
        };
    }

    if !counted_corners.is_empty() {
        // And the one join that turns the batch of pieces back into one bead solid.
        clip_steps += 1;
    }

    // What a SECOND PASS could cost, budgeted before it is known to be needed: for each concave
    // group that might be re-read, one clip pair, one sweep's union, and the one intersect that
    // caps it to the source hull - the pass adds its beads through BevelBeadClip.ApplyAll
    // (BevelSecondPassStage.Add), which bounds every bead that went on untrimmed by the hull
    // whether or not anything in the batch was clippable. Plus the one union that puts them on.
    // Counted up rather than exactly, because nothing knows how many creases the cut joined until
    // the cut has been made - and a term left OUT here walks the bar past 1.0 and leaves it there,
    // while one left in only finishes it early.
    // A CHAMFER never re-reads anything (BevelSecondPass hands its groups straight back: the union
    // of two half spaces is already the planar cutback a chamfer wants), so it is budgeted nothing
    // - otherwise every chamfer with a concave group in it leaves the bar short of the end.
    let second_pass_steps = if cutters > 0 && round_profile {
        (4 * could_be_reread(resolutions, eligible)) + 1
    } else {
        0
    };

    second_pass_steps
        + pair_corner_trims
        + capped_cutters
        + shadow_clips
        // A clip that carries a body may have to be cut a SECOND time with the prisms alone,
        // counted where it CAN happen since nothing knows which way the kernel will go until the
        // first cut.
        + shadow_clips_with_a_body
        + i32::from(cutters > 1)
        + i32::from(cutters > 0)
        + clip_steps
        + i32::from(bead_count > 1)
        + i32::from(bead_count > 0 && cutters > 0)
        + i32::from(bead_count > 0)
        + i32::from(blends > 1)
        + i32::from(blends > 0)
}

/// How many groups the second pass might re-read: the eligible ones that are concave throughout.
/// Used for the progress budget only, before anything is cut - what is actually re-read is decided
/// on the cut mesh by `BevelSecondPass`, and is never more than this.
pub fn could_be_reread(resolutions: &[BevelEdgeGroupResolution], eligible: &[bool]) -> i32 {
    let mut count = 0;

    for (i, resolution) in resolutions.iter().enumerate() {
        if !eligible[i] {
            continue;
        }

        let mut concave = !resolution.edges.is_empty();

        for edge in &resolution.edges {
            concave &= edge.convexity == EdgeConvexity::Concave;
        }

        if concave {
            count += 1;
        }
    }

    count
}
