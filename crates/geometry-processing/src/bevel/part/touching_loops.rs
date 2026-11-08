//! Which CLOSED loops of a many-loop bevel pick to leave alone so that no two of the rest touch
//! ([`the_ones_to_leave_alone`]) - the pick the level line refuses by name
//! (`BandGapRefusal::ChainsTouch`) made into one it can be asked again. Reads touches with the
//! level line's own `pick_validation::every_touching_pair`.
//!
//! Without this the refused pass went to the older Surface construction, which refuses a loop of
//! its own choosing - on a plate with many port mouths, an unrelated mouth it cannot roll - and
//! the pass came round again with the touching pair still in it: twenty-one innocent mouths
//! dropped, one a round, before the pair happened to go.
//!
//! Only CLOSED loops are judged. A corner - where the kernel builds touching chains by turning one
//! into the other - needs a chain that ENDS at the shared vertex (`PickValidation.CornersOf`), so
//! two closed loops meeting at a vertex are never a corner and always the named refusal. Open runs
//! are left to the route as it was, which is how a stage-3 corner pick keeps the construction that
//! builds it.
//!
//! The rule for a pair: the loop with MORE edges stays, and on a tie the one picked first. Nothing
//! has been built loop by loop at this point, so which of the two would build alone is not known
//! without paying a build per loop; the larger loop is the more of the user's bevel kept, and the
//! order is the pick's own so the answer is the same every rebuild. Measured on that plate at 0.5
//! the other way round (leave the two 70 edge loops, keep the 64 edge mouths) builds the same 37
//! of 52: whichever of the pair stays is then refused by size.

use std::collections::{HashMap, HashSet};

use super::THE_MANY_LOOP_RIM_FLOOR;
use crate::bevel::level_line::pick_validation::every_touching_pair;

/// The loops to leave alone, as `(loop, partner, vertex)` -
/// indices into `loops`, each with the loop it touches that stays and the vertex where they meet -
/// or nothing where no two closed loops touch. `loops` holds each loop's edges as
/// `(start vertex, end vertex)`, in the pick's own order.
pub fn the_ones_to_leave_alone(loops: &[Vec<(i32, i32)>]) -> Vec<(usize, usize, i32)> {
    let mut left_alone = Vec::new();

    if loops.len() < THE_MANY_LOOP_RIM_FLOOR {
        return left_alone;
    }

    // The same loop picked in two groups is ONE loop here: paired with its own copy it shares
    // every vertex with it, and one of the two would be left alone for touching itself. A copy
    // takes part in no pair of its own and goes wherever its first appearance goes.
    let mut first_of: HashMap<Vec<(i32, i32)>, usize> = HashMap::new();
    let mut original = vec![0usize; loops.len()];

    for (index, edges) in loops.iter().enumerate() {
        let mut key: Vec<(i32, i32)> = edges
            .iter()
            .map(|&(start, end)| (start.min(end), start.max(end)))
            .collect();
        key.sort_unstable();
        original[index] = *first_of.entry(key).or_insert(index);
    }

    let vertices: Vec<Vec<i32>> = loops
        .iter()
        .enumerate()
        .map(|(index, edges)| {
            if original[index] != index {
                return Vec::new();
            }
            // Each loop's vertices without repeats, in first appearance order, so the pick is
            // stable.
            let mut seen = HashSet::new();
            edges
                .iter()
                .flat_map(|&(start, end)| [start, end])
                .filter(|vertex| seen.insert(*vertex))
                .collect()
        })
        .collect();

    // The kernel's own reading of a touch (a shared vertex), taken over every pair at once so the
    // whole rim is answered in one round rather than a pair per rebuild.
    let touching = every_touching_pair(&vertices);

    // All or nothing: where an OPEN run is in any touch, the route goes on exactly as it did. On
    // Many Holes that is the first passes, whose open runs at the cut end are dropped by size down
    // the older path; answering only the closed pairs there took a loop out from under that path
    // and it then refused the whole rim at once. Once the open runs are gone, the closed pairs are
    // all that is left.
    if touching.iter().any(|&(chain, other, _)| {
        !is_closed(&loops[chain as usize]) || !is_closed(&loops[other as usize])
    }) {
        return left_alone;
    }

    let mut gone = HashSet::new();

    for (first, second, vertex) in touching {
        let (first, second) = (first as usize, second as usize);

        // Either one already gone breaks this touch too, and dropping a second would take a loop
        // the user keeps for nothing.
        if gone.contains(&first) || gone.contains(&second) {
            continue;
        }

        let leaving = if loops[second].len() > loops[first].len() {
            first
        } else {
            second
        };

        let staying = if leaving == first { second } else { first };

        gone.insert(leaving);

        for (index, &of) in original.iter().enumerate() {
            if of == leaving {
                left_alone.push((index, staying, vertex));
            }
        }
    }

    left_alone
}

/// Whether a loop's edges close into a ring - at least three, and every vertex on
/// exactly two of them.
fn is_closed(edges: &[(i32, i32)]) -> bool {
    if edges.len() < 3 {
        return false;
    }

    let mut uses: HashMap<i32, i32> = HashMap::new();

    for &(start, end) in edges {
        *uses.entry(start).or_default() += 1;
        *uses.entry(end).or_default() += 1;
    }

    uses.values().all(|&count| count == 2)
}
