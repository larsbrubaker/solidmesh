//! Whether the ROOM and the BUILD read the same partial run-offs on one pass of plans - the guard
//! that keeps a size allowed for an end nobody then closes from being built. Ports
//! `Bevel/Surface/SurfaceRunOffAgreement.cs` (C# `internal static class`; a free function here):
//! [`the_same_on_both_readings`]. Lifted out of `SurfaceBevelBuilder`; the reading is the
//! builder's own and reads nothing else of it.

use std::collections::HashSet;

use super::{SurfaceBandRunsOffOverAStretch, SurfaceEdgePlan};
use crate::bevel::{BevelAdvisory, BevelAdvisoryCode};

/// One partial run-off by NAME: which plan, which edge of that plan, which side, which end.
type Named = (i32, i32, i32, bool);

/// C# `TheSameOnBothReadings`: whether the partial run-offs this build read (`over_a_stretch`,
/// C#'s `[edge, side]` table) are the ones the ROOM read (`the_rooms_own`), side for side and end
/// for end. Where the caller did not hand its own reading over (`None`) there is nothing to compare
/// and the build goes on with its own. `Err` names the first edge whose reading differs.
///
/// The two are the same call on the same plans, so they can only differ where one of them never
/// asks: this build SKIPS a side whose strip a blend swallowed, and one whose rail the mitres have
/// consumed, and the room asks of every side of every edge. A stretch on such a side is a size the
/// room allowed for an end this build never closes - the strip would stand off the part with
/// nothing to end it - and the difference is what says so.
///
/// Compared by NAME rather than by geometry - which plan, which edge of that plan, which side and
/// which end - because that is what the build looks the stretch up by when it closes the end.
/// C#'s `plans` parameter, which the body never reads, is dropped.
pub fn the_same_on_both_readings(
    first_of_plan: &[i32],
    over_a_stretch: &[[Option<SurfaceBandRunsOffOverAStretch>; 2]],
    the_rooms_own: Option<&[SurfaceBandRunsOffOverAStretch]>,
    edges: &[SurfaceEdgePlan],
) -> Result<(), BevelAdvisory> {
    let Some(the_rooms_own) = the_rooms_own else {
        return Ok(());
    };

    // Insertion-ordered sets (C#'s `HashSet` enumerates in insertion order while nothing is
    // removed), so the edge a disagreement is named at is the one C# names.
    let mut mine: Vec<Named> = Vec::new();
    let mut theirs: Vec<Named> = Vec::new();
    let mut mine_set: HashSet<Named> = HashSet::new();
    let mut theirs_set: HashSet<Named> = HashSet::new();

    // C#'s `foreach` over a `T?[,]` walks it row by row: edge by edge, side by side.
    for stretch in over_a_stretch.iter().flatten().flatten() {
        let named = (
            stretch.plan_index,
            stretch.edge_index,
            stretch.side,
            stretch.at_the_start,
        );

        if mine_set.insert(named) {
            mine.push(named);
        }
    }

    for stretch in the_rooms_own {
        let named = (
            stretch.plan_index,
            stretch.edge_index,
            stretch.side,
            stretch.at_the_start,
        );

        if theirs_set.insert(named) {
            theirs.push(named);
        }
    }

    let disagree = |named: &Named| {
        BevelAdvisory::new(
            BevelAdvisoryCode::SurfaceRunOffReadingsDisagree,
            vec![edges[(first_of_plan[named.0 as usize] + named.1) as usize]
                .from()
                .into()],
        )
    };

    for named in &theirs {
        if !mine_set.contains(named) {
            return Err(disagree(named));
        }
    }

    for named in &mine {
        if !theirs_set.contains(named) {
            return Err(disagree(named));
        }
    }

    Ok(())
}
