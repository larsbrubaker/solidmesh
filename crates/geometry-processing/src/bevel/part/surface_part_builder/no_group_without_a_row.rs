//! The last word on a part's rows: a group the surface construction was offered always comes back
//! with one. The passes write a row for every group they build or refuse, and a pass that ends
//! with no mesh and no reason of its own leaves its groups without any; a group answered with
//! nothing would vanish from the caller's list, with no reason and nothing to act on.

use crate::bevel::{BevelAdvisory, BevelAdvisoryCode, BevelGroupResult, BevelResultGroup};
use crate::topology::BevelEdgeGroup;

/// Writes a refusal for each of `building` (indices into `groups` and `results`) that has no row,
/// leaving every row already written alone. `edge_count_of` is how many edges the group resolved
/// to.
pub fn refuse_the_groups_with_no_row(
    building: &[i32],
    results: &mut [Option<BevelGroupResult>],
    groups: &[BevelEdgeGroup],
    edge_count_of: impl Fn(usize) -> i32,
) {
    for &index in building {
        let slot = index as usize;
        if results[slot].is_none() {
            results[slot] = Some(BevelGroupResult::new(
                index,
                BevelResultGroup::Edge(groups[slot].clone()),
                edge_count_of(slot),
                Some(BevelAdvisory::new(
                    BevelAdvisoryCode::SurfaceConstructionNotBuilt,
                    Vec::new(),
                )),
                Vec::new(),
            ));
        }
    }
}
