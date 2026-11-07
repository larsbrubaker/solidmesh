//! The edits the property panel's group list makes to a fillet or chamfer's selection, as plain
//! list arithmetic.
//!
//! Every function returns a NEW list and leaves the one it was handed - and every group in it -
//! exactly as it found them: a rebuild reads the selection on a worker while the panel keeps
//! editing it, so assigning a whole new list is what cannot be seen half done. Slots compare by
//! identity, not value: the lists hold `Arc`s and a slot "holds" a group when it is the same
//! allocation (`Arc::ptr_eq`), so two equal groups stay two rows. Generic over the kind of group
//! so both legs share one set of rules.

use std::sync::Arc;

use super::bevel_group::BevelGroup;

/// A field-for-field copy of a group, so a reader already walking the old list keeps seeing the
/// numbers the build it started was given.
pub fn copy<T: BevelGroup>(group: &T) -> T {
    group.copy()
}

/// The list without one group, or `None` when that slot does not hold that group any more.
pub fn without<T: BevelGroup>(
    groups: &[Arc<T>],
    index: usize,
    expected: &Arc<T>,
) -> Option<Vec<Arc<T>>> {
    if !holds(groups, index, expected) {
        return None;
    }

    let mut updated = Vec::with_capacity(groups.len() - 1);
    for (i, group) in groups.iter().enumerate() {
        if i != index {
            updated.push(Arc::clone(group));
        }
    }

    Some(updated)
}

/// The list with one group swapped for another in its own slot, or `None` when that slot does
/// not hold the group the caller was built for. This is what choosing a reading does: a reading
/// is a whole different group, so it takes the old one's place rather than being written into it.
pub fn replacing<T: BevelGroup>(
    groups: &[Arc<T>],
    index: usize,
    expected: &Arc<T>,
    replacement: Arc<T>,
) -> Option<Vec<Arc<T>>> {
    if !holds(groups, index, expected) {
        return None;
    }

    let mut updated = Vec::with_capacity(groups.len());
    for (i, group) in groups.iter().enumerate() {
        updated.push(if i == index {
            Arc::clone(&replacement)
        } else {
            Arc::clone(group)
        });
    }

    Some(updated)
}

/// Whether a slot still holds the group the caller thinks it does. A panel row captures its
/// index and its group when drawn, and the list can change under it before the row rebuild
/// catches up; without this a Remove aimed at one edge would drop whoever now has the slot.
fn holds<T>(groups: &[Arc<T>], index: usize, expected: &Arc<T>) -> bool {
    index < groups.len() && Arc::ptr_eq(&groups[index], expected)
}
