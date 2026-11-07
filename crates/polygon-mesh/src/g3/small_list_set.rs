//! The small list set (derived from geometry3Sharp): the per-vertex edge lists of
//! [`super::DMesh3`]. Each list is an 8-slot block plus a linked overflow list that new values
//! are pushed onto the front of; removing from the block shifts left and pulls the overflow's
//! head into the freed last slot. That layout fixes the order every one-ring walk, and so
//! every boundary loop and edge collapse, follows, so each list is kept as that block and
//! that overflow, without pooled storage.

use std::collections::VecDeque;

/// The slots in a list's fixed block before values overflow.
const BLOCK_SIZE: usize = 8;

#[derive(Debug, Clone, Default)]
struct SmallList {
    block: Vec<i32>,
    /// The linked overflow list: new values go in at the front.
    overflow: VecDeque<i32>,
}

/// One small list per vertex id.
#[derive(Debug, Clone, Default)]
pub(crate) struct SmallListSet {
    lists: Vec<SmallList>,
}

impl SmallListSet {
    /// An empty list at `index`, clearing one already there.
    pub(crate) fn allocate_at(&mut self, index: i32) {
        let index = index as usize;
        if index >= self.lists.len() {
            self.lists.resize_with(index + 1, SmallList::default);
        }
        self.lists[index] = SmallList::default();
    }

    /// Adds `value` to list `index`: into the block while it has room, else onto the front of
    /// the overflow.
    pub(crate) fn insert(&mut self, index: i32, value: i32) {
        let list = &mut self.lists[index as usize];
        if list.block.len() < BLOCK_SIZE {
            list.block.push(value);
        } else {
            list.overflow.push_front(value);
        }
    }

    /// Removes `value` from list `index`; false when it is not in the list.
    pub(crate) fn remove(&mut self, index: i32, value: i32) -> bool {
        let list = &mut self.lists[index as usize];
        if let Some(i) = list.block.iter().position(|&v| v == value) {
            list.block.remove(i);
            if let Some(head) = list.overflow.pop_front() {
                list.block.push(head);
            }
            return true;
        }
        if let Some(i) = list.overflow.iter().position(|&v| v == value) {
            list.overflow.remove(i);
            return true;
        }
        false
    }

    /// Empties list `index`.
    pub(crate) fn clear(&mut self, index: i32) {
        if let Some(list) = self.lists.get_mut(index as usize) {
            *list = SmallList::default();
        }
    }

    /// The number of values in list `index`.
    pub(crate) fn count(&self, index: i32) -> usize {
        let list = &self.lists[index as usize];
        list.block.len() + list.overflow.len()
    }

    /// The first value of list `index`.
    pub(crate) fn first(&self, index: i32) -> i32 {
        self.lists[index as usize].block[0]
    }

    /// The values of list `index`: block order, then the overflow from its head.
    pub(crate) fn values(&self, index: i32) -> Vec<i32> {
        let list = &self.lists[index as usize];
        list.block
            .iter()
            .chain(list.overflow.iter())
            .copied()
            .collect()
    }

    /// The values of list `index` in the same order, borrowed.
    pub(crate) fn iter(&self, index: i32) -> impl Iterator<Item = i32> + '_ {
        let list = &self.lists[index as usize];
        list.block.iter().chain(list.overflow.iter()).copied()
    }
}
