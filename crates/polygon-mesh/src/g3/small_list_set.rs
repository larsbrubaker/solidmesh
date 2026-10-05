//! geometry3Sharp `core/SmallListSet.cs`: the per-vertex edge lists of [`super::DMesh3`]. C#
//! packs each list into an 8-slot block plus a linked overflow list that new values are pushed
//! onto the front of; removing from the block shifts left and pulls the overflow's head into the
//! freed last slot. Only the order that layout produces is observable (every one-ring walk, and
//! so every boundary loop and edge collapse, follows it), so each list is kept here as that
//! block and that overflow, without the pooled storage.

use std::collections::VecDeque;

/// C# `SmallListSet.BLOCKSIZE`.
const BLOCK_SIZE: usize = 8;

#[derive(Debug, Clone, Default)]
struct SmallList {
    block: Vec<i32>,
    /// The linked list: front is the head C# inserts at.
    overflow: VecDeque<i32>,
}

/// g3 `SmallListSet`.
#[derive(Debug, Clone, Default)]
pub(crate) struct SmallListSet {
    lists: Vec<SmallList>,
}

impl SmallListSet {
    /// C# `AllocateAt` (after `Clear` when the slot exists): an empty list at `index`.
    pub(crate) fn allocate_at(&mut self, index: i32) {
        let index = index as usize;
        if index >= self.lists.len() {
            self.lists.resize_with(index + 1, SmallList::default);
        }
        self.lists[index] = SmallList::default();
    }

    /// C# `Insert`.
    pub(crate) fn insert(&mut self, index: i32, value: i32) {
        let list = &mut self.lists[index as usize];
        if list.block.len() < BLOCK_SIZE {
            list.block.push(value);
        } else {
            list.overflow.push_front(value);
        }
    }

    /// C# `Remove`: false when `value` is not in the list.
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

    /// C# `Clear`.
    pub(crate) fn clear(&mut self, index: i32) {
        if let Some(list) = self.lists.get_mut(index as usize) {
            *list = SmallList::default();
        }
    }

    /// C# `Count`.
    pub(crate) fn count(&self, index: i32) -> usize {
        let list = &self.lists[index as usize];
        list.block.len() + list.overflow.len()
    }

    /// C# `First`.
    pub(crate) fn first(&self, index: i32) -> i32 {
        self.lists[index as usize].block[0]
    }

    /// C# `ValueItr`, collected: block order, then the overflow from its head.
    pub(crate) fn values(&self, index: i32) -> Vec<i32> {
        let list = &self.lists[index as usize];
        list.block
            .iter()
            .chain(list.overflow.iter())
            .copied()
            .collect()
    }

    /// C# `ValueItr`, borrowed.
    pub(crate) fn iter(&self, index: i32) -> impl Iterator<Item = i32> + '_ {
        let list = &self.lists[index as usize];
        list.block.iter().chain(list.overflow.iter()).copied()
    }
}
