//! The reference-counted id allocator (derived from geometry3Sharp) behind
//! [`super::DMesh3`]'s vertices, triangles and edges. A freed id goes on a stack and is handed
//! out again before any new one, which is why mesh ids are not dense after an edit - and why
//! the order ids come back in is part of what an edit sequence produces, so the free stack is
//! strictly last-in, first-out.

/// The count a freed id holds.
const INVALID: i32 = -1;

/// A reference count per id, `INVALID` once freed.
#[derive(Debug, Clone, Default)]
pub(crate) struct RefCountVector {
    ref_counts: Vec<i32>,
    free_indices: Vec<i32>,
    used_count: usize,
}

impl RefCountVector {
    /// The number of live ids.
    pub(crate) fn count(&self) -> usize {
        self.used_count
    }

    /// One past the largest id ever handed out.
    pub(crate) fn max_index(&self) -> usize {
        self.ref_counts.len()
    }

    #[allow(dead_code)] // mesh API only the tests call so far
    /// Nothing has ever been freed (or every free id was reused).
    pub(crate) fn is_dense(&self) -> bool {
        self.free_indices.is_empty()
    }

    /// Whether `index` is an allocated (not freed) id.
    pub(crate) fn is_valid(&self, index: i32) -> bool {
        index >= 0
            && (index as usize) < self.ref_counts.len()
            && self.ref_counts[index as usize] > 0
    }

    /// The reference count of `index`; zero for a freed id.
    pub(crate) fn ref_count(&self, index: i32) -> i32 {
        let n = self.ref_counts[index as usize];
        if n == INVALID {
            0
        } else {
            n
        }
    }

    /// The stored count of `index`, `INVALID` for a freed id.
    pub(crate) fn raw_ref_count(&self, index: i32) -> i32 {
        self.ref_counts[index as usize]
    }

    /// The most recently freed id, else a new one, with a count of one.
    pub(crate) fn allocate(&mut self) -> i32 {
        self.used_count += 1;
        let mut free = INVALID;
        while free == INVALID {
            match self.free_indices.pop() {
                Some(index) => free = index,
                None => break,
            }
        }
        if free != INVALID {
            self.ref_counts[free as usize] = 1;
            return free;
        }
        self.ref_counts.push(1);
        (self.ref_counts.len() - 1) as i32
    }

    /// Adds `increment` to the count of `index` and returns the new count.
    pub(crate) fn increment(&mut self, index: i32, increment: i32) -> i32 {
        self.ref_counts[index as usize] += increment;
        self.ref_counts[index as usize]
    }

    /// Frees the id when its count reaches zero.
    pub(crate) fn decrement(&mut self, index: i32, decrement: i32) {
        let count = &mut self.ref_counts[index as usize];
        *count -= decrement;
        if *count == 0 {
            *count = INVALID;
            self.free_indices.push(index);
            self.used_count -= 1;
        }
    }

    /// Live ids, ascending.
    pub(crate) fn indices(&self) -> impl Iterator<Item = i32> + '_ {
        self.ref_counts
            .iter()
            .enumerate()
            .filter(|(_, &count)| count > 0)
            .map(|(i, _)| i as i32)
    }
}
