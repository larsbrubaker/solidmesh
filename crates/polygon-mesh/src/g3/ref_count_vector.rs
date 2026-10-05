//! geometry3Sharp `core/RefCountVector.cs`: the id allocator behind [`super::DMesh3`]'s vertices,
//! triangles and edges. A freed id goes on a stack and is handed out again before any new one,
//! which is why g3 ids are not dense after an edit - and why the order ids come back in is part
//! of what an edit sequence produces, so the free stack is kept exactly as C# keeps it.

/// C# `RefCountVector.invalid`.
const INVALID: i32 = -1;

/// g3 `RefCountVector`: a reference count per id, `INVALID` once freed.
#[derive(Debug, Clone, Default)]
pub(crate) struct RefCountVector {
    ref_counts: Vec<i32>,
    free_indices: Vec<i32>,
    used_count: usize,
}

impl RefCountVector {
    /// C# `count`: live ids.
    pub(crate) fn count(&self) -> usize {
        self.used_count
    }

    /// C# `max_index`: one past the largest id ever handed out.
    pub(crate) fn max_index(&self) -> usize {
        self.ref_counts.len()
    }

    #[allow(dead_code)] // geometry3Sharp API no ported caller reaches yet; the tests do
    /// C# `is_dense`: nothing has ever been freed (or every free id was reused).
    pub(crate) fn is_dense(&self) -> bool {
        self.free_indices.is_empty()
    }

    /// C# `isValid`.
    pub(crate) fn is_valid(&self, index: i32) -> bool {
        index >= 0
            && (index as usize) < self.ref_counts.len()
            && self.ref_counts[index as usize] > 0
    }

    /// C# `refCount`: zero for a freed id.
    pub(crate) fn ref_count(&self, index: i32) -> i32 {
        let n = self.ref_counts[index as usize];
        if n == INVALID {
            0
        } else {
            n
        }
    }

    /// C# `rawRefCount`.
    pub(crate) fn raw_ref_count(&self, index: i32) -> i32 {
        self.ref_counts[index as usize]
    }

    /// C# `allocate`: the most recently freed id, else a new one, with a count of one.
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

    /// C# `increment`.
    pub(crate) fn increment(&mut self, index: i32, increment: i32) -> i32 {
        self.ref_counts[index as usize] += increment;
        self.ref_counts[index as usize]
    }

    /// C# `decrement`: frees the id when its count reaches zero.
    pub(crate) fn decrement(&mut self, index: i32, decrement: i32) {
        let count = &mut self.ref_counts[index as usize];
        *count -= decrement;
        if *count == 0 {
            *count = INVALID;
            self.free_indices.push(index);
            self.used_count -= 1;
        }
    }

    /// C# `foreach (int i in refcounts)`: live ids, ascending.
    pub(crate) fn indices(&self) -> impl Iterator<Item = i32> + '_ {
        self.ref_counts
            .iter()
            .enumerate()
            .filter(|(_, &count)| count > 0)
            .map(|(i, _)| i as i32)
    }
}
