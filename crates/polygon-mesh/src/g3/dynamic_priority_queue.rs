//! geometry3Sharp `core/DynamicPriorityQueue.cs`: a binary min-heap on `float` priorities,
//! 1-based as in C#. Ties come out in the order this exact heap produces, which decides the
//! order `MergeCoincidentEdges` tries its merges in, so C#'s sift-up / sift-down comparisons
//! (strict `<`) are kept. Only the operations the merge uses are ported.

/// g3 `DynamicPriorityQueue<T>` (enqueue / dequeue subset); the node is the value itself.
#[derive(Debug, Clone)]
pub(crate) struct DynamicPriorityQueue<T> {
    /// Slot 0 is unused, as in C#.
    nodes: Vec<(T, f32)>,
}

impl<T: Copy + Default> DynamicPriorityQueue<T> {
    pub(crate) fn new() -> Self {
        DynamicPriorityQueue {
            nodes: vec![(T::default(), 0.0)],
        }
    }

    /// C# `Count`.
    pub(crate) fn count(&self) -> usize {
        self.nodes.len() - 1
    }

    /// C# `has_higher_priority`.
    fn higher(&self, a: usize, b: usize) -> bool {
        self.nodes[a].1 < self.nodes[b].1
    }

    /// C# `Enqueue`.
    pub(crate) fn enqueue(&mut self, node: T, priority: f32) {
        self.nodes.push((node, priority));
        let index = self.nodes.len() - 1;
        self.move_up(index);
    }

    /// C# `Dequeue`: the lowest priority; `None` when empty (C# reads a null node).
    pub(crate) fn dequeue(&mut self) -> Option<T> {
        let num_nodes = self.count();
        if num_nodes == 0 {
            return None;
        }
        let first = self.nodes[1].0;
        // C# `Remove(nodes[1])`
        if num_nodes == 1 {
            self.nodes.pop();
            return Some(first);
        }
        self.nodes.swap(1, num_nodes);
        self.nodes.pop();
        // C# `on_node_updated` for the former last node, now at 1: it has no parent
        self.move_down(1);
        Some(first)
    }

    /// C# `move_up`.
    fn move_up(&mut self, mut index: usize) {
        let mut parent = index / 2;
        while parent >= 1 {
            if self.higher(parent, index) {
                break;
            }
            self.nodes.swap(index, parent);
            index = parent;
            parent = index / 2;
        }
    }

    /// C# `move_down`.
    fn move_down(&mut self, mut index: usize) {
        let num_nodes = self.count();
        loop {
            let mut min = index;
            let left = 2 * index;
            if left > num_nodes {
                break;
            }
            if self.higher(left, min) {
                min = left;
            }
            let right = left + 1;
            if right <= num_nodes && self.higher(right, min) {
                min = right;
            }
            if min == index {
                break;
            }
            self.nodes.swap(index, min);
            index = min;
        }
    }
}
