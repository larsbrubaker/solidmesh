//! A binary min-heap on `f32` priorities whose nodes are integer ids below a fixed maximum, with an
//! id → heap-slot map so a queued id can be found, re-prioritized or removed in place. The
//! `Reducer` (`reducer.rs`) queues its edges here. Ties come out in the order this exact heap
//! produces, so the sift-up / sift-down comparisons are strict `<`, and slot 0 is unused so a
//! node's parent is at half its index.
//!
//! `contains` also checks that the slot an id maps to still holds that id: a removed id's map entry
//! still points at its old slot, which a later insert reuses, so without the check a dequeued id
//! would read as queued and `update` would re-prioritize whichever id sat there.

#[derive(Debug, Clone, Copy, Default)]
struct QueueNode {
    /// External id.
    id: i32,
    /// The priority of this id.
    priority: f32,
    /// Index in the tree (stored as a flat array); 0 for an empty slot.
    index: i32,
}

#[derive(Debug, Clone)]
pub(crate) struct IndexPriorityQueue {
    /// Tree of allocated nodes, stored linearly, active up to `num_nodes`; grows on `insert`.
    nodes: Vec<QueueNode>,
    /// Count of active nodes.
    num_nodes: i32,
    /// Mapping from external ids to internal node indices.
    id_to_index: Vec<i32>,
}

impl IndexPriorityQueue {
    /// Ids must be below `max_id`.
    pub(crate) fn new(max_id: i32) -> Self {
        Self {
            nodes: Vec::new(),
            num_nodes: 0,
            id_to_index: vec![-1; max_id.max(0) as usize],
        }
    }

    pub(crate) fn count(&self) -> i32 {
        self.num_nodes
    }

    fn node(&self, i: i32) -> QueueNode {
        // Never reads past the end: every read index is at most `num_nodes`, which `insert` has
        // written.
        self.nodes[i as usize]
    }

    fn set_node(&mut self, i: i32, n: QueueNode) {
        let i = i as usize;
        if i >= self.nodes.len() {
            self.nodes.resize(i + 1, QueueNode::default());
        }
        self.nodes[i] = n;
    }

    /// Constant-time check whether `id` is queued.
    pub(crate) fn contains(&self, id: i32) -> bool {
        let i_node = self.id_to_index[id as usize];
        if i_node <= 0 || i_node > self.num_nodes {
            return false;
        }
        let node = self.node(i_node);
        // The slot must still hold this id: a stale map entry can point at a reused slot.
        node.index > 0 && node.id == id
    }

    /// Adds `id` with `priority`. Inserting a queued id again is not supported.
    pub(crate) fn insert(&mut self, id: i32, priority: f32) {
        self.num_nodes += 1;
        let node = QueueNode {
            id,
            priority,
            index: self.num_nodes,
        };
        self.id_to_index[id as usize] = node.index;
        self.set_node(self.num_nodes, node);
        self.move_up(self.node(self.num_nodes).index);
    }

    /// Removes the head of the queue and returns its id.
    pub(crate) fn dequeue(&mut self) -> i32 {
        let id = self.node(1).id;
        self.remove_at_index(1);
        id
    }

    #[allow(dead_code)] // only the tests call this
    /// Removes a queued `id`.
    pub(crate) fn remove(&mut self, id: i32) {
        let i_node = self.id_to_index[id as usize];
        self.remove_at_index(i_node);
    }

    /// Sets a queued `id`'s priority and moves it to its place.
    pub(crate) fn update(&mut self, id: i32, priority: f32) {
        let i_node = self.id_to_index[id as usize];
        let mut n = self.node(i_node);
        n.priority = priority;
        self.set_node(i_node, n);
        self.on_node_updated(i_node);
    }

    #[allow(dead_code)] // only the tests call this
    pub(crate) fn get_priority(&self, id: i32) -> f32 {
        self.node(self.id_to_index[id as usize]).priority
    }

    #[allow(dead_code)] // only the tests call this
    /// The id at the head of the queue.
    pub(crate) fn first(&self) -> i32 {
        self.node(1).id
    }

    #[allow(dead_code)] // only the tests call this
    /// Whether every child's priority is at least its parent's (slots below `num_nodes`).
    pub(crate) fn is_valid_queue(&self) -> bool {
        for i in 1..self.num_nodes {
            let child_left_index = 2 * i;
            if child_left_index < self.num_nodes && self.has_higher_priority(child_left_index, i) {
                return false;
            }
            let child_right_index = child_left_index + 1;
            if child_right_index < self.num_nodes && self.has_higher_priority(child_right_index, i)
            {
                return false;
            }
        }
        true
    }

    fn remove_at_index(&mut self, i_node: i32) {
        // node-is-last-node case
        if i_node == self.num_nodes {
            self.set_node(i_node, QueueNode::default());
            self.num_nodes -= 1;
            return;
        }

        // Swap the node with the last node
        self.swap_nodes_at_indices(i_node, self.num_nodes);
        // after swap, i_node is the one we want to keep, and num_nodes is the one we discard
        self.set_node(self.num_nodes, QueueNode::default());
        self.num_nodes -= 1;

        // Now shift i_node (ie the former last node) up or down as appropriate
        self.on_node_updated(i_node);
    }

    fn swap_nodes_at_indices(&mut self, i1: i32, i2: i32) {
        let mut n1 = self.node(i1);
        n1.index = i2;
        let mut n2 = self.node(i2);
        n2.index = i1;
        self.set_node(i1, n2);
        self.set_node(i2, n1);

        self.id_to_index[n2.id as usize] = i1;
        self.id_to_index[n1.id as usize] = i2;
    }

    /// Move the node at `i_from` to `i_to`.
    fn move_node(&mut self, i_from: i32, i_to: i32) {
        let mut n = self.node(i_from);
        n.index = i_to;
        self.set_node(i_to, n);
        self.id_to_index[n.id as usize] = i_to;
    }

    /// Set the node at `i_to`.
    fn set(&mut self, i_to: i32, mut n: QueueNode) {
        n.index = i_to;
        self.set_node(i_to, n);
        self.id_to_index[n.id as usize] = i_to;
    }

    /// Move `i_node` up the tree to its place by iteratively swapping with its parent.
    fn move_up(&mut self, mut i_node: i32) {
        let i_start = i_node;
        let i_start_node = self.node(i_start);

        // while our priority is lower than parent, we swap upwards, ie move parent down
        let mut i_parent = i_node / 2;
        while i_parent >= 1 {
            if self.node(i_parent).priority < i_start_node.priority {
                break;
            }

            self.move_node(i_parent, i_node);
            i_node = i_parent;
            i_parent = self.node(i_node).index / 2;
        }

        // write input node into final position, if we moved it
        if i_node != i_start {
            self.set(i_node, i_start_node);
        }
    }

    /// Move `i_node` down the tree to its place by iteratively swapping with its children.
    fn move_down(&mut self, mut i_node: i32) {
        let i_start = i_node;
        let i_start_node = self.node(i_start);

        // keep moving down until lower nodes have higher priority
        loop {
            let mut i_move_to = i_node;
            let i_left_child = 2 * i_node;

            // past end of tree, must be in the right spot
            if i_left_child > self.num_nodes {
                break;
            }

            // check if priority is larger than either child - if so we want to swap
            let mut min_priority = i_start_node.priority;
            let left_child_priority = self.node(i_left_child).priority;
            if left_child_priority < min_priority {
                i_move_to = i_left_child;
                min_priority = left_child_priority;
            }
            let i_right_child = i_left_child + 1;
            if i_right_child <= self.num_nodes && self.node(i_right_child).priority < min_priority {
                i_move_to = i_right_child;
            }

            // if we found node with higher priority, swap with it (ie move it up) and take its
            // place (but we only write start node to final position, not intermediary slots)
            if i_move_to != i_node {
                self.move_node(i_move_to, i_node);
                i_node = i_move_to;
            } else {
                break;
            }
        }

        // if we moved node, write it to its new position
        if i_node != i_start {
            self.set(i_node, i_start_node);
        }
    }

    /// Call after a node is modified, to move it to its place in the queue.
    fn on_node_updated(&mut self, i_node: i32) {
        let i_parent = i_node / 2;
        if i_parent > 0 && self.has_higher_priority(i_node, i_parent) {
            self.move_up(i_node);
        } else {
            self.move_down(i_node);
        }
    }

    /// Whether the priority at `i_higher` is less than at `i_lower`.
    fn has_higher_priority(&self, i_higher: i32, i_lower: i32) -> bool {
        self.node(i_higher).priority < self.node(i_lower).priority
    }
}
