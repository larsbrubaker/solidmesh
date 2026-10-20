//! [`MeshEdgeGraph`] built a budget at a time, for a caller on a thread that must keep drawing
//! frames (the browser's): [`MeshEdgeGraphSteps::step`] does about a budget of faces' worth of
//! the build and answers the graph from the step that finishes it.
//!
//! The build is [`MeshEdgeGraph::create`]'s, cut into pieces: the same (edge key, face index)
//! slots in face order, sorted, then the same walk over equal-key runs. The one sort becomes a
//! merge sort (short runs sorted in place, then merged pairwise a slot at a time), which puts
//! the slots in the same order: they are sorted on the whole (key, face) pair, and two slots that
//! compare equal are equal, so every sort of them gives the same sequence. The graph is therefore
//! the one `create` builds, however the work is cut up.

use crate::mesh::Mesh;
use crate::mesh_edge_graph::{edge_key, MeshEdgeGraph};

/// The slots sorted in place as one run before the merges.
const RUN_SLOTS: usize = 768;
/// The work of sorting one slot inside its run, against one slot of a merge or the walk: a run's
/// sort compares each slot about log2(`RUN_SLOTS`) times, a merge or the walk once.
const RUN_SORT_WORK: usize = 10;
/// The work of one face's three slots, in the unit [`MeshEdgeGraphSteps::step`] is budgeted in.
const SLOTS_PER_FACE: usize = 3;

type Slot = (u64, i32);

/// Where the build is.
enum Stage {
    /// Pushing each face's three slots, from `next_face`.
    Fill { next_face: usize },
    /// Sorting the runs of `RUN_SLOTS` in place, from the run starting at `next`.
    SortRuns { next: usize },
    /// Merging pairs of sorted runs `width` long into `scratch`.
    Merge(MergePass),
    /// Walking the sorted slots into the graph, from `next_slot`.
    Walk { next_slot: usize },
    /// The graph is built and answered.
    Done,
}

/// One merge pass's position: the pair of runs starting at `start`, its left run read up to
/// `left`, its right run up to `right`.
struct MergePass {
    width: usize,
    start: usize,
    left: usize,
    right: usize,
}

impl MergePass {
    fn new(width: usize) -> Self {
        Self {
            width,
            start: 0,
            left: 0,
            right: width,
        }
    }
}

/// A mesh's [`MeshEdgeGraph`] being built a step at a time.
pub struct MeshEdgeGraphSteps<'a> {
    mesh: &'a Mesh,
    stage: Stage,
    slots: Vec<Slot>,
    scratch: Vec<Slot>,
    graph: MeshEdgeGraph,
}

impl<'a> MeshEdgeGraphSteps<'a> {
    /// The build of `mesh`'s edge graph, not yet started.
    pub fn new(mesh: &'a Mesh) -> Self {
        let slot_count = mesh.faces.len() * SLOTS_PER_FACE;
        Self {
            mesh,
            stage: Stage::Fill { next_face: 0 },
            slots: Vec::with_capacity(slot_count),
            scratch: Vec::new(),
            graph: MeshEdgeGraph {
                face_indices: Vec::with_capacity(slot_count),
                ..MeshEdgeGraph::default()
            },
        }
    }

    /// Does about `face_budget` faces' worth of the build (at least one piece of it): `Some` with
    /// the graph from the step that finishes it, `None` while there is more to do. A face's worth
    /// is pushing its three slots, sorting them in their run (a few times more), merging them
    /// once per merge pass, or walking them into the graph. Not called again once it has
    /// answered the graph.
    pub fn step(&mut self, face_budget: usize) -> Option<MeshEdgeGraph> {
        let mut work_left = face_budget.max(1).saturating_mul(SLOTS_PER_FACE);
        while work_left > 0 {
            let spent = match &mut self.stage {
                Stage::Done => return None,
                Stage::Fill { next_face } => {
                    let start = *next_face;
                    let end =
                        (start + work_left.div_ceil(SLOTS_PER_FACE)).min(self.mesh.faces.len());
                    for (face_index, face) in self.mesh.faces[start..end].iter().enumerate() {
                        let face_index = (start + face_index) as i32;
                        self.slots.push((edge_key(face.v0, face.v1), face_index));
                        self.slots.push((edge_key(face.v1, face.v2), face_index));
                        self.slots.push((edge_key(face.v2, face.v0), face_index));
                    }
                    *next_face = end;
                    if end == self.mesh.faces.len() {
                        self.stage = Stage::SortRuns { next: 0 };
                    }
                    (end - start) * SLOTS_PER_FACE
                }
                Stage::SortRuns { next } => {
                    let start = *next;
                    let end = (start + RUN_SLOTS).min(self.slots.len());
                    self.slots[start..end].sort_unstable();
                    *next = end;
                    if end == self.slots.len() {
                        self.after_pass(RUN_SLOTS);
                    }
                    (end - start) * RUN_SORT_WORK
                }
                Stage::Merge(pass) => {
                    let merged = merge_some(&self.slots, &mut self.scratch, pass, work_left);
                    if pass.start >= self.slots.len() {
                        std::mem::swap(&mut self.slots, &mut self.scratch);
                        let width = pass.width * 2;
                        self.after_pass(width);
                    }
                    merged
                }
                Stage::Walk { next_slot } => {
                    let start = *next_slot;
                    let end = (start + work_left).min(self.slots.len());
                    for slot in start..end {
                        let key = self.slots[slot].0;
                        if slot == 0 || key != self.slots[slot - 1].0 {
                            self.graph.edge_vertex0.push((key >> 32) as i32);
                            self.graph.edge_vertex1.push((key & 0xFFFF_FFFF) as i32);
                            self.graph.face_starts.push(slot as i32);
                        }
                        self.graph.face_indices.push(self.slots[slot].1);
                    }
                    *next_slot = end;
                    if end == self.slots.len() {
                        self.graph.face_starts.push(self.slots.len() as i32);
                        self.stage = Stage::Done;
                        return Some(std::mem::take(&mut self.graph));
                    }
                    end - start
                }
            };
            // Each piece does at least one unit, so the loop always ends.
            work_left = work_left.saturating_sub(spent.max(1));
        }
        None
    }

    /// The stage after the runs are `width` long and sorted: another merge pass while more than
    /// one run is left, else the walk.
    fn after_pass(&mut self, width: usize) {
        if width >= self.slots.len() {
            self.scratch = Vec::new();
            self.stage = Stage::Walk { next_slot: 0 };
        } else {
            self.scratch.clear();
            self.scratch.reserve(self.slots.len());
            self.stage = Stage::Merge(MergePass::new(width));
        }
    }
}

/// Merges up to `budget` slots of `pass` from `slots` onto the end of `scratch`, pair of runs by
/// pair of runs, taking the left run's slot on a tie; answers how many it merged.
fn merge_some(
    slots: &[Slot],
    scratch: &mut Vec<Slot>,
    pass: &mut MergePass,
    budget: usize,
) -> usize {
    let len = slots.len();
    let mut merged = 0;
    while merged < budget && pass.start < len {
        let mid = (pass.start + pass.width).min(len);
        let end = (pass.start + 2 * pass.width).min(len);
        let take_left =
            pass.left < mid && (pass.right >= end || slots[pass.left] <= slots[pass.right]);
        if take_left {
            scratch.push(slots[pass.left]);
            pass.left += 1;
        } else {
            scratch.push(slots[pass.right]);
            pass.right += 1;
        }
        merged += 1;
        if pass.left >= mid && pass.right >= end {
            pass.start = end;
            pass.left = end;
            pass.right = end + pass.width;
        }
    }
    merged
}
