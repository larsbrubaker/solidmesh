//! What is wrong with a pick ([`PickFault`]), carried from the validation to whichever entry
//! point asked: the record (its initial values are `Default`) and its
//! conversions to a [`BandGapResult`] and a [`RoomReading`], which read the budget's totals but
//! charge nothing.

use super::{BandGapRefusal, BandGapResult, RoomReading, WorkBudget};

/// A refusal and the chains, vertex or edge it is about. Never thrown: every fault
/// here is a statement about the pick and the mesh under it, which leaves by name. The `-1`
/// defaults mean "none named", which is why `Default` is written out rather than derived.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PickFault {
    /// Why the pick was refused.
    pub refusal: BandGapRefusal,
    /// The chain the fault is about, or -1.
    pub chain: i32,
    /// The other chain, where the fault is between two, or -1.
    pub other: i32,
    /// The vertex the fault is at, or -1.
    pub vertex: i32,
    /// The mesh edge the fault is on, as its two vertex indices, or (-1, -1).
    pub edge: (i32, i32),
}

impl Default for PickFault {
    fn default() -> Self {
        PickFault {
            refusal: BandGapRefusal::default(),
            chain: -1,
            other: -1,
            vertex: -1,
            edge: (-1, -1),
        }
    }
}

impl PickFault {
    /// The fault as the band gap's answer: refused for its reason, with no room, naming what it
    /// found and what the build spent of `budget`.
    pub fn as_band_gap(&self, budget: &WorkBudget) -> BandGapResult {
        BandGapResult {
            refusal: self.refusal,
            room: 0.0,
            offending_chain: self.chain,
            other_chain: self.other,
            offending_vertex: self.vertex,
            offending_edge: self.edge,
            work_spent: budget.spent.get(),
            work_limit: budget.limit,
            ..BandGapResult::default()
        }
    }

    /// The fault as a room reading. A fault is an ANSWER - no size builds this pick - and not a
    /// reading that failed to finish, so it reads, with no room and the pick's own condition that
    /// failed named.
    pub fn as_room_reading(&self, budget: &WorkBudget) -> RoomReading {
        RoomReading {
            read: true,
            room: 0.0,
            chain_edge_stands_on_two_triangles: self.refusal
                != BandGapRefusal::ChainEdgeNotOnTwoTriangles,
            chains_are_apart: self.refusal != BandGapRefusal::ChainsTouch,
            chain_is_simple: self.refusal != BandGapRefusal::ChainTouchesItself,
            offending_chain: self.chain,
            other_chain: self.other,
            offending_vertex: self.vertex,
            offending_edge: self.edge,
            work_spent: budget.spent.get(),
            work_limit: budget.limit,
            ..RoomReading::default()
        }
    }
}
