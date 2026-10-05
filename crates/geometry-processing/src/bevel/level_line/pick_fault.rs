//! What is wrong with a pick ([`PickFault`]), carried from the validation to whichever entry
//! point asked. Ports the record of `Bevel/LevelLine/PickFault.cs`; its conversions to a
//! `BandGapResult` and a `RoomReading` land with those result types.

use super::BandGapRefusal;

/// C# `PickFault`: a refusal and the chains, vertex or edge it is about. Never thrown: every fault
/// here is a statement about the pick and the mesh under it, which leaves by name. The `-1`
/// defaults are C#'s initializers, which is why `Default` is written out rather than derived.
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
