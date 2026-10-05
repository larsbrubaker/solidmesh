//! What a room reading found, and what it cost ([`RoomReading`]). Ports the type of
//! `Bevel/LevelLine/RoomReading.cs`; its derived readings (`unbounded`, `is_a_size_statement`) and
//! the fault conversions land with the room-reading method package.

use super::{LevelLineWorkPhase, RoomCandidate, RoomSource};

/// C# `RoomReading`: what a room reading found, and what it cost. The `true`, `-1` and
/// `RoomSource.Nothing` defaults are C#'s initializers, which is why `Default` is written out
/// rather than derived.
///
/// The room answers "how big may this bevel be" only when the reading got through and every named
/// fault is clear. That is the one gate on handing a room to the band build as its known room: the
/// build TRUSTS the number - it refuses a size above it at once, before it reads the mesh at all -
/// so a zero that means "this chain does not divide its sides" handed in as a room turns a refusal
/// the user can act on into "your bevel is too big". Every flag here stands for one of the build's
/// own named refusals, so the two cannot disagree about which picks refuse by name.
#[derive(Debug, Clone)]
pub struct RoomReading {
    /// True when the room was read; false when the reading ran out of work.
    pub read: bool,
    /// The largest size the side's WIDTH allows, or NaN when the reading ran out of work. It is
    /// read off geometry alone - what foreign features bound the picked sides, and how much of
    /// itself each side holds - and says nothing about a side that PINCHES, two stretches of one
    /// chain closing across a neck. A pinching pick therefore refuses BY SIZE somewhere BELOW this
    /// number, naming the smaller room it found. Positive infinity when nothing on this pick bounds
    /// the size within the ceiling worth reading.
    pub room: f64,
    /// False when the chain does not divide its sides - one smooth region wraps both sides of it,
    /// so there are not two sides to set back from. The room is then ZERO, and that zero is NOT a
    /// size statement.
    pub chain_divides_sides: bool,
    /// False when there is no crease under the chain at all: its two sides are one region AND not
    /// one of its edges is a feature. Nothing is rounded at any size, so the room is ZERO rather
    /// than the INFINITY an unbounded region would otherwise read. An open chain whose sides are
    /// one region only ROUND ITS END is the dying crease, and reads a room like any other.
    pub chain_has_a_crease_to_bevel: bool,
    /// False when a chain edge does not stand on exactly two triangles, or its two triangles do not
    /// wind it consistently. The room is then ZERO and that zero is not a size statement.
    pub chain_edge_stands_on_two_triangles: bool,
    /// The offending chain edge, or (-1, -1).
    pub offending_edge: (i32, i32),
    /// False when two chains of the pick share a vertex or an edge. The room is then ZERO and that
    /// zero is not a size statement.
    pub chains_are_apart: bool,
    /// False when one chain of the pick names the same vertex twice. The room is then ZERO and that
    /// zero is not a size statement.
    pub chain_is_simple: bool,
    /// Which of the room's readings named it - see [`RoomSource`]. Kind `None` where nothing
    /// bounded the pick (the room is then positive infinity).
    pub room_set_by: RoomSource,
    /// Every reading the room was the least of, in the order they were read - foreign per chain,
    /// then chain to chain, then the self room per group. Empty for a pick of one chain, whose room
    /// is read by the older single-chain path.
    pub candidates: Vec<RoomCandidate>,
    /// How long each chain of the pick is along its own edges, in the order the chains were given.
    /// Read off the geometry, so it is the same number whatever order a pick was typed in and
    /// whatever rigid motion the part is under - which is what lets the chains-that-cannot-fit
    /// choice between two chains not depend on their indices. Empty for a pick of one chain.
    pub chain_lengths: Vec<f64>,
    /// Which chain of the pick the reading is about, or -1.
    pub offending_chain: i32,
    /// The other chain of a touching pair, or -1.
    pub other_chain: i32,
    /// The vertex two touching chains share, or -1.
    pub offending_vertex: i32,
    /// What was running when the work ran out; [`LevelLineWorkPhase::None`] otherwise.
    pub phase: LevelLineWorkPhase,
    /// How many units of work the reading spent.
    pub work_spent: i64,
    /// How many units it was allowed.
    pub work_limit: i64,
}

impl Default for RoomReading {
    fn default() -> Self {
        RoomReading {
            read: false,
            room: 0.0,
            chain_divides_sides: true,
            chain_has_a_crease_to_bevel: true,
            chain_edge_stands_on_two_triangles: true,
            offending_edge: (-1, -1),
            chains_are_apart: true,
            chain_is_simple: true,
            room_set_by: RoomSource::nothing(),
            candidates: Vec::new(),
            chain_lengths: Vec::new(),
            offending_chain: -1,
            other_chain: -1,
            offending_vertex: -1,
            phase: LevelLineWorkPhase::default(),
            work_spent: 0,
            work_limit: 0,
        }
    }
}
