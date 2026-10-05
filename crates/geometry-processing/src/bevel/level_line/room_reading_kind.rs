//! Which of the level-line room's three readings named it. Ports
//! `Bevel/LevelLine/RoomReadingKind.cs`.

/// C# `RoomReadingKind`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum RoomReadingKind {
    /// Nothing bounded the pick, or the room was handed in rather than read.
    #[default]
    None = 0,

    /// A foreign feature edge the band would reach - the room source's chain is whose.
    Foreign,

    /// C# `Self` (a Rust keyword): a side running out of itself - the room source's chain is a
    /// chain that seeds it.
    SelfRoom,

    /// Two chains of one pick closing on each other - the room source's chain reads the other
    /// chain's edges.
    ChainToChain,
}
