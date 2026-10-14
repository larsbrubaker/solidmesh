//! Which of the level-line room's three readings named it.

/// Which reading named the room, or `None` where nothing did.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum RoomReadingKind {
    /// Nothing bounded the pick, or the room was handed in rather than read.
    #[default]
    None = 0,

    /// A foreign feature edge the band would reach - the room source's chain is whose.
    Foreign,

    /// A side running out of itself (`Self` is a Rust keyword) - the room source's chain is a
    /// chain that seeds it.
    SelfRoom,

    /// Two chains of one pick closing on each other - the room source's chain reads the other
    /// chain's edges.
    ChainToChain,
}
