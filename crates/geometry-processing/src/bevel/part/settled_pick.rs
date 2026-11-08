//! What settling a many-loop pick came to ([`SettledPick`]): which chains can be built at the
//! size, which cannot and why, from ONE read of who cannot be built (`PickThatFits`) rather than
//! a reading per chain. The pick memo (`pick_memo`) keeps it across rebuilds.

use crate::bevel::level_line::{ChainCannotFit, RoomReading};

/// Which chains of the pick can be built at the size, which cannot and why, and
/// the reading that says so - all of it in the caller's own chain numbering.
#[derive(Debug, Clone)]
pub struct SettledPick {
    /// What is left of the pick, as indices into the chains the caller gave.
    pub chains: Vec<i32>,

    /// Every chain that cannot stay, with the reason and the room it is named with, by chain.
    pub condemned: Vec<ChainCannotFit>,

    /// Side groups no reading could blame - the caller falls back to one at a time for these.
    pub unresolved: Vec<Vec<i32>>,

    /// The last reading taken, of `chains`, or `None` where nothing was read.
    pub last: Option<RoomReading>,

    /// Whether what is left really does read at or above the size with nothing unresolved.
    pub fits: bool,

    /// How many room readings the settling paid for - the number settling exists to hold down.
    pub readings: i32,
}
