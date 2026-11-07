//! One of the readings the room is the least of, as a SIZE - the margin already taken off, so the
//! room is exactly the least candidate and a test or a message can say why it is that and not
//! something larger. Leans on [`RoomSource`] for which reading it is.
//!
//! [`RoomCandidate::room_without_the_other`] is derived, never re-measured, and is deliberately
//! an UNDER-read where the setback varies along either chain: it is taken as
//! `pair room * (1 + least u_j / greatest u_k)`, and since the pair room is itself an under-read
//! (the foreign-edge room) and the smallest possible ratio is used, the product can only fall
//! short of the room k truly gains. A uniform setback - every square-cornered rim, and every
//! chamfer - makes it exact, and [`RoomCandidate::room_without_the_other_at_most`] is then the
//! same number; where a setback varies the two part company and the over-read is simply not
//! known, so it reads positive infinity.
//!
//! NEITHER number says a survivor fits. Both bound one chain against ONE other chain's edges.
//! Dropping a chain re-tags the sides, which can make edges BEHIND it newly foreign to a
//! survivor; see `ChainsThatCannotFit` and `PickThatFits`, which re-reads until a reading names
//! nobody.
//!
//! No `PartialEq`: a derived equality would read a NaN room as unequal to itself, which is not
//! what two copies of one candidate are, and nothing compares candidates whole.

use super::RoomSource;

/// One room candidate. [`Self::new`] gives the optional fields their usual values, and a caller
/// sets the others with struct update syntax.
#[derive(Debug, Clone)]
pub struct RoomCandidate {
    /// Which reading this is, and the chain or pair it is about.
    pub source: RoomSource,

    /// The size this reading allows, the margin already off.
    pub room: f64,

    /// For a `RoomReadingKind::ChainToChain` pair (k, j) only: the size k would have if j were
    /// DROPPED from the pick. Dropping j turns j's crease from a band with a setback of its own
    /// into plain foreign edges for k, so what the closest approach buys stops being shared and
    /// k's room rises from `approach / (u_k + u_j)` to `approach / u_k`. NaN on every other kind
    /// of reading.
    pub room_without_the_other: f64,

    /// The MOST that same room could be - an over-read, so a chain whose value here is still under
    /// a size truly cannot be built at it whoever else leaves. This is the number a chain may be
    /// CONDEMNED on; `room_without_the_other`, being an under-read, would condemn chains that fit.
    /// Positive infinity where it is not known, which means "may stay, and a re-read decides". It
    /// is an over-read of the TRUE room only because a dropped chain's crease is a FEATURE edge
    /// that stays in the mesh. A picked chain that is not a feature edge is the exception:
    /// dropping it merges the regions either side, and the survivor's true room can then stand
    /// ABOVE this number.
    pub room_without_the_other_at_most: f64,

    /// For a `RoomReadingKind::SelfRoom` reading only: every chain that seeds the group this
    /// reading is about. A group of ONE names the chain that must go when the reading is under the
    /// size; a group of several cannot be blamed from one reading at all. `None` on
    /// every other kind.
    pub group_members: Option<Vec<i32>>,
}

impl RoomCandidate {
    /// A candidate with the usual values for the optional fields: no room without the other (NaN),
    /// an unknown most (positive infinity) and no group.
    pub fn new(source: RoomSource, room: f64) -> Self {
        Self {
            source,
            room,
            room_without_the_other: f64::NAN,
            room_without_the_other_at_most: f64::INFINITY,
            group_members: None,
        }
    }
}
