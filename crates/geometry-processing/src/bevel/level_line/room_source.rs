//! Which reading set a band's room, and which chain or pair of chains it was about. Leans on
//! [`RoomReadingKind`] for the kind; the room readings produce one.

use vector_math::Vector3;

use super::RoomReadingKind;

/// Which reading set the room, and which chain or pair of chains it was about. Useful to a message
/// a user reads ("the hole's rim is only 0.3 from the plate's edge"), and to a test that means to
/// pin WHY a room is what it is and not merely its number (a plain value; equality compares every
/// field, `crease_at` included).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RoomSource {
    /// Which reading named the room.
    pub kind: RoomReadingKind,
    /// The chain the reading was about, or -1.
    pub chain: i32,
    /// The other chain of a chain-to-chain reading, or -1.
    pub other_chain: i32,
    /// Where the crease that set a growing-crease room stands nearest the chain, or `None` for
    /// every other reading. A flag on [`RoomReadingKind::Foreign`] rather than a kind of its own,
    /// so every gate that reads Foreign - the existing construction's retry past the level line's
    /// room above all - reads a growing-crease room exactly as it did, and only the sentence a user
    /// reads tells the two apart.
    pub crease_at: Option<Vector3>,
}

impl RoomSource {
    /// A source with no crease position (`crease_at` is set afterwards by the one reading that has
    /// one).
    pub fn new(kind: RoomReadingKind, chain: i32, other_chain: i32) -> Self {
        RoomSource {
            kind,
            chain,
            other_chain,
            crease_at: None,
        }
    }

    /// The reading that names nothing - no chain, no pair.
    pub fn nothing() -> Self {
        Self::new(RoomReadingKind::None, -1, -1)
    }

    /// A foreign feature edge bounding chain `chain`'s band.
    pub fn foreign(chain: i32) -> Self {
        Self::new(RoomReadingKind::Foreign, chain, -1)
    }

    /// A foreign reading whose edge is a crease AWAY from the picked face: the far boundary of a
    /// neighbour that grows over the crease the band first reaches (`GrowingCreaseRuns.Bounds`),
    /// standing nearest the chain at `at`.
    pub fn growing_crease(chain: i32, at: Vector3) -> Self {
        RoomSource {
            crease_at: Some(at),
            ..Self::new(RoomReadingKind::Foreign, chain, -1)
        }
    }

    /// True when the room was set by a growing neighbour's far crease - see `crease_at`.
    pub fn is_a_growing_crease(&self) -> bool {
        self.kind == RoomReadingKind::Foreign && self.crease_at.is_some()
    }

    /// A side group running out of itself, named by the first chain that seeds it.
    pub fn self_room(chain: i32) -> Self {
        Self::new(RoomReadingKind::SelfRoom, chain, -1)
    }

    /// Chain `chain`'s band closing on chain `other`'s.
    pub fn chain_to_chain(chain: i32, other: i32) -> Self {
        Self::new(RoomReadingKind::ChainToChain, chain, other)
    }
}
