//! What carries a [`BevelPickMemo`] to another process and back, as plain data a caller can
//! encode: everything it holds ([`MemoSnapshot`], each settling a [`MemoSettling`]), taken and
//! restored by the memo itself (`pick_memo`), and one write it was asked to make
//! ([`MemoWrite`]), which a journaling memo records and [`BevelPickMemo::apply`] replays.
//!
//! Every key is a content key - the mesh's stamp, the profile and the chains' indices - so an
//! entry means the same thing in any process that holds the same mesh.

use std::sync::Arc;

use super::pick_memo::BevelPickMemo;
use super::settled_pick::SettledPick;
use crate::bevel::level_line::RoomSource;

/// Everything a [`BevelPickMemo`] holds, as plain data: one list per kind of entry, each sorted by
/// key, so two memos holding the same entries snapshot alike. Taken by
/// [`BevelPickMemo::snapshot`] and restored by [`BevelPickMemo::from_snapshot`].
///
/// The memo's work counters (`rooms_read`, `settlings_run`) are not in it. They count what was
/// paid for against ONE memo, and a memo restored from a snapshot has paid for nothing yet, so its
/// counts start at zero and remain its own.
#[derive(Debug, Clone, Default)]
pub struct MemoSnapshot {
    /// The content stamp of the mesh every entry was read on, or `None` while the memo is empty.
    pub the_stamp: Option<u64>,
    /// Each pick's room as read, `None` where the reading made no size statement.
    pub rooms: Vec<(String, Option<f64>)>,
    /// Each many-loop pick's settling.
    pub settlings: Vec<(String, MemoSettling)>,
    /// The lowest room a build has proved for each pick.
    pub floors: Vec<(String, f64)>,
    /// The largest size a build has carried for each pick.
    pub built: Vec<(String, f64)>,
    /// The smallest size a build was refused at for each pick, from which its floor holds.
    pub refused_at: Vec<(String, f64)>,
    /// Which reading named each remembered room.
    pub room_sources: Vec<(String, RoomSource)>,
    /// Which reading named each floor.
    pub floor_sources: Vec<(String, RoomSource)>,
    /// The chain the pinch that set each floor named, where it named one.
    pub floor_pinched_chains: Vec<(String, i32)>,
}

/// One remembered settling of a many-loop rim, as a snapshot carries it.
#[derive(Debug, Clone)]
pub struct MemoSettling {
    /// The size the settling was asked at.
    pub size: f64,
    /// What the settling came to.
    pub settled: SettledPick,
    /// The survivors' room, as the settling's last reading read it, or `None` where it made no
    /// size statement.
    pub room: Option<f64>,
}

/// One write a memo was asked to make: a variant per writing method, carrying that method's
/// arguments as owned data. A journaling memo ([`BevelPickMemo::journaling`]) records one for each
/// write it makes, in the order it made them; [`BevelPickMemo::apply`] makes the same write on
/// another memo. Each variant names its method in plain code rather than linking it, because the
/// writing methods are the kernel's own and are kept out of the rendered docs (`#[doc(hidden)]`).
#[derive(Debug, Clone)]
pub enum MemoWrite {
    /// `BevelPickMemo::remember_the_room`.
    RememberTheRoom {
        stamp: u64,
        key: String,
        room: Option<f64>,
        set_by: RoomSource,
    },
    /// `BevelPickMemo::remember_a_smaller_room`.
    RememberASmallerRoom {
        stamp: u64,
        key: String,
        room: f64,
        set_by: RoomSource,
        pinched_chain: i32,
        refused_at_size: Option<f64>,
    },
    /// `BevelPickMemo::remember_a_size_that_built`.
    RememberASizeThatBuilt { stamp: u64, key: String, size: f64 },
    /// `BevelPickMemo::remember_the_settling`.
    RememberTheSettling {
        stamp: u64,
        key: String,
        size: f64,
        settled: SettledPick,
        room: Option<f64>,
    },
    /// `BevelPickMemo::carry_the_settling_to`.
    CarryTheSettlingTo {
        stamp: u64,
        key: String,
        size: f64,
        settled: SettledPick,
        room: Option<f64>,
    },
    /// [`BevelPickMemo::forget`].
    Forget,
}

impl BevelPickMemo {
    /// Make a write another memo journaled, through the same method that made it there - so a
    /// floor only goes down, a built size only up, a new stamp empties what the old one read and
    /// the work counters count it, exactly as if the write had been made here. A journaling memo
    /// journals a write it is asked to apply, as it does any other.
    pub fn apply(&self, write: &MemoWrite) {
        match write {
            MemoWrite::RememberTheRoom {
                stamp,
                key,
                room,
                set_by,
            } => self.remember_the_room(*stamp, key, *room, *set_by),
            MemoWrite::RememberASmallerRoom {
                stamp,
                key,
                room,
                set_by,
                pinched_chain,
                refused_at_size,
            } => self.remember_a_smaller_room(
                *stamp,
                key,
                *room,
                *set_by,
                *pinched_chain,
                *refused_at_size,
            ),
            MemoWrite::RememberASizeThatBuilt { stamp, key, size } => {
                self.remember_a_size_that_built(*stamp, key, *size)
            }
            MemoWrite::RememberTheSettling {
                stamp,
                key,
                size,
                settled,
                room,
            } => self.remember_the_settling(*stamp, key, *size, Arc::new(settled.clone()), *room),
            MemoWrite::CarryTheSettlingTo {
                stamp,
                key,
                size,
                settled,
                room,
            } => self.carry_the_settling_to(*stamp, key, *size, Arc::new(settled.clone()), *room),
            MemoWrite::Forget => self.forget(),
        }
    }
}
