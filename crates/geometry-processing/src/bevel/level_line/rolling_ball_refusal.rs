//! Why the rolling ball refused a pass (`rolling_ball_setbacks::with_the_sides`), as the words the
//! route and the user's row carry and as a kind the build can act on. The kind is set where the
//! words are written (`rolling_ball_legs` for the walk and its end stations, every other refusal
//! `Other`), so nothing downstream reads meaning back out of the text: a refused pass of one
//! chain searches smaller sizes for the ball's own room only where the kind says the size asked
//! is too big (`rolling_ball_size_room`).

use crate::bevel::rolling_ball::{EndContactFault, StationFailure, StationFault, WalkError};

/// A refusal of the ball: `why` in words, and what kind of refusal it is.
#[derive(Clone, Debug, PartialEq)]
pub struct BallRefusal {
    /// The reason, as the route and the user's row carry it.
    pub why: String,
    /// What kind of refusal it is.
    pub kind: RefusalKind,
}

/// What kind of refusal a [`BallRefusal`] is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RefusalKind {
    /// A station's ball whose round reaches past the part (`StationFault::ReachesPastThePart`).
    ReachesPastThePart,
    /// An end station's contact that is no setback for the round (`EndContactFault`).
    EndContact(EndContactFault),
    /// Anything else: an unsolved or jumping station, a rail that misses its end face, sides that
    /// differ where they may not, a pass with no sides to bound.
    Other,
}

impl RefusalKind {
    /// The kind of a walk's error: a station whose round reaches past the part, or `Other`.
    pub fn of_the_walk(error: &WalkError) -> RefusalKind {
        match error {
            WalkError::At {
                failure:
                    StationFailure::Fault {
                        fault: StationFault::ReachesPastThePart { .. },
                        ..
                    },
                ..
            } => RefusalKind::ReachesPastThePart,
            _ => RefusalKind::Other,
        }
    }

    /// Whether the refusal says the size asked is too big, so a smaller size may walk: the round
    /// reaching past the part, or an end station's contact resting on its side's own rim without
    /// touching the side square there (a size past the side's width). A contact on a hole of the
    /// side's extension is no matter of size.
    pub fn says_the_size_is_too_big(self) -> bool {
        matches!(
            self,
            RefusalKind::ReachesPastThePart
                | RefusalKind::EndContact(EndContactFault::OffTheSidesFaces)
        )
    }
}

impl BallRefusal {
    /// A refusal of kind [`RefusalKind::Other`].
    pub fn other(why: String) -> BallRefusal {
        BallRefusal {
            why,
            kind: RefusalKind::Other,
        }
    }
}

impl From<String> for BallRefusal {
    fn from(why: String) -> BallRefusal {
        BallRefusal::other(why)
    }
}
