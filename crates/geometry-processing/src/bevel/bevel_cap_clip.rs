//! Clipping a cutter whose run ended on a cap plane the sweep could not carry
//! (`Bevel/BevelCapClip.cs`, a C# `internal static class`, so a module here): one cap a boolean
//! owes ([`ClippedCap`]), one cutter with the caps to take off it ([`Candidate`]) and a cutter
//! that could not be clipped and was withdrawn ([`Withdrawal`]). The clip itself lands with the
//! core's packages as free functions in this module.

use vector_math::Vector3;

use super::{BevelAdvisory, BevelEndCap};

/// C# `BevelCapClip.ClippedCap`: one cap the boolean owes - the plane to subtract, and the
/// vertex the run ends at, which is what the refusal has to name if the subtraction cannot be
/// made.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct ClippedCap {
    /// The half space the tool must stay out of, in the cutter's own space.
    pub plane: BevelEndCap,
    /// Where the run ends, for the message.
    pub at: Vector3,
}

/// C# `BevelCapClip.Candidate`: one cutter and the cap planes to take off it.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Candidate {
    /// Where this cutter is in the builder's cutter list.
    pub cutter_index: i32,
    /// Which group to fail if the clip cannot be made.
    pub group_index: i32,
    /// The operation's bevel size, which the refusal quotes.
    pub size: f64,
    /// The caps to subtract. Never empty.
    pub clips: Vec<ClippedCap>,
}

/// C# `BevelCapClip.Withdrawal`: a cutter that could not be clipped and has been taken out of
/// the batch, with what to say on its group's row.
#[derive(Debug, Clone)]
pub struct Withdrawal {
    /// The group whose cutter was withdrawn.
    pub group_index: i32,
    /// The refusal to put on that group's row.
    pub reason: BevelAdvisory,
}
